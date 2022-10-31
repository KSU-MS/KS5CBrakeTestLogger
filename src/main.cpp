#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <SdFat.h>
#include <SimpleKalmanFilter.h>
#include <SPI.h>
#include <mcp_can.h>
#include <Adafruit_GPS.h>
#include <Metro.h>
#include <KSU_GPS.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> CAN;
static CAN_message_t msg_tx;
Metro GPS_out_timer = Metro(200);
Metro GPS_parse_timer = Metro(100);
Metro shonks_out_timer = Metro(100);
Metro shonks_parse_timer = Metro(1);
SimpleKalmanFilter shonkFL(1,1,0.01); //front left shonk
SimpleKalmanFilter shonkFR(1,1,0.01); //front right shonk
SimpleKalmanFilter shonkRL(1,1,0.01); //rear left shonk
SimpleKalmanFilter shonkRR(1,1,0.01); //rear right shonk

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

// Adafruit_GPS GPS(&Serial4);
void process_gps();
static bool pending_gps_data;
MCU_GPS_readings mcu_gps_readings;
//adc variables
//space out bt using pins 0 and 3 to avoid short on board
int16_t adc0,adc1,adc2,adc3;
//0-3 is purple, blue,yellow,orange
//0-RR
//1-FL
//2-FR
//3-RL
float volts0,volts1,volts2, volts3;
//string for writing to SD
#ifdef SD_LOGGING
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN
#define SPI_CLOCK SD_SCK_MHZ(50)
//fuck

// Try to select the best SD card configuration.

#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif  ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif  // HAS_SDIO_CLASS
#define SD_FAT_TYPE 1
#if SD_FAT_TYPE == 0
SdFat sd;
typedef File file_t;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
typedef File32 file_t;
#elif SD_FAT_TYPE == 2
SdExFat sd;
typedef ExFile file_t;
#elif SD_FAT_TYPE == 3
SdFs sd;
typedef FsFile file_t;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE
#define FILE_BASE_NAME "ShonkPotLog"
file_t myFile;
const uint8_t BASE_NAME_SIZE=sizeof(FILE_BASE_NAME)-1;
char fileName[] = FILE_BASE_NAME "00.txt";
#endif
//Stuff for CAN sending
byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
// MCP_CAN CAN0(10);
void setup() {
  digitalWrite(LED_BUILTIN,HIGH);
  delay(2000);
  // GPS.begin(9600);
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // specify data to be received (minimum + fix)
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // set update rate (10Hz)
  // GPS.sendCommand(PGCMD_ANTENNA); // report data about antenna
  //picking pin 5
#ifdef SD_LOGGING
  if (!sd.begin(SD_CONFIG)) {
    Serial.println("tuff");
    //digitalWrite(ledPin,HIGH);
    sd.initErrorHalt(&Serial);
  }
  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      Serial.println(F("Can't create file name"));
      return;
    }
  }
  if (!myFile.open(fileName, FILE_WRITE)) {
    sd.errorHalt("opening test.txt for write failed");
  }
  //myFile.open(fileName);
  myFile.println(fileName);
  myFile.println("Time,FrontLeft,FrontRight,RearLeft,RearRight");
  #endif
  delay(10);
  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV
  //initialize the ADC
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    digitalWrite(LED_BUILTIN,HIGH);
    while (1);
  }
    CAN.begin();
    CAN.setBaudRate(1000000);
}

void loop() {
  digitalWrite(LED_BUILTIN,LOW);
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);
  int16_t estimated_FL=shonkFL.updateEstimate(adc1);
  int16_t estimated_FR=shonkFR.updateEstimate(adc2);
  int16_t estimated_RL=shonkRL.updateEstimate(adc3);
  int16_t estimated_RR=shonkRR.updateEstimate(adc0);
  float now = millis();
  float timestamp = now/1000;
  char buffer[80];
  sprintf(buffer,"%f,%d,%d,%d,%d\n",timestamp,estimated_FL,estimated_FR,estimated_RL,estimated_RR);
  // GPS.read();
  // if (GPS.newNMEAreceived()) {
  //   GPS.parse(GPS.lastNMEA());
  //   pending_gps_data = true;
  // }
  // if (GPS_out_timer.check() && pending_gps_data) {
  //   process_gps();
  // }
  if(shonks_out_timer.check()){
    digitalWrite(LED_BUILTIN,HIGH);
#ifdef SD_LOGGING
    myFile.println("Time,FrontLeft,FrontRight,RearLeft,RearRight");
    n=sprintf(buffer,"Time\t%f\tLeft wheel\t%f\tRight wheel\t%f\tBrake 1\t%fBrake 2\t%f\n",timestamp,current_rpm2,current_rpm,volts0,volts3);
    myFile.open(fileName,FILE_WRITE);
    myFile.write(buffer);
    myFile.close();
    Serial.print(buffer);
#endif
    Serial.print(buffer);
    memcpy(&msg_tx.buf[0],&estimated_FL,sizeof(estimated_FL));
    memcpy(&msg_tx.buf[2],&estimated_FR,sizeof(estimated_FR));
    memcpy(&msg_tx.buf[4],&estimated_RL,sizeof(estimated_RL));
    memcpy(&msg_tx.buf[6],&estimated_RR,sizeof(estimated_RR));
    msg_tx.id = 0xC5;
    msg_tx.len = 8;
    CAN.write(msg_tx);
  }
}
// void process_gps() {
//     mcu_gps_readings.set_latitude(GPS.latitude_fixed);
//     mcu_gps_readings.set_longitude(GPS.longitude_fixed);
//     Serial.print("Latitude (x100000): ");
//     Serial.println(mcu_gps_readings.get_latitude());
//     Serial.print("Longitude (x100000): ");
//     Serial.println(mcu_gps_readings.get_longitude());
//     uint8_t gps_buf[8];
//     mcu_gps_readings.write(gps_buf);
//     byte sndStat = CAN0.sendMsgBuf(0xE7, 0, 8, gps_buf);
//     if(sndStat == CAN_OK){
//       Serial.println("GPS Message Sent Successfully!");
//     } else {
//       Serial.println("Error Sending Message...");
//     }
//     pending_gps_data = false;
// }