#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <SdFat.h>
#include <SimpleKalmanFilter.h>
SimpleKalmanFilter shonkFL(1,1,0.01); //front left shonk
SimpleKalmanFilter shonkFR(1,1,0.01); //front right shonk
SimpleKalmanFilter shonkRL(1,1,0.01); //rear left shonk
SimpleKalmanFilter shonkRR(1,1,0.01); //rear right shonk
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
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
String dataLine;
unsigned long log_rate=0;
#define SPI_CLOCK SD_SCK_MHZ(50)
//fuck
const int ledPin = 13;
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
//Stuff for talking to esp8266
void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  delay(10);

  // picking pin 5

  if (!sd.begin(SD_CONFIG)) {
    digitalWrite(ledPin,HIGH);
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
   //Serial.begin(57600);
  //while (!Serial) ; // wait for Arduino Serial Monitor
  delay(10);
  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
}

void loop() {
  digitalWrite(ledPin, LOW);
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);
  int16_t estimated_FL=shonkFL.updateEstimate(adc1);
  int16_t estimated_FR=shonkFR.updateEstimate(adc2);
  int16_t estimated_RL=shonkRL.updateEstimate(adc3);
  int16_t estimated_RR=shonkRR.updateEstimate(adc0);

 //0-RR
//1-FL
//2-FR
//3-RL
float now = millis();
float timestamp = now/1000;
char buffer[80];
  int n;
  n=sprintf(buffer,"%f,%d,%d,%d,%d\n",timestamp,estimated_FL,estimated_FR,estimated_RL,estimated_RR);
if(millis()-log_rate>=100){
  log_rate=millis();
  digitalWrite(ledPin, HIGH);
  
  //  myFile.println("Time,FrontLeft,FrontRight,RearLeft,RearRight");
  //n=sprintf(buffer,"Time\t%f\tLeft wheel\t%f\tRight wheel\t%f\tBrake 1\t%fBrake 2\t%f\n",timestamp,current_rpm2,current_rpm,volts0,volts3);
  myFile.open(fileName,FILE_WRITE);
  myFile.write(buffer);
  myFile.close();
  Serial.print(buffer);
  }
}