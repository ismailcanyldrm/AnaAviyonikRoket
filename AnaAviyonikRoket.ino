/*#if !defined(ARDUINO_ARCH_RP2040)
  #error For RP2040 only
#endif
#if defined(ARDUINO_ARCH_MBED)  
  #define PIN_SD_MOSI       PIN_SPI_MOSI
  #define PIN_SD_MISO       PIN_SPI_MISO
  #define PIN_SD_SCK        PIN_SPI_SCK
  #define PIN_SD_SS         PIN_SPI_SS
#else
  #define PIN_SD_MOSI       PIN_SPI0_MOSI
  #define PIN_SD_MISO       PIN_SPI0_MISO
  #define PIN_SD_SCK        PIN_SPI0_SCK
  #define PIN_SD_SS         PIN_SPI0_SS 
#endif
#define _RP2040_SD_LOGLEVEL_       */
int t1 = 14;
int t2 = 3;
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU9250_asukiaaa.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <MovingAverageFilter.h>
#include <SPI.h>
//#include <RP2040_SD.h>

#define SEALEVELPRESSURE_HPA (877.20)
float aX, aY, aZ, gX, gY, gZ, mX, mY, mZ;
static void smartdelay(unsigned long ms);

Adafruit_BME280 bme;
MPU9250_asukiaaa mySensor;
TinyGPS gps;
SoftwareSerial ss(11,12);
SoftwareSerial lora(1,0);
//File myFile;

MovingAverageFilter mvg00(20);
MovingAverageFilter mvg01(20);
MovingAverageFilter mvg02(20);
MovingAverageFilter mvg03(20);

MovingAverageFilter mvg10(20);
MovingAverageFilter mvg11(20);
MovingAverageFilter mvg12(20);

MovingAverageFilter mvg20(20);
MovingAverageFilter mvg21(20);
MovingAverageFilter mvg22(20);

MovingAverageFilter mvg30(20);
MovingAverageFilter mvg31(20);
MovingAverageFilter mvg32(20);

void setup() {
  pinMode(25 , OUTPUT);
  pinMode(t1,OUTPUT);
  pinMode(t2, OUTPUT);
  buzzer(10, 50);
  
  Serial.begin(9600);
  bme.begin(0x76);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  ss.begin(9600);
  lora.begin(9600);
 // SD.begin(PIN_SD_SS);
  #define fileName  "anah.txt"
  
  
  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
}

void loop() 
{
  
  mpu9250();
  bmp280();
  gpsKonum();
  gpsSatellite();
  smartdelay(500);
}

void bmp280()
{
    float T  = bme.readTemperature();
    float P = bme.readPressure() / 100.0F;
    float A = bme.readAltitude(SEALEVELPRESSURE_HPA);
    float H = bme.readHumidity();
  
    float avgT = mvg00.process(T);
    float avgP = mvg01.process(P);
    float avgA = mvg02.process(A);
    float avgH = mvg03.process(H);
    
    Serial.println(avgT);
    Serial.println(avgP);
    Serial.println(avgA);
    Serial.println(avgH);
   /* myFile = SD.open(fileName, FILE_WRITE);
    myFile.println(avgT);
    myFile.println(avgP);
    myFile.println(avgA);
    myFile.println(avgH);
    myFile.close();*/

}

void mpu9250()
{
    float result;
  result = mySensor.accelUpdate();
    float aX = mySensor.accelX();
    float aY = mySensor.accelY();
    float aZ = mySensor.accelZ();

    float avgaX = mvg10.process(aX);
    float avgaY = mvg11.process(aY);
    float avgaZ = mvg12.process(aZ);

  result = mySensor.gyroUpdate();
    float gX = mySensor.gyroX();
    float gY = mySensor.gyroY();
    float gZ = mySensor.gyroZ();

    float avggX = mvg20.process(gX);
    float avggY = mvg21.process(gY);
    float avggZ = mvg22.process(gZ);

  result = mySensor.magUpdate();
    float mX = mySensor.magX();
    float mY = mySensor.magY();
    float mZ = mySensor.magZ();

    float avgmX = mvg30.process(mX);
    float avgmY = mvg31.process(mY);
    float avgmZ = mvg32.process(mZ);

    Serial.println(aX);
    Serial.println(aY);
    Serial.println(aZ);
    Serial.println(gX);
    Serial.println(gY);
    Serial.println(gZ);
    Serial.println(mX);
    Serial.println(mY);
    Serial.println(mZ);
  /*  myFile = SD.open(fileName, FILE_WRITE);
    myFile.println(avgaX);
    myFile.println(avgaY);
    myFile.println(avgaZ);
    myFile.println(avggX);
    myFile.println(avggY);
    myFile.println(avggZ);
    myFile.println(avgmX);
    myFile.println(avgmY);
    myFile.println(avgmZ);
    myFile.close();*/
}

void gpsKonum() {
  float flat, flon, invalid;
  gps.f_get_position(&flat, &flon);
  invalid = TinyGPS::GPS_INVALID_F_ANGLE;
  
  if (flat == invalid || flon == invalid) {
    lora.println(float(0),6);
    lora.println(float(0),6);
  } else {
    lora.println(float(flat),6);
    lora.println(float(flon),6);
  /*  myFile = SD.open(fileName, FILE_WRITE);
    myFile.println(float(flat),6);
    myFile.println(float(flon),6);
    myFile.close();*/
  }
}
void gpsSatellite() {
  float satellite, invalid;
  satellite = gps.satellites();
  invalid   = TinyGPS::GPS_INVALID_SATELLITES;
  if (satellite == invalid) {
    lora.println(float(0.00));
  } else {
    lora.println(satellite);
/*    myFile = SD.open(fileName, FILE_WRITE);
    myFile.println(satellite);
    myFile.close();*/
    
  }
}
static void smartdelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
      
  } while (millis() - start < ms);
}

void buzzer(int tekrar, int sure) {
  for (int i = 0; i < tekrar; i++) {
    digitalWrite(25, HIGH);
    delay(sure);
    digitalWrite(25, LOW);
    delay(sure);
  }
}
