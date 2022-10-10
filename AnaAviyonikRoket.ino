#if !defined(ARDUINO_ARCH_RP2040)
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

#define _RP2040_SD_LOGLEVEL_       4

#include <SPI.h>
#include <RP2040_SD.h>
#include <MPU9250_asukiaaa.h>
#include <BME280I2C.h>
#include <movingAvg.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <MovingAverageFilter.h>

MovingAverageFilter movingAverageFilter(20);

#define SERIAL_BAUD 115200

BME280I2C bme; 
#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 4
#define SCL_PIN 5
#endif

TinyGPS gps;
SoftwareSerial serialgps(8,9);

MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long chars;
unsigned short sentences, failed_checksum;
File sdkart;

float b[3] = {0.0,0.0,0.0};
float a[3] = {0.0,0.0,0.0};
float g[3] = {0.0,0.0,0.0};
float angle[3] = {0.0,0.0,0.0};

int gpsint[3] = {0,0,0}; 
double gpsdouble[4] = {0.0,0.0,0.0,0.0}; 
float gpsfloat[5] = {0.0,0.0,0.0,0.0,0.0}; 

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("started");
      serialgps.begin(115200);

        Serial.println("");
        Serial.println(" ...Uydu Bağlantısı Bekleniyor... ");
        Serial.println("");
        
#ifdef MPU9250_I2C_ADR 0x68 // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
#endif

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
  while(!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }
  #if defined(ARDUINO_ARCH_MBED)
  Serial.print("Starting SD Card ReadWrite on MBED ");
#else
  Serial.print("Starting SD Card ReadWrite on ");
#endif
  
  Serial.println(BOARD_NAME);
  Serial.println(RP2040_SD_VERSION);
  
  Serial.print("Initializing SD card with SS = ");  Serial.println(PIN_SD_SS);
  Serial.print("SCK = ");   Serial.println(PIN_SD_SCK);
  Serial.print("MOSI = ");  Serial.println(PIN_SD_MOSI);
  Serial.print("MISO = ");  Serial.println(PIN_SD_MISO);

  if (!SD.begin(PIN_SD_SS)) 
  {
    Serial.println("Initialization failed!");
    return;
  }
  
  Serial.println("Initialization done.");

}
void loop() {

   Serial.println("BME280 VERİLERİ");
   printBME280Data(&Serial);



 sdkart = SD.open("AnaAviyonik.txt",FILE_WRITE);

 if(sdkart)
 {
  sdkart.println("BME 280 VERİLERİ");
  sdkart.print("Basınç değeri: ");sdkart.print(b[0]);
  sdkart.print(" , ");
  sdkart.print("Nem değeri: ");sdkart.print(b[1]);
  sdkart.print(" , ");
  sdkart.print("Sıcaklık değeri: ");sdkart.print(b[2]);
  sdkart.println(" ");
  sdkart.println("İVME VERİLERİ");
  sdkart.print("İvmeX: ");sdkart.print(a[0]);
  sdkart.print(" , ");
  sdkart.print("İvmeY: ");sdkart.print(a[1]);
  sdkart.print(" , ");
  sdkart.print("İvmeZ: ");sdkart.print(a[2]);
  sdkart.println(" ");
  sdkart.println("GYRO VERİLERİ");
  sdkart.print("GyroX: ");sdkart.print(g[0]);
  sdkart.print(" , ");
  sdkart.print("GyroY: ");sdkart.print(g[1]);
  sdkart.print(" , ");
  sdkart.print("GyroZ: ");sdkart.print(g[2]);
  sdkart.println(" ");
  sdkart.println("GPS VERİLERİ");
  sdkart.print("Enlem: ");sdkart.print(gpsfloat[3]);
  sdkart.print(" , ");
  sdkart.print("Boylam: ");sdkart.print(gpsfloat[4]);
  sdkart.print(" , ");
  sdkart.print("Gün: ");sdkart.print(gpsint[0]);
  sdkart.print(" ");
  sdkart.print("Ay: ");sdkart.print(gpsint[1]);
  sdkart.print(" , ");
  sdkart.print("Yıl: ");sdkart.print(gpsint[2]);
  sdkart.print(" , ");
  sdkart.print("Saat: ");sdkart.print(gpsdouble[0]);
  sdkart.print(" , ");
  sdkart.print("Dakika: ");sdkart.print(gpsdouble[1]);
  sdkart.print(" , ");
  sdkart.print("Saniye: ");sdkart.print(gpsdouble[2]);
  sdkart.print(" , ");
  sdkart.print("Salise: ");sdkart.print(gpsdouble[3]);
  sdkart.print(" ");
  sdkart.print("Yükseklik: ");sdkart.print(gpsfloat[0]);
  sdkart.print(" , ");
  sdkart.print("Doğrusal Hız: ");sdkart.print(gpsfloat[2]);
  sdkart.print(" , ");
  sdkart.print("Açı: ");sdkart.print(gpsfloat[1]);
  sdkart.println(" ");
  
  sdkart.close();  
 }
 else
 {
  Serial.println("AnaAviyonik.txt açılamadı.");
 }
   
  delay(500);
}
void printBME280Data
(
   Stream* client
)
{
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);
   
    float avgP = movingAverageFilter.process(pres);
    float avgN = movingAverageFilter.process(hum);
    float avgT = movingAverageFilter.process(temp);

   client->print("Temp: ");
   client->print(avgT);
   client->print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
   client->print("\t\tHumidity: ");
   client->print(avgN);
   client->print("% RH");
   client->print("\t\tPressure: ");
   client->print(avgP);
   client->println("Pa");

   b[0] = avgT;
   b[1] = avgN;
   b[2] = avgP;
  
  
   
}
