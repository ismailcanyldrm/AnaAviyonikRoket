#include <BME280.h>
#include <BME280I2C.h>
#include <BME280I2C_BRZO.h>
#include <BME280Spi.h>
#include <BME280SpiSw.h>
#include <EnvironmentCalculations.h>

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
  Serial.begin(9600);
  while(!Serial);
  Serial.println("started");
      serialgps.begin(9600);

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
  Serial.println("MPU9250 VERİLERİ");
   mpu_();
   Serial.println("BME280 VERİLERİ");
   printBME280Data(&Serial);
   Serial.println("GPS VERİLERİ");
   GPSS();


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
void mpu_()
{
  uint8_t sensorId;
  int result;

  result = mySensor.readId(&sensorId);
  if (result == 0) {
    Serial.println("sensorId: " + String(sensorId));
  } else {
    Serial.println("Cannot read sensorId " + String(result));
  }

  result = mySensor.accelUpdate();
  if (result == 0) {

    float avgx = movingAverageFilter.process(mySensor.accelX());
    float avgy = movingAverageFilter.process(mySensor.accelY());
    float avgz = movingAverageFilter.process(mySensor.accelZ());
            Serial.print("Avg X İvmesi : ");
    Serial.print(avgx);Serial.println("(m/s²)"); 
        Serial.print("Avg Y İvmesi : ");
    Serial.print(avgy);Serial.println("(m/s²)");    
        Serial.print("Avg Z İvmesi : ");
    Serial.print(avgz);Serial.println("(m/s²)");  

    a[0] = avgx;
    a[1] = avgy;
    a[2] = avgz;

  } else {
    Serial.println("Cannod read accel values " + String(result));
  }

  result = mySensor.gyroUpdate();
  if (result == 0) {
          
   
    float avgHx = movingAverageFilter.process(mySensor.gyroX());
    float avgHy = movingAverageFilter.process(mySensor.gyroY());
    float avgHz = movingAverageFilter.process(mySensor.gyroX());
    
            Serial.print("Avg X Açısal Hızı : ");
    Serial.print(avgHx,2);Serial.println("(rad/sn)"); 
        Serial.print("Avg Y Açısal Hızı : ");
    Serial.print(avgHy,2);Serial.println("(rad/sn)");   
        Serial.print("Avg Z Açısal Hızı : ");
    Serial.print(avgHz,2);Serial.println("(rad/sn)"); 

    g[0] = avgHx;
    g[1] = avgHy;
    g[2] = avgHz;
    
  } else {
    Serial.println("Cannot read gyro values " + String(result));
  }

  Serial.println("at " + String(millis()) + "ms");
  Serial.println(""); // Add an empty line
    
}
void GPSS()
{
   while(serialgps.available())
    {
        int c = serialgps.read();
        if(gps.encode(c))
        {
            float latitude, longitude;
            gps.f_get_position(&latitude, &longitude);
            Serial.print("Enlem/Boylam: ");
            Serial.print(latitude,5);
            Serial.print(" / ");
            Serial.println(longitude,5);
            gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
            Serial.print("Gün: ");
            Serial.print(day, DEC);
            Serial.print("Ay: ");
            Serial.print(month, DEC);
            Serial.print("Yıl: ");
            Serial.print(year);
            Serial.print(" Saat: ");
            Serial.print((hour+3), DEC);
            Serial.print(":");
            Serial.print(minute, DEC);
            Serial.print(":");
            Serial.print(second, DEC);
            Serial.print(".");
            Serial.println(hundredths, DEC);
            Serial.print("Yükseklik (meters): ");
            Serial.println((gps.f_altitude()/1229.0000));
            Serial.print("Rota (degrees): ");
            Serial.println(gps.f_course());
            Serial.print("Hız(kmph): ");
            Serial.println(gps.f_speed_kmph());
            Serial.print("Uydu Sayısı: ");
            Serial.println(gps.satellites());
            Serial.println();
            
            // adding cardinal course
            Serial.print("Kardinal Pusula Yönü  ");
            Serial.println(gps.cardinal(gps.f_course()));
            // give it a try
            gpsint[0] = day; 
            gpsint[1] = month;
            gpsint[2] = year;
            gpsdouble[0] = hour+3;
            gpsdouble[1] = minute;
            gpsdouble[2] = second;
            gpsdouble[3] = hundredths ;
            gpsfloat[0] = (gps.f_altitude()/1229.0000);
            gpsfloat[1] = gps.f_course();
            gpsfloat[2] = gps.f_speed_kmph();
            gpsfloat[3] = latitude;
            gpsfloat[4] = longitude;
            
            gps.stats(&chars, &sentences, &failed_checksum);
        }
    }
}
