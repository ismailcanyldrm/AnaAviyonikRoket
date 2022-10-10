#include <BME280.h>
#include <BME280I2C.h>
#include <BME280I2C_BRZO.h>
#include <BME280Spi.h>
#include <BME280SpiSw.h>
#include <EnvironmentCalculations.h>

#include <MPU9250_asukiaaa.h>
#include <movingAvg.h> 
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Adafruit_Sensor.h>



#define SEALEVELPRESSURE_HPA (1013.25)
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

MPU9250 mpu9250;
SoftwareSerial mpu9250(6,7);

File sdkart;

MPU9250 IMU(Wire,0x68);
int status;
int RXPin = 10;
int TXPin = 11;

TinyGPS gps;
SoftwareSerial serialgps(10,11);
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long chars;
unsigned short sentences, failed_checksum;
int GPSBaud = 9600;
  

movingAvg avgmpu9250x(1);  
movingAvg avgmpu9250z(2); 
movingAvg avgmpu9250y(3);

movingAvg avgmpu9250gyrox(1);  
movingAvg avgmpu9250gyroz(2); 
movingAvg avgmpu9250gyroy(3);

movingAvg avgmpu9250acix(1);  
movingAvg avgmpu9250aciz(2); 
movingAvg avgmpu9250aciy(3);

movingAvg avgbmp180P(1);  
movingAvg avgbmp180Y(2); 

void setup() {

   Wire_00.begin();
   Wire_00.setClock(100000);
  Serial.begin(115200);
  while(!Serial) {}

  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
 
     while ( !Serial ) delay(100);   
  Serial.println(F("BMP280 test"));
  unsigned status;

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
        avgmpu9250x.begin();  
    avgmpu9250y.begin(); 
    avgmpu9250z.begin(); 
        avgmpu9250gyrox.begin();  
    avgmpu9250gyroy.begin(); 
    avgmpu9250gyroz.begin();  
        avgmpu9250acix.begin();  
    avgmpu9250aciy.begin(); 
    avgmpu9250aciz.begin();       
    serialgps.begin(9600);
    Serial.println("");
    Serial.println(" ...Uydu Bağlantısı Bekleniyor... ");
    Serial.println("");  
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

void loop() 
{
  Serial.println("************AÇI VERİLERİ************");
  aci();
  Serial.println("************İVME VERİLERİ************");
  ivme();
  Serial.println("************AÇISAL HIZ VERİLERİ************");
  acisalhiz();
  Serial.println("************SICAKLIK VERİSİ************");
  Serial.print("Sıcaklık :");
  Serial.print(IMU.getTemperature_C(),2);Serial.println("°C");
  Serial.println("************BASINÇ VERİSİ************");
  Basinc();
    Serial.println("************YÜKSEKLİK VERİSİ************");
  Yukseklik();
  Serial.println("************GPS VERİLERİ************");
  GPSS();
 
  delay(250);
} 
void sd_card()
{
  int a = 1;
  int b = 2;
  Serial.print("Basınç değeri : ");Serial.println(b);
 Serial.print(" , ");
 Serial.print("Yükseklik değeri : ");Serial.println(a);

 sdkart = SD.open("dataAaAa.txt",FILE_WRITE);

 if(sdkart)
 {
  sdkart.print("Basınç değeri: ");sdkart.print(double(b));
  sdkart.print(" , ");
  sdkart.print("Yükseklik değeri: ");sdkart.println(double(a));
  sdkart.close();  
 }
 else
 {
  Serial.println("dataAaAa.txt açılamadı.");
 }
}
void ivme()
{
  IMU.readSensor();
      float ax[3]={0.0,0.0,0.0};
           ax[0] = IMU.getAccelX_mss();    
    float avgx = avgmpu9250x.reading(ax[0]); 
          ax[1] = IMU.getAccelY_mss();    
    float avgy = avgmpu9250y.reading(ax[1]); 
          ax[2] = IMU.getAccelZ_mss();    
    float avgz = avgmpu9250z.reading(ax[2]); 
      
            Serial.print("Avg X İvmesi : ");
    Serial.print(avgx,6);Serial.println("(m/s²)"); 
        Serial.print("Avg Y İvmesi : ");
    Serial.print(avgy,6);Serial.println("(m/s²)");    
        Serial.print("Avg Z İvmesi : ");
    Serial.print(avgz,6);Serial.println("(m/s²)");  
}
void acisalhiz()
{
    IMU.readSensor();
      float aH[3]={0.0,0.0,0.0};
           aH[0] = IMU.getGyroX_rads();
    float avgHx = avgmpu9250gyrox.reading(aH[0]); 
          aH[1] = IMU.getGyroY_rads();    
    float avgHy = avgmpu9250gyroy.reading(aH[1]); 
          aH[2] = IMU.getGyroZ_rads();    
    float avgHz = avgmpu9250gyroz.reading(aH[2]); 
      
            Serial.print("Avg X Açısal Hızı : ");
    Serial.print(avgHx,2);Serial.println("(rad/sn)"); 
        Serial.print("Avg Y Açısal Hızı : ");
    Serial.print(avgHy,2);Serial.println("(rad/sn)");   
        Serial.print("Avg Z Açısal Hızı : ");
    Serial.print(avgHz,2);Serial.println("(rad/sn)"); 
}
void aci()
{
  IMU.readSensor();
      float aA[3]={0.0,0.0,0.0};
      float Anglex = degrees (IMU.getGyroX_rads());
           aA[0] = Anglex;
    float avgAx = avgmpu9250acix.reading(aA[0]); 
     float Angley = degrees (IMU.getGyroY_rads());
          aA[1] = Angley;    
    float avgAy = avgmpu9250aciy.reading(aA[1]); 
     float Anglez = degrees (IMU.getGyroZ_rads());
          aA[2] = Anglez;    
    float avgAz = avgmpu9250aciz.reading(aA[2]); 
      
            Serial.print("Avg X Açısı : ");
    Serial.print(avgAx,2);Serial.println("°"); 
        Serial.print("Avg Y Açısı : ");
    Serial.print(avgAy,2);Serial.println("°"); 
        Serial.print("Avg Z Açısı : ");
    Serial.print(avgAz,2);Serial.println("°");
}


   void  Basinc ()
   {
            float aP[]={0.0};
           aP[0] = (bme.readPressure());
    float avgPx =avgbmp180P.reading(aP[0]); 
           Serial.print("Avg Basınç : ");
    Serial.print(avgPx,2);Serial.println(" Pa ");
 }

    void Yukseklik ()
    {
              float aY[]={0.0};
           aY[0] = (bme.readAltitude(SEALEVELPRESSURE_HPA));
    float avgYx =avgbmp180Y.reading(aY[0]); 
           Serial.print("Avg Yükseklik : ");
    Serial.print(avgYx,2);Serial.println(" m ");

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

            gps.stats(&chars, &sentences, &failed_checksum);
        }
    }
}
