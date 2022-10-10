#include <MPU9250_asukiaaa.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>


MPU9250_asukiaaa mySensor;
BME280I2C bme;
TinyGPS gps;
SoftwareSerial ss(13,12);
SoftwareSerial lora(1,0);

float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
String gpsKonumStr, gpsSatelliteStr , gpsAltitudeStr ;
String Data0,Data1,Data2,Data3,Data4,Data5,Data6 = "";

unsigned long chars;
unsigned short sentences, failed_checksum;
float bmp[3]={0.0,0.0,0.0};

void setup() {
  Serial.begin(115200);
  ss.begin(115200);
  lora.begin(9600);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  Wire.begin();
  bme.begin();
  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
}

void loop() {
  
   mpu();
   printBME280Data(&Serial);
   dataGonder();
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

   client->print("Temp: ");
   client->print(temp);
   client->print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
   client->print("\t\tHumidity: ");
   client->print(hum);
   client->print("% RH");
   client->print("\t\tPressure: ");
   client->print(pres);
   client->println("Pa");


   delay(1000);
} 

void mpu() {  
  uint8_t sensorId;
  int result;

  result = mySensor.accelUpdate();
  if (result == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
    Serial.println("accelX: " + String(aX));
    Serial.println("accelY: " + String(aY));
    Serial.println("accelZ: " + String(aZ));
    Serial.println("accelSqrt: " + String(aSqrt));
  } else {
    Serial.println("Cannod read accel values " + String(result));
  }

  result = mySensor.gyroUpdate();
  if (result == 0) {
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
    Serial.println("gyroX: " + String(gX));
    Serial.println("gyroY: " + String(gY));
    Serial.println("gyroZ: " + String(gZ));
  } else {
    Serial.println("Cannot read gyro values " + String(result));
  }

  result = mySensor.magUpdate();
  if (result == 0) {
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    mDirection = mySensor.magHorizDirection();
    Serial.println("magX: " + String(mX));
    Serial.println("maxY: " + String(mY));
    Serial.println("magZ: " + String(mZ));
    Serial.println("yatay yön: " + String(mDirection));
  } else {
    Serial.println("Cannot read mag values " + String(result));
  }
}

String gpsKonum() {
  float flat, flon, invalid;
  gps.f_get_position(&flat, &flon);
  invalid = TinyGPS::GPS_INVALID_F_ANGLE;
  if (flat == invalid || flon == invalid) {
    return ("NULL");
  } else {
    String latitude = String(flat, 6);
    String longitude = String(flon, 6);
    return (latitude + " " + longitude);
  }
}
String gpsSatellite() {
  float satellite, invalid;
  satellite = gps.satellites();
  invalid   = TinyGPS::GPS_INVALID_SATELLITES;
  if (satellite == invalid) {
    return ("NULL");
  } else {
    return (String(satellite)); 
  }
}

String gpsAltitude(){
  float gpsAltitude , invalid;
  gpsAltitude = gps.altitude() ;
  invalid   = TinyGPS::GPS_INVALID_ALTITUDE;
  if (gpsAltitude == invalid) {
    return ("NULL");
  } else {
    return (String(gpsAltitude)); 
  }
}

static void smartdelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
      
  } while (millis() - start < ms);
}

void dataGonder() {


  gpsKonumStr     = gpsKonum();
  gpsSatelliteStr = gpsSatellite();
  gpsAltitudeStr = gpsAltitude();
  
  Data0 = "|01-Ana";
  Data1 = "|GPS" ;
  Data1 += "|" + gpsSatelliteStr ;
  Data1 += "|" + gpsKonumStr + "|" ;
  Data1 += "|" + gpsAltitudeStr + "|" ;
  Data2 = "| Basinç : 876.00 |" ;
  Data3 = "| Yükseklik : 0.4 m |"  ;
  Data4 = "| Gyro X : " + String(gX) +"| Gyro Y : " + String(gY) +"|  Gyro Z : " + String(gZ) ;
  Data5 = "| İvmeX : " + String(mX) +"| İvmeY : " + String(mY) +"| İvmeZ : " + String(mZ);
  Data6 = "| aciX : " + String(aX) +"| aciY : "+ String(aY) +"| aciZ : " + String(aZ); 

  Serial.println(Data0);
  Serial.println(Data1);
  Serial.println(Data2);
  Serial.println(Data3);
  Serial.println(Data4);
  Serial.println(Data5);
  Serial.println(Data6);

  lora.println(Data0);
  lora.println(Data1);
  lora.println(Data2);
  lora.println(Data3);
  lora.println(Data4);
  lora.println(Data5);
  lora.println(Data6);
}
