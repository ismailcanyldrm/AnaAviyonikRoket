
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU9250_asukiaaa.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <MovingAverageFilter.h>
#include "LoRa_E22.h"

#define SEALEVELPRESSURE_HPA (906.56)
float aX, aY, aZ, gX, gY, gZ, mX, mY, mZ,ts,ta;
float iX, iY, iZ, ggX, ggY, ggZ, mmX, mmY, mmZ;
static void smartdelay(unsigned long ms);
float baseline,hp1,hp2,t ,e ,b,be,bes; 
int t1 = 14;
int t2 = 3;
int tix , th, thf  = 0;
float fark ,fark2= 0.0;

Adafruit_BME280 bme;
MPU9250_asukiaaa mySensor;
TinyGPS gps;
SoftwareSerial ss(13,12);
SoftwareSerial lora(1,0);
//File myFile;
LoRa_E22 E22(&lora);


MovingAverageFilter mvg00(10);
MovingAverageFilter mvg01(10);
MovingAverageFilter mvg02(10);
MovingAverageFilter mvg03(10);

MovingAverageFilter mvg10(10);
MovingAverageFilter mvg11(10);
MovingAverageFilter mvg12(10);

MovingAverageFilter mvg20(10);
MovingAverageFilter mvg21(10);
MovingAverageFilter mvg22(10);

MovingAverageFilter mvg30(10);
MovingAverageFilter mvg31(10);
MovingAverageFilter mvg32(10);


struct Signal {
    byte Xgyro[4];
    byte Ygyro[4];
    byte Zgyro[4];
    byte Xivme[4];
    byte Yivme[4];
    byte Zivme[4];
    byte Xmage[4];
    byte Ymage[4];
    byte Zmage[4];
    byte h[4];
    byte p[4];
    byte gpsE[4];
    byte gpsB[4];
    byte gpsH[4];
    byte gpsS[4];

} data;

void setup() {
  pinMode(t1,OUTPUT);
  pinMode(t2, OUTPUT);
  Wire.begin();
  Serial.begin(9600);
  bme.begin(0x76);
  ss.begin(9600);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  E22.begin();


  
  
  // You can set your own offset for mag values
  mySensor.magXOffset = +18;
  mySensor.magYOffset = -29;
  mySensor.magZOffset = -18;
}

void loop() 
{
  mpu9250();
  bmp280();
  gpsSpeed();
  gpsAltitude();
  gpsKonum();
  smartdelay(250);

    Yfark();
    Xfark();
    if (hp2 >= 1600 && fark >1.0 && iX<=-7 && fark2>1){
      digitalWrite(t1, HIGH);
      thf=1 ;
      bmp280();
      ts = hp2;
    }
    if(hp2 <= 610 && thf == 1 && fark > 1.0){
      digitalWrite(t2, HIGH); 
      thf =0;
      bmp280();
      ta = hp2;
    }
   
  *(float*)(data.Xgyro) = ggX ;
  *(float*)(data.Ygyro) = ggY ;
  *(float*)(data.Zgyro) = ggZ ;

  *(float*)(data.Xivme) = iX ;
  *(float*)(data.Yivme) = iY ;
  *(float*)(data.Zivme) = iZ ;

  *(float*)(data.Xmage) = mmX ;
  *(float*)(data.Ymage) = mmY ;
  *(float*)(data.Zmage) = mmZ ;
  
  *(float*)(data.h) = hp2 ;
  *(float*)(data.p) = hp1 ;
  
  *(float*)(data.gpsE) = e ;
  *(float*)(data.gpsB) = b ;
  *(float*)(data.gpsH) = be ;
  *(float*)(data.gpsS) = bes ;


  ResponseStatus rs = E22.sendFixedMessage(0, 62, 22, &data, sizeof(Signal));
}
void Yfark(){
  
  bmp280();
  fark = hp2;
  delay(250);
  bmp280();
  fark = fark - hp2;
  
}
void Xfark(){
  
  bmp280();
  fark2 = hp2;
  delay(250);
  bmp280();
  fark2= fark2 - hp2;
  
}

void bmp280()
{
    float hp[2]= {0.0,0.0};
    float T  = bme.readTemperature();
    float P = bme.readPressure() / 100.0F;
    float A = bme.readAltitude(SEALEVELPRESSURE_HPA);
    float H = bme.readHumidity();
  
    float avgT = mvg00.process(T);
    float avgP = mvg01.process(P);
    float avgA = mvg02.process(A);
    float avgH = mvg03.process(H);

      hp[0]=avgP;
      hp[1]=avgA;
      hp1=hp[0];
      hp2=hp[1];
}

void mpu9250()
{
    float ivme[3]= {0.0,0.0,0.0};
    float result;
  result = mySensor.accelUpdate();
    float aX = mySensor.accelX();
    float aY = mySensor.accelY();
    float aZ = mySensor.accelZ();

    float avgaX = mvg10.process(aX);
    float avgaY = mvg11.process(aY);
    float avgaZ = mvg12.process(aZ);

      ivme[0]=avgaX*9.81;
      ivme[1]=avgaY*9.81;
      ivme[2]=avgaZ*9.81;

      iX=ivme[0];
      iY=ivme[1];
      iZ=ivme[2];

      
      
    float gyro[3]= {0.0,0.0,0.0};
  result = mySensor.gyroUpdate();
    float gX = mySensor.gyroX();
    float gY = mySensor.gyroY();
    float gZ = mySensor.gyroZ();

    float avggX = mvg20.process(gX);
    float avggY = mvg21.process(gY);
    float avggZ = mvg22.process(gZ);
      gyro[0]=avggX;
      gyro[1]=avggY;
      gyro[2]=avggZ;
      
      ggX=gyro[0];
      ggY=gyro[1];
      ggZ=gyro[2];

    float mage[3]= {0.0,0.0,0.0};
  result = mySensor.magUpdate();
    float mX = mySensor.magX();
    float mY = mySensor.magY();
    float mZ = mySensor.magZ();

    float avgmX = mvg30.process(mX);
    float avgmY = mvg31.process(mY);
    float avgmZ = mvg32.process(mZ);
      mage[0]=avgmX;
      mage[1]=avgmY;
      mage[2]=avgmZ;
      
      mmX=mage[0];
      mmY=mage[1];
      mmZ=mage[2];
}

void gpsKonum() {
  float gpsk[2]= {0.000000,0.000000};
  float flat, flon, invalid;
  gps.f_get_position(&flat, &flon);
  invalid = TinyGPS::GPS_INVALID_F_ANGLE;
  
  if (flat == invalid && flon == invalid) {
    gpsk[0]= 0.000000;
    gpsk[1]= 0.000000;
    e = gpsk[0];
    b = gpsk[1];
  } else {
    gpsk[0]= float(flat),6;
    gpsk[1]= float(flon),6;
    e = gpsk[0];
    b = gpsk[1]; 
  }
}
void gpsAltitude(){
  float gpsa[1]= {0.00};
  float gpsAltitude , invalid;
  gpsAltitude = gps.f_altitude() ;
  invalid   = TinyGPS::GPS_INVALID_F_ALTITUDE;
  if (gpsAltitude == invalid) {
    gpsa[0]= 0.00;
    be = gpsa[0];
  } else {
    gpsa[0]= float(gpsAltitude);
    be = gpsa[0];
    }
}
void gpsSpeed(){
  float gpss[1]= {0.00};
  float gpsSpeed , invalid;
  gpsSpeed = gps.f_speed_kmph() ;
  invalid   = TinyGPS::GPS_INVALID_F_SPEED;
  if (gpsSpeed == invalid) {
    gpss[0]= 0.00;
    bes = gpss[0];
  } else {
    gpss[0]= float(gpsSpeed);
    bes = gpss[0];
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
    digitalWrite(9, HIGH);
    delay(sure);
    digitalWrite(9, LOW);
    delay(sure);
  }
}

void led(int tekrar, int sure) {
  for (int i = 0; i < tekrar; i++) {
    digitalWrite(25, HIGH);
    delay(sure);
    digitalWrite(25, LOW);
    delay(sure);
  }
}
