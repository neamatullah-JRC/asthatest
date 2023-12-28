#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 4///GPPIO 4= D2 PIN

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
SoftwareSerial gpsSerial(D1);  //RX,TX new design ok
TinyGPSPlus gps;

int x;
float xx;
int y;
float yy;
int hh;
int mm;
int ss;
int dd;
int mn;
int yr;
float speeed;
float directione;
int Min;
int Max;
float Sum;
float lat;
float alat;
int Ming;
int Maxg;
float Sumg;
float lng;
float alng;
float R1 = 100300;  // Resistor R1 100k
float R2 = 10140;   // Resistor R2 10k
int MinA;
int MaxA;
float SumA;
float ADC;
float aADC;
float temperature;


String password="123456";

void setup() {
  delay(10000);
  Serial.begin(115200);
  gpsSerial.begin(9600);  // gps serial
  DS18B20.begin();
  pinMode(D7, OUTPUT);  //RELAY PIN
  pinMode(D5, OUTPUT);  //FIND VEHICLE OUTPUT
  pinMode(D0, INPUT);   //ACC DETECTION
  pinMode(D3, INPUT);   //DIGITAL INPUT A
  pinMode(D6, INPUT);   //DIGITAL INPUT B
  pinMode(A0, INPUT);   //ADC CHANNEL
  Serial.println("AT+CMGF=1\r");
  delay(1000);
  Serial.println("AT+CNMI=2,2,0,0,0\r");
  delay(1000);
}
void loop() {
  

  GPSDelay(1000);
  MinA = 1000;  //Initilize/reset to limit
  MaxA = 0;     //Initilize/reset to limit
  SumA = 0;     //Initialize/reset
  for (int i = 0; i < 1000; i++) {
    ADC = ((((analogRead(A0)) * 3.3) / 1023.0) / (R2 / (R1 + R2))) - 0.3800;  //ERROR VALUE=0.3810
    SumA = SumA + ADC;
    if (ADC < MinA)
      MinA = ADC;
    if (ADC > MaxA)
      MaxA = ADC;
  }
  aADC = (SumA / 1000);
  dd = gps.date.day();
  mn = gps.date.month();
  yr = gps.date.year();
  hh = gps.time.hour();
  mm = gps.time.minute();
  ss = gps.time.second();
  Min = 25;  //Initilize/reset to limit
  Max = 0;   //Initilize/reset to limit
  Sum = 0;   //Initialize/reset
  //Take 1000 readings, find min, max, and average.  This loop takes about 100ms.
  for (int i = 0; i < 25; i++) {
    lat = gps.location.lat();
    Sum = Sum + lat;  //Sum for averaging
    if (lat < Min)
      Min = lat;
    if (lat > Max)
      Max = lat;
  }
  alat = (Sum / 25);
  Ming = 25;  //Initilize/reset to limit
  Maxg = 0;   //Initilize/reset to limit
  Sumg = 0;   //Initialize/reset
  //Take 1000 readings, find min, max, and average.  This loop takes about 100ms.
  for (int i = 0; i < 25; i++) {
    lng = gps.location.lng();
    Sumg = Sumg + lng;  //Sum for averaging
    if (lng < Min)
      Min = lng;
    if (lng > Max)
      Max = lng;
  }
  alng = (Sumg / 25);
  x = xx = alat;
  y = yy = alng;
  speeed = gps.speed.kmph() / 2;
  directione = gps.course.deg();
  float gps_lat = (xx - x) * 60;
  float gps_long = (yy - y) * 60;
  DS18B20.requestTemperatures();
  temperature = DS18B20.getTempCByIndex(0);

  
  if (Serial.available() > 0) {
    String textMessage = Serial.readString();
    Serial.println(textMessage);
   
 if ((textMessage.indexOf("PARAM") >= 0)||(textMessage.indexOf("Param") >= 0)||(textMessage.indexOf("param") >= 0)) {
      String RN = textMessage.substring(textMessage.indexOf("+CMT:") + 10, (textMessage.indexOf("+CMT:") + 21));
      Serial.println("AT");
      delay(100);
      Serial.println("AT+CMGF=1");
      delay(100);
      Serial.print("AT+CMGS=\"");
      Serial.print(RN);
      Serial.println("\"");
      delay(100);
      Serial.println("server,apn,timer,numbers............");
      Serial.write(char(26));
      delay(5000);
     }
     if ((textMessage.indexOf("STATUS") >= 0)||(textMessage.indexOf("Status") >= 0)||(textMessage.indexOf("status") >= 0)) {
      String RN = textMessage.substring(textMessage.indexOf("+CMT:") + 10, (textMessage.indexOf("+CMT:") + 21));
      Serial.println("AT");
      delay(100);
      Serial.println("AT+CMGF=1");
      delay(100);
      Serial.print("AT+CMGS=\"");
      Serial.print(RN);
      Serial.println("\"");
      delay(100);
      Serial.println("gprs link, gps position, url, etc...........");
      Serial.write(char(26));
      delay(5000);
     }
 if ((textMessage.indexOf("ENON") >= 0 )||(textMessage.indexOf("Enon") >= 0 )||(textMessage.indexOf("enon") >= 0 )) {
      String RN = textMessage.substring(textMessage.indexOf("+CMT:") + 10, (textMessage.indexOf("+CMT:") + 21));
      digitalWrite(D7, LOW);  // RELAY LOW
      Serial.println("AT");
      delay(100);
      Serial.println("AT+CMGF=1");
      delay(100);
      Serial.print("AT+CMGS=\"");
      Serial.print(RN);
      Serial.println("\"");
      delay(100);
      Serial.println("IGNITION LINE CONNECTED.");
      Serial.write(char(26));
      delay(5000);
    }
 if ((textMessage.indexOf("ENOFF") >= 0 )||(textMessage.indexOf("Enoff") >= 0 )||(textMessage.indexOf("enoff") >= 0 )) {
      String RN = textMessage.substring(textMessage.indexOf("+CMT:") + 10, (textMessage.indexOf("+CMT:") + 21));
      digitalWrite(D7, HIGH);  // RELAY LOW
      Serial.println("AT");
      delay(100);
      Serial.println("AT+CMGF=1");
      delay(100);
      Serial.print("AT+CMGS=\"");
      Serial.print(RN);
      Serial.println("\"");
      delay(100);
      Serial.println("IGNITION LINE DISCONNECTED.");
      Serial.write(char(26));
      delay(5000);
    }
    

 if ((textMessage.indexOf("TCP,") >= 0)||(textMessage.indexOf("Tcp,") >= 0)||(textMessage.indexOf("tcp,") >= 0)) {
    String RN = textMessage.substring(textMessage.indexOf("+CMT:") + 10, (textMessage.indexOf("+CMT:") + 21));
    String ServerAddress = textMessage.substring(textMessage.indexOf("SERVER,") + 7, textMessage.indexOf("*"));  // Extract the IP address

      Serial.println("AT");
      delay(100);
      Serial.println("AT+CMGF=1");
      delay(100);
      Serial.print("AT+CMGS=\"");
      Serial.print(RN);
      Serial.println("\"");
      delay(100);
      Serial.println("TCP: "+ServerAddress);
      Serial.write(char(26));
      delay(5000);
     }
















  }
  Serial.println("AT");  //ok
  delay(10);
  Serial.println("AT+IPR=115200");  //ok
  delay(10);
  Serial.println("AT&W");  //ok
  delay(10);
  Serial.println("ATO");  //ok
  delay(10);
  Serial.println("ATA");  //ok
  delay(10);
  Serial.println("AT+CGATT=1");  //ok
  delay(10);
  Serial.println("AT+CIPMUX=0");  //ok
  delay(10);
  Serial.println("AT+CSTT=\"gpinternet\",\"\",\"\"");  //           APN    SETUP
  delay(10);
  Serial.println("AT+CIICR");
  delay(10);
  Serial.println("AT+CIFSR");
  delay(10);
  Serial.println("AT+CIPSTART=\"TCP\",\"75.119.146.146\",\"6016\"");//75.119.146.146
  delay(100);
  Serial.println("AT+CIPSEND");
  delay(100);
  Serial.print("#");
  Serial.print("LITTLESPARK0002");  //                                [DEVICE ID]
  Serial.print("##");
  if ((gps.location.isValid()) && (digitalRead(D0) == LOW)) {
    Serial.print(F("1"));
  } else {
    Serial.print(F("0"));
  }  ///VALID,0,1#
  Serial.print("#");
  if (temperature < -110) {
    Serial.print("TNC");
  }
  if (temperature > -110) {
    Serial.print(temperature);  // PASSWORD
    Serial.print("°C");
  }
  Serial.print("#");
  Serial.print("A-");
  if (digitalRead(D3) == LOW) {
    Serial.print("ON");
  }
  if (digitalRead(D3) == HIGH) {
    Serial.print("OFF");
  }
  Serial.print(",B-");
  if (digitalRead(D6) == LOW) {
    Serial.print("ON");
  }
  if (digitalRead(D6) == HIGH) {
    Serial.print("OFF");
  }  //       TWO DIGITAL INPUT                [EVENT](FREEZE STATUS)
  Serial.print("#");
  if (digitalRead(D0) == HIGH) { Serial.print("0"); }
  if (digitalRead(D0) == LOW) { Serial.print("1"); }  // OUTPUT STATUS, 1                    [PACKET](acc)
  Serial.print("#");
  if (aADC <= 0) { Serial.print("0.00"); }
  if (aADC > 0) { Serial.print(aADC, 2); }  // LBS  ADC VALUE
  Serial.print("#");
  if (y < 100) Serial.print(F("0"));
  Serial.print(y);
  if (gps_long < 10) Serial.print(F("0"));
  Serial.print(gps_long, 5);
  Serial.print(",E,");
  Serial.print(x);
  if (gps_lat < 10) Serial.print(F("0"));
  Serial.print(gps_lat, 5);
  Serial.print(",N,");
  if (digitalRead(D0) == HIGH) { Serial.print("0.00"); }
  if (digitalRead(D0) == LOW) { Serial.print(speeed); }  //                 [SPEED]
  Serial.print(",");
  Serial.print(directione);  //direction                                    [DIRECTION]
  Serial.print("#");
  if (dd < 10) Serial.print(F("0"));
  Serial.print(dd);
  if (mn < 10) Serial.print(F("0"));
  Serial.print(mn);
  if (yr < 10) Serial.print(F("0"));
  Serial.print(yr - 2000);
  Serial.print("#");
  if (hh < 10) Serial.print(F("0"));
  Serial.print(hh);
  if (mm < 10) Serial.print(F("0"));
  Serial.print(mm);
  if (ss < 10) Serial.print(F("0"));
  Serial.print(ss);
  Serial.println("##");
  delay(10);
  Serial.write(char(26));
  delay(5000);
}
static void GPSDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
    gps.location.isUpdated();
  } while (millis() - start < ms);
}

