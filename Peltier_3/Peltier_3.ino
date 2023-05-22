//
// Projekt: PeltierRegler 
//          Temperaturmess-Sensor DS18B20 
//          MotorShield mit Peltierelement
//          Temperatur-Regler für Kühlung und Heizung
//
//          Version:  v0.3
//          Stand:    22.02.2023
//
//          (c) Kapeller Markus 
//

//Einbinden der Bibliotheken
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MotorDriver.h"
#include <PID_v1.h> 

#define ONE_WIRE_BUS 3  //Sensor DS18B20 am digitalen Pin 3

OneWire oneWire(ONE_WIRE_BUS);    //
MotorDriver motor;                //

//Übergabe der OnewWire Referenz zum kommunizieren mit dem Sensor.
DallasTemperature sensors(&oneWire);

int sensorCount;

float SollTemp;   // SollTemperatur
float IstTemp;    // IstTemperatur
float DiffTemp;   // Temperaturdifferenz
float OutFloat;   // Regleroutout für Displayanzeige

// Tuning parameters 
//float Kp=2.5; //Initial Proportional Gain 
//float Ki=25; //Initial Integral Gain 
//float Kd=0; //Initial Differential Gain 

// Tuning parameters "optimized by try and fault"
float Kp=60; //Initial Proportional Gain (120)
float Ki=800; //Initial Integral Gain (800)
float Kd=0; //Initial Differential Gain 

double Setpointc, Inputc, Outputc;  // Regler für Kühlung 
PID myPIDc(&Inputc, &Outputc, &Setpointc, Kp, Ki, Kd, DIRECT); 

double Setpointh, Inputh, Outputh;  // Regler für Heizung 
PID myPIDh(&Inputh, &Outputh, &Setpointh, Kp, Ki, Kd, DIRECT); 

const int sampleRate = 1; // Variable that determines how fast our PID loop runs 

// Communication setup 
const long serialPing = 500; //This determines how often we ping our loop (500)
// Serial pingback interval in milliseconds 

unsigned long now = 0; //This variable is used to keep track of time 

// placehodler for current timestamp 
unsigned long lastMessage = 0; //This keeps track of when our loop last spoke to serial 
// last message timestamp. 

// AnalogRead's
int poti_A2;              
float TSoll;              // Solltemperatur -2.0 bis 32.0 Grad
const int soll_min =  50;
const int soll_max = 400;

// kühlen / heizen
//int Faktor = 0;

// Definition der LCD-Größe, um sie für andere Displays zentral ändern zu können
const int LCDSpaltenzahl = 40;
const int LCDZeilenzahl = 2;

//Displayanschlüsse initialisieren
LiquidCrystal lcd(10, 2, 4, 5, 6, 7); // Creates an LC object. Parameters: (rs, enable, d4, d5, d6, d7)

void setup(void) { 

  // set up the LCD's number of columns and rows:
  lcd.begin (LCDSpaltenzahl, LCDZeilenzahl);
  lcd.noCursor();
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("Projekt: Temperatur DS18B20");
  lcd.setCursor(10, 1);
  lcd.print("(c) Kapeller Markus");  
  delay(2000);
  lcd.clear();
  motor.begin();

  Serial.begin(9600); //Starten der seriellen Kommunikation mit 9600 baud
  Serial.println("OutC:,OutH:,Diff:");         // Legend for Serialplot
  //Serial.println("Temperatursensor - DS18B20"); 
  //Serial.println("PID-Regler mit Peltierelemt"); 
  sensors.begin(); //Starten der Kommunikation mit dem Sensor
  sensorCount = sensors.getDS18Count(); //Lesen der Anzahl der angeschlossenen Temperatursensoren.

  myPIDc.SetMode(AUTOMATIC); //Turn on the PID loop 
  myPIDc.SetSampleTime(sampleRate); //Sets the sample rate 
  myPIDh.SetMode(AUTOMATIC); //Turn on the PID loop 
  myPIDh.SetSampleTime(sampleRate); //Sets the sample rate 
  lastMessage = millis(); // timestamp 
 
}

void loop(void){ 
 if(sensorCount ==0){
   //Serial.println("Es wurde kein Temperatursensor gefunden!");
   //Serial.println("Bitte überprüfe deine Schaltung!");
   lcd.setCursor(0, 0);
   lcd.print("Es wurde kein Temperatursensor gefunden!");
   lcd.setCursor(0, 1);
   lcd.print("Bitte ueberpruefe deine Schaltung!");
 }
 //Es können mehr als 1 Temperatursensor am Datenbus angschlossen werden.
 //Anfordern der Temperaturwerte aller angeschlossenen Temperatursensoren.
 sensors.requestTemperatures(); 

 //Ausgabe aller Werte der angeschlossenen Temperatursensoren.
 for(int i=0;i<sensorCount;i++){
  /*
  Serial.print(i); 
  Serial.println(". Temperatur :"); 
  printValue(sensors.getTempCByIndex(i), "°C");
  printValue(sensors.getTempFByIndex(i), "°F");
  */
   //printValue("", sensors.getTempCByIndex(i));
 
   // Soll- & Ist-Temperatur
   SollTemp = Soll_poti();
   IstTemp = sensors.getTempCByIndex(i);
   DiffTemp = IstTemp-SollTemp;
   
   /*
   // kühlen
   if (DiffTemp > 0) {
      Input = SollTemp;
      Setpoint = IstTemp;
      Faktor = -1;
   }
   // heizen
   if (DiffTemp < 0) {
      Input = IstTemp;
      Setpoint = SollTemp;
      Faktor = 1;
   }
   */
  
  // Soll- und Istwerte für die Regler
  Inputc = SollTemp;
  Setpointc = IstTemp;
  Inputh = IstTemp;
  Setpointh = SollTemp;

    
 }
    
   lcd.setCursor(0, 0);
   lcd.print("Soll ");
   lcd.print(SollTemp);
   lcd.print(" ");
   //lcd.print((char)223);
   //lcd.print("C ");
   lcd.setCursor(13, 0);
   lcd.print("Ist ");
   lcd.print(IstTemp);
   lcd.print(" ");
   //lcd.print((char)223);
   //lcd.print("C "); 
   lcd.setCursor(25, 0);
   lcd.print("Diff  ");  
     if ((DiffTemp >= 0)&&(DiffTemp < 10)) lcd.print(" ");
   lcd.print(DiffTemp);
   lcd.print(" ");
   lcd.print((char)223);
   lcd.print("C  ");

   
   myPIDc.Compute();                 // Run the PID loop cooling 
   myPIDh.Compute();                 // Run the PID loop heating 
   motor.speed(0, Outputh-Outputc);   
   //set motor0 speed to Output
   now = millis();                  // Keep track of time 

   lcd.setCursor(0, 1);
   lcd.print(" Oh:");
    if (Outputh < 100) lcd.print(" "); 
    if (Outputh < 10) lcd.print(" ");
   lcd.print(Outputh);
   lcd.print(" ");
   lcd.setCursor(13, 1);
   lcd.print("Oc:");
    if (Outputc < 100) lcd.print(" ");    
    if (Outputc < 10) lcd.print(" ");
   lcd.print(Outputc);
   lcd.print(" ");
   lcd.setCursor(25, 1);
   lcd.print("Out: ");
   /*
     if (((Outputh - Outputc) >= 0)&&((Outputh - Outputc) < 100)) lcd.print(" ");
     if (((Outputh - Outputc) >= 0)&&((Outputh - Outputc) < 10)) lcd.print(" ");
     if (((Outputh - Outputc) < 0)&&((Outputh - Outputc) > -10)) lcd.print(" ");   
   lcd.print(Outputh - Outputc);
   */
   OutFloat=(Outputh-Outputc)*100/255;
     if ((OutFloat >= 0)&&(OutFloat < 100)) lcd.print(" ");
     if ((OutFloat >= 0)&&(OutFloat < 10)) lcd.print(" ");
     if ((OutFloat < 0)&&(OutFloat > -10)) lcd.print(" ");      
   lcd.print(OutFloat,2);
   lcd.print("% ");

   lcd.setCursor(38, 1); 
     //if ((Outputh - Outputc) >= 0) lcd.print("+ ");
     //else if ((Outputh - Outputc) < 0)  lcd.print("- ");
     if ((Outputh  - Outputc) >= 0) lcd.print("H ");
     else if ((Outputh - Outputc) < 0)  lcd.print("C ");
     
   //printValue( "Outc ", Outputc);
   //printValue( "Outh ", Outputh);
  Serial.print(Outputc);
  Serial.print(",");
  Serial.print(Outputh);
  Serial.print(",");
  Serial.println(Outputh - Outputc);
   
} 

void printValue(String text, float value){
  Serial.print("\t\t");
  Serial.print(text);  
  Serial.println(value);
}

float Soll_poti() {
   poti_A2 = analogRead(A2);  
   TSoll = map(poti_A2, 0, 1023, soll_min, soll_max);  
   return TSoll/10;
}
