#include <EEPROM.h> 
#include <LiquidCrystal_I2C.h>  
#include <RTClib.h> 
#include <Wire.h>   
#include <TimeLib.h>
#include <DS1307RTC.h>
#include "EmonLib.h"             // Include Emon Library
EnergyMonitor emon1;             // Create an instance
#define SERIAL_OPTION 1
// simulated encoder
int CLK = 2;        
int DT = 3;         
int SW = 4;         
const int Sw1 = 5;    
const int Sw2 = 6;    
const int Sw3 = 7;    
const int Rs1 = A6;    
const int Rs2 = A7;          
// the number of the output pin
const int R1 =  8;      
const int R2 =  9; 
const int R3 =  10; 
const int R4 =  11; 
const int R5 =  12; 
const int R6 =  13; 
// Buzzer
const byte buzzerPIN = A0;  
bool buzzerFlag = false;
bool buzzerFlag1 = false;
bool buzzerFlag2 = false;
bool buzzerFlag3 = false;
bool buzzerFlag4 = false;
bool buzzerFlag5 = false;
bool buzzerFlag6 = false;
// variables will change: for reading the pushbutton status
int Sw1State = 0;         
int Sw2State = 0;         
int Sw3State = 0;  
int Rs1State = 0;   
int Rs2State = 0;                  
int state = 0, Loadstate=0;
int state1 = 0, Loadstate1=0;
int state2 = 0, Loadstate2=0;
int state3 = 0, Loadstate3=0;
// Main Current reading
int MCread = A1;  
double kilos = 0;
int peakPower = 0;
int RMSPower = 0;
double RMSCurrent = 0;
// Main Voltage reading
const int MVread = A2; 
// Test Current reading
const int TCread = A3;
int Tsens = 185; 
int Tadc= 00;
int Toffset = 2500; 
double Tvolt = 00;
double Tcurr = 00;
double Twatt = 00;

int OCP,OVP,UVP,TOCP;
int mcur,mocp,movp,muvp,mtocp,mk,mpp;

LiquidCrystal_I2C lcd(0x27,16,2); 
RTC_DS1307 rtc;

static int oldCLK = LOW;  
static int oldDT = LOW;  
int currb=HIGH,lastb=HIGH;

char buf[20];

int mode=0; // 0=normal, 1=setup
int menu=0; // 0=METER, 1=TEST, 2=OCP,3=OVP,4=UVP,5=TOCP 6=END

long mkwh = 999999;                                       // x will automatically hold 0F423F
byte mkwh1 = (byte) mkwh & 0x0000FF;                   //x1 will hold 3F
byte mkwh2 = (byte) (mkwh>>8) & 0x0000FF;          //x2 will hold 42
byte mkwh3 = (byte) (mkwh>>16) & 0x0000FF; 

void setup()
{
  initsetting(); 
  loadsetting();
  lcd.init(); 
  lcd.backlight(); 
  rtc.begin();
  if (SERIAL_OPTION) Serial.begin(9600);
  emon1.voltage(MVread, 234.26, 1.7);  // Voltage: input pin, calibration, phase_shift
  emon1.current(MCread, 11.1);       // Current: input pin, calibration.
  emon1.calcVI(20,2000);         // Calculate all. No.of half wavelengths (crossings), time-out
   float realPower       = emon1.realPower;        //extract Real Power into variable
  float apparentPower   = emon1.apparentPower;    //extract Apparent Power into variable
  float powerFActor     = emon1.powerFactor;      //extract Power Factor into Variable
  float supplyVoltage   = emon1.Vrms;             //extract Vrms into Variable
  float Irms            = emon1.Irms;             //extract Irms into Variable
  delay(1000);
// initialize pin ouput:
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(R3, OUTPUT);
  pinMode(R4, OUTPUT);
  pinMode(R5, OUTPUT);
  pinMode(R6, OUTPUT);
// initialize pin reading:
  pinMode(MCread,INPUT);
  pinMode(MVread,INPUT);
  pinMode(TCread,INPUT);
// initialize the pushbutton pin as an input:
  pinMode(Sw1, INPUT);
  pinMode(Sw2, INPUT);
  pinMode(Sw3, INPUT);
  pinMode(Rs1, INPUT);
  pinMode(Rs2, INPUT);
// initialize Encoder pin as an input:
  pinMode(CLK, INPUT_PULLUP); 
  pinMode(DT, INPUT_PULLUP);  
  pinMode(SW, INPUT_PULLUP);  
  attachInterrupt(digitalPinToInterrupt(DT), rotdial, CHANGE);
// welcome action
  lcd.setCursor(0,0);
  lcd.print("   XP-8003PLC   ");
  lcd.setCursor(0,1);
  lcd.print("PROG. LOGIC CTRL");    
  delay(10);
  tone(buzzerPIN, 1397, 200);
  delay(150);
  tone(buzzerPIN, 1568, 200);
  delay(150);
  tone(buzzerPIN, 2093, 300);
  delay(250);
  noTone(buzzerPIN);
  delay(1000);
  lcd.clear();
// POWER ON SELF TEST SEQUENCES
  lcd.setCursor(0,0);
  lcd.print("   INITIALIZE   ");
  

  delay(1);
// Test Current Reading
  Tadc = analogRead(TCread);
  Tvolt = (Tadc / 1024.0) * 500;
  Tcurr = ((Tvolt - Toffset) / Tsens);
  Twatt = (Tcurr*emon1.Vrms/1.2);
  delay(1);
// Main Current Protection 
  if (emon1.Irms > OCP){
  digitalWrite(R4, LOW);
  lcd.setCursor(0,1);
  lcd.print(" MAIN  OVERLOAD ");
  lcd.setCursor(0,0); 
  lcd.print("!!   ");          
  lcd.print(emon1.Irms);
  lcd.print(" A  !! ");
  Rs1Wait(Rs1State);
  }else { 
    lcd.setCursor(0,1); 
    digitalWrite(R4, HIGH);
    }
// Main Voltage Protection 
  if (emon1.Vrms > UVP && emon1.Vrms < OVP){
  lcd.setCursor(0,1);  
  lcd.print("   BOOTING.     ");  
  digitalWrite(R4, HIGH);
  delay(1000);
  }else { 
      digitalWrite(R4, LOW);
      lcd.setCursor(0,1);
      lcd.print("  MAIN VOLTAGE  ");
      lcd.setCursor(0,0);   
      lcd.print("!!  ");        
      lcd.print(emon1.Vrms);
      lcd.print(" V  !!");
      Rs1Wait(Rs1State);
    }
 // Main Current Protection 2
  if (emon1.Irms > OCP){
  digitalWrite(R4, LOW);
  digitalWrite(R5, LOW);
  lcd.setCursor(0,1);
  lcd.print(" MAIN  OVERLOAD ");
  lcd.setCursor(0,0); 
  lcd.print("!!   ");          
  lcd.print(emon1.Irms);
  lcd.print(" A  !! ");
  Rs1Wait(Rs1State);
  }else { 
    lcd.setCursor(0,1); 
    digitalWrite(R4, HIGH);
    digitalWrite(R5, HIGH);
    }
// Main Voltage Protection 2
  if (emon1.Vrms > UVP && emon1.Vrms < OVP){
  lcd.setCursor(0,1);  
  lcd.print("   BOOTING..    ");  
  digitalWrite(R4, HIGH);
  digitalWrite(R5, HIGH);
  delay(1000);
  }else { 
      digitalWrite(R4, LOW);
      digitalWrite(R5, LOW);
      lcd.setCursor(0,1);
      lcd.print("  MAIN VOLTAGE  ");
      lcd.setCursor(0,0);   
      lcd.print("!!  ");        
      lcd.print(emon1.Vrms);
      lcd.print(" V  !!");
      Rs1Wait(Rs1State);
    }
// Test Current Protection 
  if (Tcurr > TOCP){
  digitalWrite(R6, LOW);
  lcd.setCursor(0,1);
  lcd.print(" TEST  OVERLOAD ");
  lcd.setCursor(0,0);
  lcd.print("!!  ");           
  lcd.print(Tcurr);
  lcd.print("  A  !!   ");
  Rs2Wait(Rs2State);
  }
  else { 
      lcd.setCursor(0,1);
      lcd.print("   BOOTING...   "); 
      digitalWrite(R6, HIGH);
      delay(2000);
    }
  lcd.clear();
}
void loop()
{
  Sw1State = digitalRead(Sw1);
  Sw2State = digitalRead(Sw2);
  Sw3State = digitalRead(Sw3);
  Rs1State = analogRead(A6) < 100 ? 0 : 1;
  Rs2State = analogRead(A7) < 100 ? 0 : 1;

  DateTime now = rtc.now();
  emon1.calcVI(20,2000);         // Calculate all. No.of half wavelengths (crossings), time-out
 
  
  float realPower       = emon1.realPower;        //extract Real Power into variable
  float apparentPower   = emon1.apparentPower;    //extract Apparent Power into variable
  float powerFActor     = emon1.powerFactor;      //extract Power Factor into Variable
  float supplyVoltage   = emon1.Vrms;             //extract Vrms into Variable
  float Irms            = emon1.Irms;             //extract Irms into Variable

  currb=debounceButton(currb);

  if (mode==0 && currb==LOW && lastb==HIGH) {
    lcd.clear(); mode=1; mcur=0; mocp=OCP; movp=OVP; muvp=UVP; mtocp=TOCP;  
  }
  else if (mode==1 && currb==LOW && lastb==HIGH && mcur!=6) {
    lcd.clear(); mode=2; menu=mcur; mk=mkwh; mpp=peakPower; 
  }
  else if (mode==1 && currb==LOW && lastb==HIGH && mcur==6) {
    mode=0;
    lcd.clear();
  }
  else if (mode==2 && currb==LOW && lastb==HIGH) {
        lcd.clear();
    lcd.setCursor(0,0); sprintf(buf,"   SAVING ."); lcd.print(buf); delay(500);
    lcd.setCursor(0,0); sprintf(buf,"   SAVING .."); lcd.print(buf); delay(500);
    lcd.setCursor(0,0); sprintf(buf,"   SAVING ..."); lcd.print(buf); delay(500);
        tone(buzzerPIN, 4186, 20);
        delay(100);
        tone(buzzerPIN, 4186, 20);
        delay(50);
        noTone(buzzerPIN);
    savesetting();
    lcd.clear(); mode=1;
  }
//
 if (emon1.apparentPower > peakPower)
  {
    peakPower = emon1.apparentPower;
  }
  kilos = kilos + (emon1.apparentPower * (2.05/60/60/1000));    //Calculate kilowatt hours used
  mkwh = kilos * 1000;
// Test Current Reading
  Tadc = analogRead(TCread);
  Tvolt = (Tadc / 1024.0) * 500;
  Tcurr = ((Tvolt - Toffset) / Tsens);
  Twatt = (Tcurr*emon1.Vrms/1.2);
  delay(1);
// Main Current Protection 
  if (emon1.Irms > OCP){
  digitalWrite(R4, LOW);
  digitalWrite(R5, LOW);
  lcd.setCursor(0,1);
  lcd.print(" MAIN  OVERLOAD ");
  lcd.setCursor(0,0); 
  lcd.print("!!   ");          
  lcd.print(emon1.Irms);
  lcd.print(" A  !! ");
  Serial.print(" MAIN  OVERLOAD ");
  Rs1Wait(Rs1State);
  }else { 
    digitalWrite(R4, HIGH);
    digitalWrite(R5, HIGH);
    }
// Main Voltage Protection 
  if (emon1.Vrms > UVP ){ //&& emon1.Vrms < OVP
  digitalWrite(R4, HIGH);
  digitalWrite(R5, HIGH);
  }else { 
    digitalWrite(R4, LOW);
    digitalWrite(R5, LOW);
      lcd.setCursor(0,1);
      lcd.print("  MAIN VOLTAGE  ");
      lcd.setCursor(0,0);   
      lcd.print("!!  ");        
      lcd.print(emon1.Vrms);
      lcd.print(" V  !!");
      Serial.print(" MAIN  VOLTAGE ");
      Rs1Wait(Rs1State);
    }
// Test Current Protection 
  if (Tcurr > TOCP){
  digitalWrite(R6, LOW);
  lcd.setCursor(0,1);
  lcd.print(" TEST  OVERLOAD ");
  lcd.setCursor(0,0);
  lcd.print("!!  ");           
  lcd.print(Tcurr);
  lcd.print("  A  !!   ");
    Serial.print(" TEST  OVERLOAD ");
  Rs2Wait(Rs2State);
  }
  else { 
    digitalWrite(R6, HIGH);
    }
// Button 1 Function
  if (state1 == 0 && Sw1State == HIGH) {                       // turn ON Output 1:
    state1 = 1;
    Loadstate1=!Loadstate1;
  }
  if (state1 == 1 && Sw1State == LOW) {   
    state1 = 0;
  }
   if (Loadstate1==HIGH){
    digitalWrite(R1, HIGH);
    if (buzzerFlag1 == false ) {
        tone(buzzerPIN, 3520, 60);
        delay(50);
        noTone(buzzerPIN);
        buzzerFlag = false;
        buzzerFlag1 = true;
        }
    }
    else{                                                       // turn OFF Output 1:
    digitalWrite(R1, LOW); 
    if (buzzerFlag == false ) {
        tone(buzzerPIN, 4186, 20);
        delay(100);
        tone(buzzerPIN, 4186, 20);
        delay(50);
        noTone(buzzerPIN);
        buzzerFlag1 = false;
        buzzerFlag = true;
        }         
    }
// Button 2 Function
  if (state2 == 0 && Sw2State == HIGH) {                       // turn ON Output 2:
    state2 = 1;
    Loadstate2=!Loadstate2;
  }
  if (state2 == 1 && Sw2State == LOW) {   
    state2 = 0;
  }
   if (Loadstate2==HIGH){
    digitalWrite(R2, HIGH);
    if (buzzerFlag3 == false ) {
        tone(buzzerPIN, 3520, 60);
        delay(50);
        noTone(buzzerPIN);
        buzzerFlag2 = false;
        buzzerFlag3 = true;
        }
    }
    else{                                                       // turn OFF Output 2:
    digitalWrite(R2, LOW); 
    if (buzzerFlag2 == false ) {
        tone(buzzerPIN, 4186, 20);
        delay(100);
        tone(buzzerPIN, 4186, 20);
        delay(50);
        noTone(buzzerPIN);
        buzzerFlag3 = false;
        buzzerFlag2 = true;
        }         
    }
// Button 3 Function
  if (state3 == 0 && Sw3State == HIGH) {                       // turn ON Output 3:
    state3 = 1;
    Loadstate3=!Loadstate3;
  }
  if (state3 == 1 && Sw3State == LOW) {   
    state3 = 0;
  }
   if (Loadstate3==HIGH){
    digitalWrite(R3, HIGH);
    if (buzzerFlag5 == false ) {
        tone(buzzerPIN, 3520, 60);
        delay(50);
        noTone(buzzerPIN);
        buzzerFlag4 = false;
        buzzerFlag5 = true;
        }
    }
    else{                                                       // turn OFF Output 3:
    digitalWrite(R3, LOW); 
    if (buzzerFlag4 == false ) {
        tone(buzzerPIN, 4186, 20);
        delay(100);
        tone(buzzerPIN, 4186, 20);
        delay(50);
        noTone(buzzerPIN);
        buzzerFlag5 = false;
        buzzerFlag4 = true;
        }         
    }
//
  lcdshow();
  lastb=currb;
//
    if (SERIAL_OPTION) {
      
    Serial.print(now.year()); Serial.print("/");
    Serial.print(now.month()); Serial.print("/");
    Serial.print(now.day()); Serial.print(" ");
    Serial.print(now.hour()); Serial.print(":");
    Serial.print(now.minute()); Serial.print(":");
    Serial.print(now.second()); Serial.print("\n");
    Serial.print("A: ");
    Serial.println(emon1.Irms);
    Serial.print("W: ");
    Serial.println(emon1.apparentPower);
    Serial.print("kwh: ");
    Serial.println(kilos);
    Serial.print("Wp: ");
    Serial.println(peakPower);
    Serial.print("Voltage: ");
    Serial.println(emon1.Vrms);
    Serial.print("T-ADC = " );
    Serial.print(Tadc);
    Serial.print(" T-mV = ");
    Serial.print(Tvolt,3);
    Serial.print(" T-A = ");
    Serial.println(Tcurr,3);
    Serial.print(" OCP = ");
    Serial.println(OCP);
    Serial.print(" OVP = ");
    Serial.println(OVP);
    Serial.print(" UVP = ");
    Serial.println(UVP);
    Serial.print(" TOCP = ");
    Serial.println(TOCP);
    emon1.serialprint();           // Print out all variables (realpower, apparent power, Vrms, Irms, power factor)
    delay(1);  
      }
}
// end loop


boolean debounceButton(boolean state){
  boolean stateNow=digitalRead(SW);
  if (state!=stateNow) {
    delay(10);
    stateNow=digitalRead(SW);
  }
  return stateNow;
}

void lcdshow(){
      if (mode==0) {
     DateTime now = rtc.now();
     lcd.setCursor(0,0); 
     lcd.print(now.hour()); lcd.print(":"); lcd.print(now.minute()); lcd.print(":"); lcd.print(now.second()); lcd.print(" ");
     lcd.setCursor(8,0);
     lcd.print("  ");
     lcd.setCursor(0,1);             
     lcd.print(emon1.Vrms);
     lcd.setCursor(6,1);
     lcd.print(" V  ");
     lcd.setCursor(10,0);       
     lcd.print(emon1.Irms);
     lcd.print(" A ");
     lcd.setCursor(9,1);         
     lcd.print(emon1.apparentPower);
     lcd.setCursor(14,1);   
     lcd.print(" W");
  }
  if (mode==1 && mcur==0){ 
     DateTime now = rtc.now();
    lcd.setCursor(0,1); 
    lcd.print(("    ")); lcd.print(now.hour()); lcd.print(":"); lcd.print(now.minute()); lcd.print(":"); lcd.print(now.second()); lcd.print("     ");
    lcd.setCursor(0,0);
    lcd.print(("   ")); lcd.print(now.year()); lcd.print("/"); lcd.print(now.month()); lcd.print("/"); lcd.print(now.day()); lcd.print("    "); 
  }
    if (mode==1 && mcur==1){ 
    lcd.setCursor(0,0);             
    sprintf(buf," TEST EQUIPMENT ");
    lcd.print(buf);
    lcd.setCursor(0,1);             
    sprintf(buf,"   MONITORING   ",mcur==1?'>':' ');
    lcd.print(buf);  
  }
    if (mode==1 && mcur==2){ 
    lcd.setCursor(0,0);             
    sprintf(buf,"  OVER CURRENT  ");
    lcd.print(buf);
    lcd.setCursor(0,1);             
    sprintf(buf,"MAIN  PROTECTION",mcur==2?'>':' ');
    lcd.print(buf);  
  }
    if (mode==1 && mcur==3){ 
    lcd.setCursor(0,0);             
    sprintf(buf,"  OVER VOLTAGE  ");
    lcd.print(buf);
    lcd.setCursor(0,1);             
    sprintf(buf,"MAIN  PROTECTION",mcur==3?'>':' ');
    lcd.print(buf);  
  }
    if (mode==1 && mcur==4){ 
    lcd.setCursor(0,0);             
    sprintf(buf," UNDER  VOLTAGE ");
    lcd.print(buf);
    lcd.setCursor(0,1);             
    sprintf(buf,"MAIN  PROTECTION",mcur==4?'>':' ');
    lcd.print(buf);  
  }
    if (mode==1 && mcur==5){ 
    lcd.setCursor(0,0);             
    sprintf(buf,"  OVER CURRENT  ");
    lcd.print(buf);
    lcd.setCursor(0,1);             
    sprintf(buf," TEST EQUIPMENT ",mcur==5?'>':' ');
    lcd.print(buf);  
  }
    if (mode==1 && mcur==6){ 
    lcd.setCursor(0,0);             
    sprintf(buf,"      EXIT      ");
    lcd.print(buf); 
    lcd.setCursor(0,1);             
    sprintf(buf,"    SETTINGS     ",mcur==6?'>':' ');
    lcd.print(buf);  
  } 
  if (mode==2){
    switch (menu) {
      case 0:
        lcd.setCursor(0,0);           
        lcd.print(emon1.Irms);
        lcd.print(" A  ");
        lcd.setCursor(9,0);
        lcd.print(emon1.apparentPower);
        lcd.print(" W    ");
        lcd.setCursor(0,1);
        sprintf(buf,"%3d",mkwh);
        lcd.print(buf);
        lcd.print(" Wh  ");
        lcd.setCursor(9,1);
        sprintf(buf,"%3d Wp  ",peakPower);
        lcd.print(buf);
        return;
      case 1:
        lcd.setCursor(0,0);             
        lcd.print("PWR : ");
        lcd.print(Twatt);
        lcd.print("W     ");
        lcd.setCursor(0,1); 
        lcd.print("CUR : ");
        lcd.print(Tcurr);
        lcd.print("A     ");
        break;
      case 2:
        lcd.setCursor(0,0);             
        sprintf(buf,"   OCP  LIMIT  ");
        lcd.print(buf);
        lcd.setCursor(0,1);             
        sprintf(buf," CURRENT: %3d A",mocp);
        lcd.print(buf);
        break;
      case 3:
        lcd.setCursor(0,0);             
        sprintf(buf,"   OVP  LIMIT");
        lcd.print(buf);
        lcd.setCursor(0,1);             
        sprintf(buf," VOLTAGE: %3d V",movp);
        lcd.print(buf);
        break;
      case 4:
        lcd.setCursor(0,0);             
        sprintf(buf,"   UVP  LIMIT");
        lcd.print(buf);
        lcd.setCursor(0,1);             
        sprintf(buf," VOLTAGE: %3d V",muvp);
        lcd.print(buf);
        break;
      case 5:
        lcd.setCursor(0,0);             
        sprintf(buf," TEST OCP LIMIT");
        lcd.print(buf);
        lcd.setCursor(0,1);             
        sprintf(buf," CURRENT: %3d A",mtocp);
        lcd.print(buf);
        break;
    }    
  }
}

void loadsetting(){
  peakPower=EEPROM.read(1);
  OCP=EEPROM.read(2);
  OVP=EEPROM.read(3);
  UVP=EEPROM.read(4);
  TOCP=EEPROM.read(5);
//  byte value =EEPROM.read(6);
//  mkwh=EEPROM.read(6);
}
void initsetting(){
  if (EEPROM.read(0)!=101) {
    EEPROM.update(0,101);
    EEPROM.update(1,0);
    EEPROM.update(2,30);
    EEPROM.update(3,220);
    EEPROM.update(4,140);
    EEPROM.update(5,30);
  //  EEPROM.update(6,0);
 //   EEPROM.update(7,0);
 //   EEPROM.update(8,0);
  }
}
void savesetting(){
  peakPower=mpp; EEPROM.update(1,peakPower);
  OCP=mocp; EEPROM.update(2,OCP);
  OVP=movp; EEPROM.update(3,OVP);
  UVP=muvp; EEPROM.update(4,UVP);
  TOCP=mtocp; EEPROM.update(5,TOCP);
//  mkwh=mk; EEPROM.update(0x0006,mkwh1);
//  mkwh=mk; EEPROM.update(0x0007,mkwh2);
//  mkwh=mk; EEPROM.update(0x0008,mkwh3);
}

void rotdial() {
  if (mode==0) return;

  int direct = 0;                 
  int newCLK = digitalRead(CLK);  
  int newDT = digitalRead(DT);    
  if (newCLK != oldCLK) {         
    if (oldCLK == LOW) {          
      direct = -1*(oldDT * 2 - 1);    
    }
  }
  oldCLK = newCLK;  
  oldDT = newDT;    

  if (mode==1) {
    mcur+=direct; if (mcur>6) mcur=0; if (mcur<0) mcur=6; return;
  }
  
  switch (menu) {
      case 0: mk+=0.01*direct; if (mk>255) mk=0.00; if(mk<0.00) mk=255;break;
      case 1: mpp+=1*direct; if (mpp>255) mpp=0; if(mpp<0) mpp=255;break;
      case 2: mocp+=1*direct; if (mocp>30) mocp=0; if(mocp<0) mocp=30;break;
      case 3: movp+=1*direct; if (movp>255) movp=0; if(movp<0) movp=255;break;
      case 4: muvp+=1*direct; if (muvp>255) muvp=0; if(muvp<0) muvp=255;break;
      case 5: mtocp+=1*direct; if (mtocp>30) mtocp=0; if(mtocp<0) mtocp=30;break;
  }
}

void Rs1Wait(int Rs1State){
  int Rs1S = 0;
  while(1){
    Rs1S = analogRead(A6) < 100 ? 0 : 1;
      tone(A0, 3520, 60);
      delay(100);
      noTone(A0);
    if (Rs1S == HIGH) {
      lcd.clear();
      return;
    }
  }
}

void Rs2Wait(int Rs2State){
  int Rs2S = 0;
  while(1){
      tone(A0, 3520, 60);
      delay(500);
      noTone(A0);
    Rs2S = analogRead(A7) < 100 ? 0 : 1;
    if (Rs2S == HIGH) {
      lcd.clear();
      return;
    }
  }
}
