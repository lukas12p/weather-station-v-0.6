#include <Wire.h>
#include <LiquidCrystal_I2C.h>                   //LCD
#include <VirtualWire.h>                        //
#include <FastLED.h>                          //RGB LED
#include <MPL3115A2.h>                       //Pressure sensor
#include <HTU21D.h>                           //Humi&Temp inside sensor
#include <SoftwareSerial.h>                  //BT RX,TX
//#include <RunningAverage.h>                  //for averaging fun ;)

#define DS3231_I2C_ADDRESS 0x68              // RTC adress
#define NUM_LEDS 1                          //how many RGB Leds
#define DATA_PIN 6                          //PIN RGB
#define encoder0PinA 3
#define encoder0PinB 12
#define encoderKeyPin 13
                                          
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
//////////////////////////////////////////////////// MENU + ENCODER
volatile int encoder0Pos = 1;
static boolean rotating=false;
unsigned int encValueTracking = 1;
enum pressDuration { normalPress, longPress, veryLongPress };
long presses[] = { 400, 1000, 2000 };
byte presss[]={1, 2, 3, 0};
int dur = 4;
byte menu = true;
byte s = 1;
byte key = 0;                                //button pressed
byte dh = false;
byte dti = false; byte ti = 5;
byte dBTi = false; unsigned int BTi = 60;
byte dlighti = false; unsigned int lighti = 120;
byte RTCset = false; int Dayi, Monthi, Yeari, Houri, Minutei = 1;
byte lineRTC[] = {1,1,1,1,1,1,2,3}; byte colRTC[] = {1,4,7,10,13,16,0,0};
///////////////////////////////////////////////////////
CRGB leds[NUM_LEDS];
byte jasnosc = 30;                            //brightness RGB Led
int wejscie = A0;                            // buttons
int d = 200;                                //global delay
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
float t; int h; float vvv; float tempt;
static int c = 0;
const byte STAT1 = 7; const byte STAT2 = 8;
const byte REFERENCE_3V3 = A3;
const byte LIGHT = A1; const byte BATT = A2;
float humiIn = 0; float tempIn = 0; float t1 = 0; float pressIn = 0; float p1 = 0; float pressRel = 0;
float DC = 11.8; float light = 455;
unsigned long last = 0; int interval = 0;
unsigned long last1 = 0; int interval1 = 0;
unsigned long last2 = 0; unsigned long interval2 = 0;
byte dzien = 0; byte godzina = 0; byte minuta = 0; byte sekunda = 0;
unsigned long last3 = -999; unsigned long interval3 = 0; unsigned long interval4 = 0;
int x = 0;
int wysokosc = 210;
byte przycisk = 0;
unsigned long BL;
byte z, z2, z3, z5;
/////////////////////////////////////////// AVG.
const int iloscPomiarowG = 60;
int odczytyG[iloscPomiarowG];
int indexG = 0;                  // the index of the current reading
int totalG = 0;                  // the running total
int averageG = 0;                // the average
float sr = 0, srG = 0;

const int iloscPomiarowD = 24*60;
int odczytyD[iloscPomiarowD];
int indexD = 0;                  // the index of the current reading
int totalD = 0;                  // the running total
int averageD = 0;                // the average
///////////////////////////////////////////////
MPL3115A2 myPress;
HTU21D myHumi;
///////////////////////////////////////////////
int bluetoothTx = 4; int bluetoothRx = 5;
SoftwareSerial BT(bluetoothTx, bluetoothRx);

/*----------------------------------------------------------------------------*/
void setup()
{
  pinMode(STAT1, OUTPUT);               //Status LED Blue .BT
  pinMode(STAT2, OUTPUT);               //Status LED Green .working
  pinMode(wejscie, INPUT);             //buttons
  pinMode(REFERENCE_3V3, INPUT);
  pinMode(LIGHT, INPUT);
  pinMode(3,OUTPUT);
  Wire.begin();  
                                        //Configure the pressure sensor
  myPress.begin();                       //Get sensor online
  myPress.setModeActive();
  myPress.setModeBarometer();            //Measure pressure in Pascals from 20 to 110 kPa
  myPress.setOversampleRate(7);         //Set Oversample to the recommended 128
  myPress.enableEventFlags();            //Enable all three pressure and temp event flags 
  
  pinMode(encoder0PinA, INPUT);           //Encoder
  digitalWrite(encoder0PinA, HIGH);       
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);
  pinMode(encoderKeyPin, INPUT); 
  digitalWrite(encoderKeyPin, HIGH); 
  attachInterrupt(1, rotEncoder, CHANGE);

                                        //Configure the humidity sensor
  myHumi.begin();
  
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  
  Serial.begin(9600);	// Debugging only
  Serial.println("START");// Debugging only
    
  BT.begin(9600);
  lcd.begin(20,4);
  lcd.backlight();lcd.setCursor(0,0);lcd.print("Stacja meteo"); ///////////////Intro
  lcd.setCursor(0,1);lcd.print("v.0.6  lukas12p");
  delay(3000);lcd.clear();lcd.noBacklight();          

  vw_setup(1000);	                 // RF433 rec. bits per sec
  vw_set_rx_pin(2);
  vw_rx_start();                         // Start the receiver PLL running
  delay(d);
}
/*----------------------------------------------------------------------------*/  
void rotEncoder()
{
  rotating=true; 
  while(rotating) 
  {
    delay(1);                             //Encoder button debounce + hardware debounce by 0.1uF !!!
    if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) { encoder0Pos++; } 
    else { encoder0Pos--; }
    rotating=false;
//    Serial.println(encoder0Pos);
  }
}
/*----------------------------------------------------------------------------*/
void loop()
{
  if (millis() < 1000) {last=0;last1=0;last2=0;last3=0;BL=0;interval3=0;interval4=0;} // reset timers if millis reset after ~50 days
  digitalWrite(STAT2, true);
  //////////////////////////////////////////////////////////////////////// ENCODER + BUTTON
  static unsigned long btnHeld = 0;
  rotating=true;
  // Upon button press...
  if((digitalRead(encoderKeyPin) == LOW) && !btnHeld)
  {
    btnHeld = millis();
  }
  // Upon button release...
  if((digitalRead(encoderKeyPin) == HIGH) && btnHeld)
  {
    long t = millis();
    t -= btnHeld;
    dur = veryLongPress;
    for(int i = 0; i<= veryLongPress; i++)
    {
      if(t > presses[i])
         continue;
      dur = i;
      break;
    }
    btnHeld = 0;
  }
  //////////////////////////////////////////////////////////////////// RF433 RECIEVER RX
  uint8_t buf[VW_MAX_MESSAGE_LEN];    
  uint8_t buflen = VW_MAX_MESSAGE_LEN;
 
      if (vw_get_message(buf, &buflen))   
      {
      String dane, temp, humi, v;
      int i;
        for (i = 0; i < 10; i++)
          {   dane +=char(buf[i]);  }
	for (i = 0; i < 4; i++)
          {   temp +=char(buf[i]);  }
        for (i > 4; i < 6; i++)
          {   humi +=char(buf[i]);  }
        for (i > 6; i < 10; i++)
          {   v +=char(buf[i]);  }
      
        t = temp.toFloat();
        t = t/10;                      //temp OUT
        h = humi.toFloat();            //humi OUT
        float vf = v.toInt();
        vvv = map (vf, 0, 1023, 0, 665);
        vvv = vvv / 100;                //V BAT
        
        last3 = millis();
        lcd.clear();
      }
///////////////////////////////////////////////////  
  interval3 = millis() - last3;
  interval4 = millis() - BL;
  interval = millis() - last;
/////////////////////////////////////////////////   INSIDE SENSORS READING  
  if (interval > ti*1000 )
    { humiIn = myHumi.readHumidity();
      t1 = myPress.readTemp();
      p1 = myPress.readPressure();
      light = get_light_level();
      DC = get_battery_level();
      last = millis();
      if (t1 > -50 && t1 < 60) {tempIn = t1;}          //IF ERROR
      if (p1 > 1000) {pressIn = p1;}
    }
   // Need to clen up..........
  if (z2 > 1 || lighti > 300000) z2 = 0;                                                          //MODE
  if (z5 > 3) z5 = 0;          //SET

  int odczyt = analogRead(wejscie);    //przyciski - drabinka
    
  if (odczyt < 950 && przycisk == 0)
  {
    if (odczyt < 10) 
    { z2 = z2 + 1;lcd.clear(); leds[0] = CRGB::Black; FastLED.show();}
               
    else if (odczyt < 200 & odczyt > 130) 
    { z = z + 1;lcd.clear(); leds[0] = CRGB::Black; FastLED.show();}
              
    else if (odczyt < 400 & odczyt > 280)
    { z3 = z3 + 1;lcd.clear(); leds[0] = CRGB::Black; FastLED.show();}  
            
    else if (odczyt < 650 & odczyt > 490)
    { z3 = z3 - 1;lcd.clear(); leds[0] = CRGB::Black; FastLED.show();}
    
    else if (odczyt < 880 & odczyt > 750)
    { z5 = z5 + 1;lcd.clear(); leds[0] = CRGB::Black; FastLED.show();}
    Serial.println(odczyt);        
    przycisk = przycisk++;   
    BL = millis();
  }
  
  if (odczyt > 950) przycisk = 0;
  
  digitalWrite(STAT2, false);  
///////////////////////////////////////////////////////////  STOPWATCH SINCE RESTART                         
  interval1 = millis() - last1;                  
  if (interval1 > 59998) {minuta++; last1 = millis();}
  if (minuta > 59)       {godzina++; minuta = 0;}
  if (godzina > 23)      {dzien++; godzina = 0;}
/////////////////////////////////////////////////////////// CLEARS LCD, SENDS BLUETOOTH DATA    
  interval2 = millis() - last2;  
  if (interval2 >= BTi*1000 )              
    {
    last2 = millis(); 
    digitalWrite(STAT1, true);
    lcd.clear();
    BT.print(dzien);BT.print(",");BT.print(godzina);BT.print(",");BT.print(minuta);BT.print(",");
    BT.print(t);BT.print(",");BT.print(h);BT.print(",");BT.print(vvv);BT.print(",");
    BT.print(tempIn);BT.print(",");BT.print(humiIn);BT.print(",");BT.print(pressIn/100);BT.print(",");BT.print(light);BT.println();
    digitalWrite(STAT1, false);
    sr = sredniaGodz() / 100;
    }

  if (menu)                                  // If We are in main menu then:
  { 
    if (encoder0Pos > 4 ) {encoder0Pos = 0;}
    if (encoder0Pos < 0) {encoder0Pos = 4;}
    if (z != encoder0Pos) { lcd.clear(); z = encoder0Pos;}
    if (presss[dur] == 1 && z < 4) { z2 = 1; dur = 4;}
  }
///////////////////////////////////////////////////////////////////////////  ALL MENU PAGES     
    switch (z)
    {
    case 0: {statsus(); break;}
    case 1: {meteo1(); break;}
    case 2: {meteo2(); break;}
    case 3: {baro(z5,z3); break;}
    case 4: {setu(); break;}
    case 5: {set0(); break;} 
    case 6: {set1(); break;}
    case 7: {set2(); break;}
    case 8: {RTCsetup(); break;}
    }
    switch (z2)
    {
    case 0: lcd.noBacklight(); break;
    case 1: lcd.backlight(); break;
    }
  
/////////////////////////////////////////////////////////////////////////////// 
 
  
  
  }  
/*----------------------------------------------------------------------------*///  MENU SETUP
void setu()
{
  lcd.setCursor(5,1);
  lcd.print("SETUP");
    
  if (presss[dur] == 1) { dur = 4; encoder0Pos = 0; lcd.clear(); set0();}
}
/*----------------------------------------------------------------------------*///  MENU
void set0()
{
  menu = false;
  
  if (encoder0Pos < 0 || encoder0Pos > 3) { encoder0Pos = 3; }
  if (s != encoder0Pos) { s = encoder0Pos; lcd.clear();}
  
  lcd.setCursor(1,0);
  lcd.print("Setup RTC");
  lcd.setCursor(0,s);
  lcd.print(">");
  lcd.setCursor(1,1);
  lcd.print("Setup 1");
  lcd.setCursor(1,2);
  lcd.print("Setup 2");
  lcd.setCursor(1,3);
  lcd.print("Back");
  
  z=5;
  if (s == 0 && presss[dur] == 1) { dur = 4; lcd.clear(); encoder0Pos = 0; 
    readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year); RTCsetup();}
  
  if (s == 1 && presss[dur] == 1) { dur = 4; lcd.clear(); encoder0Pos = 1; set1();}
  if (s == 2 && presss[dur] == 1) { dur = 4; lcd.clear(); encoder0Pos = 1; set2();}
  if (s == 3 && presss[dur] == 1) { encoder0Pos = 1; dur = 4; menu = true; lcd.clear();}  
}
/*----------------------------------------------------------------------------*///  RTC SETUP
void RTCsetup()
{
  menu = false;
  
  if (!RTCset) 
    {
    lcd.setCursor(colRTC[s],lineRTC[s]);
    if (s < 6) { lcd.print("||"); } else { lcd.print(">"); }
    if (encoder0Pos < 0 || encoder0Pos > 7) { encoder0Pos = 7; }
    if (s != encoder0Pos) { s = encoder0Pos; lcd.clear();}
    }
  
  lcd.setCursor(1,0);
  if (dayOfMonth<10) {lcd.print("0");}
  lcd.print(dayOfMonth);
  lcd.setCursor(3,0);
  lcd.print("/");
  lcd.setCursor(4,0);
  if (month<10) {lcd.print("0");}
  lcd.print(month);
  lcd.setCursor(6,0);
  lcd.print("/");
  lcd.setCursor(7,0);
  if (year<10) {lcd.print("0");}
  lcd.print(year);
  lcd.setCursor(10,0);
  if (hour<10) {lcd.print("0");}
  lcd.print(hour);
  lcd.setCursor(12,0);
  lcd.print(":");
  lcd.setCursor(13,0);
  if (minute<10) {lcd.print("0");}
  lcd.print(minute);
  lcd.setCursor(16,0);
  switch(dayOfWeek){
  case 1:
    lcd.print("Sun");
    break;
  case 2:
    lcd.print("Mon");
    break;
  case 3:
    lcd.print("Tue");
    break;
  case 4:
    lcd.print("Wen");
    break;
  case 5:
    lcd.print("Thr");
    break;
  case 6:
    lcd.print("Fri");
    break;
  case 7:
    lcd.print("Sat");
    break;
  }
  
  lcd.setCursor(1,2);
  lcd.print("Save & exit");
  lcd.setCursor(1,3);
  lcd.print("Back");
      
  z = 8;
  if (s == 0 && !RTCset && presss[dur] == 1) { RTCset = true; encoder0Pos = dayOfMonth; dur = 4; lcd.clear();}
  if (s == 0 && RTCset == 1) { 
    if (encoder0Pos > 31 || encoder0Pos < 1) { encoder0Pos = 1;}
    if (dayOfMonth != encoder0Pos) { dayOfMonth = encoder0Pos; lcd.clear();};}
  if (s == 0 && RTCset == 1 && presss[dur] == 1) { RTCset = false; dur = 4; encoder0Pos = 1; }
  
  if (s == 1 && !RTCset && presss[dur] == 1) { RTCset = true; encoder0Pos = month; dur = 4; lcd.clear();}
  if (s == 1 && RTCset == 1) { 
    if (encoder0Pos > 12 || encoder0Pos < 1) { encoder0Pos = 1;}
    if (month != encoder0Pos) { month = encoder0Pos; lcd.clear();};}
  if (s == 1 && RTCset == 1 && presss[dur] == 1) { RTCset = false; dur = 4; encoder0Pos = 2; }
  
  if (s == 2 && !RTCset && presss[dur] == 1) { RTCset = true; encoder0Pos = year; dur = 4; lcd.clear();}
  if (s == 2 && RTCset == 1) { 
    if (encoder0Pos > 50 || encoder0Pos < 00) { encoder0Pos = 10;}
    if (year != encoder0Pos) { year = encoder0Pos; lcd.clear();};}
  if (s == 2 && RTCset == 1 && presss[dur] == 1) { RTCset = false; dur = 4; encoder0Pos = 3; }
  
  if (s == 3 && !RTCset && presss[dur] == 1) { RTCset = true; encoder0Pos = hour; dur = 4; lcd.clear();}
  if (s == 3 && RTCset == 1) { 
    if (encoder0Pos > 23 || encoder0Pos < 0) { encoder0Pos = 0;}
    if (hour != encoder0Pos) { hour = encoder0Pos; lcd.clear();};}
  if (s == 3 && RTCset == 1 && presss[dur] == 1) { RTCset = false; dur = 4; encoder0Pos = 4; }
  
  if (s == 4 && !RTCset && presss[dur] == 1) { RTCset = true; encoder0Pos = minute; dur = 4; lcd.clear();}
  if (s == 4 && RTCset == 1) { 
    if (encoder0Pos > 59 || encoder0Pos < 0) { encoder0Pos = 0;}
    if (minute != encoder0Pos) { minute = encoder0Pos; lcd.clear();};}
  if (s == 4 && RTCset == 1 && presss[dur] == 1) { RTCset = false; dur = 4; encoder0Pos = 5; }
  
  if (s == 5 && !RTCset && presss[dur] == 1) { RTCset = true; encoder0Pos = dayOfWeek; dur = 4; lcd.clear();}
  if (s == 5 && RTCset == 1) { 
    if (encoder0Pos > 7 || encoder0Pos < 1) { encoder0Pos = 2;}
    if (dayOfWeek != encoder0Pos) { dayOfWeek = encoder0Pos; lcd.clear();};}
  if (s == 5 && RTCset == 1 && presss[dur] == 1) { RTCset = false; dur = 4; encoder0Pos = 6; }
    
  if (s == 6 && presss[dur] == 1) { encoder0Pos = 0; dur = 4; lcd.clear(); z = 5; 
      setDS3231time(00, minute, hour, dayOfWeek, dayOfMonth, month, year); }                //SAVE
  if (s == 7 && presss[dur] == 1) { encoder0Pos = 0; dur = 4; lcd.clear(); z = 5; }      //EXIT
}
/*----------------------------------------------------------------------------*///  MENU SETUP1
void set1()
{
  menu = false;
  if (!dh) 
    { 
    lcd.setCursor(0,s);
    lcd.print(">");
    if (encoder0Pos < 1 || encoder0Pos > 3) { encoder0Pos = 1; }
    if (s != encoder0Pos) { s = encoder0Pos; lcd.clear();}
    }
  
  lcd.setCursor(0,0);
  lcd.print("SETUP 1");
  lcd.setCursor(1,1);
  lcd.print("Wysokosc");
  lcd.setCursor(14,1);
  lcd.print(wysokosc);  
  lcd.setCursor(1,2);
  lcd.print("yyy");
  lcd.setCursor(1,3);
  lcd.print("Back");
    
  z = 6;
  if (s == 1 && !dh && presss[dur] == 1) { dh = true; encoder0Pos = wysokosc; dur = 4; lcd.clear(); }
  if (s == 1 && dh) { if (wysokosc != encoder0Pos) {wysokosc = encoder0Pos; lcd.clear();} }
  if (s == 1 && dh && presss[dur] == 1) { dh = false; dur = 4; encoder0Pos = 3;}
  
  if (s == 2 && presss[dur] == 1) { lcd.clear(); z = 5; dur = 4; }
  if (s == 3 && presss[dur] == 1) { encoder0Pos = 3; dur = 4; lcd.clear(); z = 5; }  
}
/*----------------------------------------------------------------------------*///  MENU SETUP2
void set2()
{
  menu = false;
  if (!dlighti && !dBTi) 
    { 
    lcd.setCursor(0,s);
    lcd.print(">");
    if (encoder0Pos < 1 || encoder0Pos > 3) { encoder0Pos = 1; }
    if (s != encoder0Pos) { s = encoder0Pos; lcd.clear();}
    }
  
  lcd.setCursor(0,0);
  lcd.print("SETUP 2");
  lcd.setCursor(1,1);
  lcd.print("Backlight");
  lcd.setCursor(14,1);
  lcd.print(lighti);
  lcd.setCursor(1,2);
  lcd.print("BT interval");
  lcd.setCursor(14,2);
  lcd.print(BTi);
  lcd.setCursor(1,3);
  lcd.print("Back");
    
  z = 7;
  if (s == 1 && !dlighti && presss[dur] == 1) { dlighti = true; encoder0Pos = lighti; dur = 4; lcd.clear();}
  if (s == 1 && dlighti == 1) { if (lighti != encoder0Pos) { lighti = encoder0Pos; lcd.clear();};}
  if (s == 1 && dlighti == 1 && presss[dur] == 1) { dlighti = false; dur = 4;}
  
  if (s == 2 && !dBTi && presss[dur] == 1) { dBTi = true; encoder0Pos = BTi; dur = 4; lcd.clear();}
  if (s == 2 && dBTi == 1) { if (BTi != encoder0Pos) { BTi = encoder0Pos; lcd.clear();}; }
  if (s == 2 && dBTi == 1 && presss[dur] == 1) { dBTi = false; dur = 4;}
  
  if (s == 3 && presss[dur] == 1) { encoder0Pos = 3; dur = 4; lcd.clear(); z = 5; }  
}
/*----------------------------------------------------------------------------*/
void baro (int q0, byte q1)
{
  static byte sw;
  static byte qq1;
  
  switch (sw)
  {
    case 0: break;
    case 1: break;
  }
  
  if (qq1>q1) wysokosc--;
  if (qq1<q1) wysokosc++;
  qq1=q1;
  
  tempt = t + 0.003*wysokosc; 
  pressRel = (pressIn/100)* exp((9.81*wysokosc)/(287.058*(273.16+tempt)));
  lcd.setCursor(0,0);
  lcd.print("REL");
  lcd.setCursor(4,0);
  lcd.print(pressRel,1);
  lcd.setCursor(11,0);
  lcd.print("h");
  lcd.setCursor(12,0);
  lcd.print(wysokosc);
  lcd.setCursor(0,1);
  lcd.print("ABS");
  lcd.setCursor(4,1);
  lcd.print(pressIn/100,1);
}
/*----------------------------------------------------------------------------*/
float humidex(float tt, int hh)      //
{
  float dew = pow((hh/100),(1/8)) * (112+(0.9*tt)) + (0.1*tt) - 112;
  float a = (5417.7530*((1/273.16)-(1/(273.16+dew))));
  float indexx = tt + 0.5555*(6.11 * exp(a)-10);
  return indexx; 
}
/*----------------------------------------------------------------------------*/
float get_light_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);
  float lightSensor = analogRead(LIGHT);
  operatingVoltage = 3.3 / operatingVoltage;     //Vref to 3.3V
  lightSensor = operatingVoltage * lightSensor;
  return(lightSensor);
}
/*----------------------------------------------------------------------------*/
float get_battery_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);
  float rawVoltage = analogRead(BATT);
  operatingVoltage = 3.30 / operatingVoltage; //odniesienie do 3.3V
  rawVoltage = operatingVoltage * rawVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin
  rawVoltage *= 4.90;                         //(3.9k+1k)/1k 
  return(rawVoltage);
}
/*----------------------------------------------------------------------------*/
long sredniaGodz()
{
  // subtract the last reading:
  totalG = totalG - odczytyG[indexG];        
  // read from the sensor:  
  odczytyG[indexG] = t * 100;
  // add the reading to the total:
  totalG = totalG + odczytyG[indexG];      
  // advance to the next position in the array:  
  indexG = indexG + 1;                    

  // if we're at the end of the array...
  if (indexG >= iloscPomiarowG)             
    // ...wrap around to the beginning:
    indexG = 0;                          

  // calculate the average:
  srG = totalG / iloscPomiarowG;
    return srG;        
}
/*----------------------------------------------------------------------------*/
void meteo1()
{
  int index = humidex(tempIn,humiIn);

  lcd.setCursor(0,0);
  lcd.print("Tin");
  lcd.setCursor(4,0);
  lcd.print(tempIn,2);
  lcd.setCursor(10,0);
  lcd.print("L");
  lcd.setCursor(12,0);
  lcd.print(light);
  lcd.setCursor(0,1);
  lcd.print("Hin");
  lcd.setCursor(4,1);
  lcd.print(humiIn,1);
  lcd.setCursor(10,1);
  lcd.print(pressIn/100,2);
                                            //RGB Led colour based on humidex
  FastLED.setBrightness(jasnosc);
  if (index < 30) {leds[0] = CRGB::Green;}
  else if (index >= 30 && index <40) leds[0] = CRGB::Yellow;
  else if (index >= 40 && index <46) leds[0] = CRGB::Orange;
  else if (index >= 46) leds[0] = CRGB::Red;
  FastLED.show();
  }
/*----------------------------------------------------------------------------*/
void meteo2()
{
  int index = humidex(t,h);

  lcd.setCursor(0,0);
  lcd.print("Tout");
  lcd.setCursor(5,0);
  lcd.print(t,1);
  lcd.setCursor(10,0);
  lcd.print("Av");
  lcd.setCursor(13,0);
  lcd.print(sr,1);
  lcd.setCursor(0,1);
  lcd.print("Hout");
  lcd.setCursor(5,1);
  lcd.print(h);
  lcd.setCursor(9,1);
  lcd.print("Vb ");
  lcd.setCursor(12,1);
  lcd.print(vvv);
                                            
  FastLED.setBrightness(jasnosc);
  if (index < 30) {leds[0] = CRGB::Green;}
  else if (index >= 30 && index <40) leds[0] = CRGB::Yellow;
  else if (index >= 40 && index <46) leds[0] = CRGB::Orange;
  else if (index >= 46) leds[0] = CRGB::Red;
  FastLED.show();
}
/*----------------------------------------------------------------------------*/
void statsus()
{
  unsigned long inter = interval3/1000; 
  lcd.setCursor(0,0); 
  lcd.print("Last rec:");
  lcd.setCursor(10,0);
  if (inter < 10) lcd.print("0");
  if (inter < 100) lcd.print("0");
  lcd.print(inter), 
  lcd.setCursor(15,0); 
  lcd.print("s");
  lcd.setCursor(0,1); 
  lcd.print("Online:");
  lcd.setCursor(8,1);
  if (dzien < 10) lcd.print("0");
  lcd.print(dzien), 
  lcd.setCursor(10,1); 
  lcd.print(":");
  lcd.setCursor(11,1);
  if (godzina < 10) lcd.print("0");
  lcd.print(godzina);
  lcd.setCursor(13,1); 
  lcd.print(":");
  lcd.setCursor(14,1);
  if (minuta < 10) lcd.print("0"); 
  lcd.print(minuta);
}
/*----------------------------------------------------------------------------*/
void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte
dayOfMonth, byte month, byte year)
{
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(second)); // set seconds
  Wire.write(decToBcd(minute)); // set minutes
  Wire.write(decToBcd(hour)); // set hours
  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(month)); // set month
  Wire.write(decToBcd(year)); // set year (0 to 99)
  Wire.endTransmission();
}
void readDS3231time(byte *second,
byte *minute,
byte *hour,
byte *dayOfWeek,
byte *dayOfMonth,
byte *month,
byte *year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}
                                          // Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}
