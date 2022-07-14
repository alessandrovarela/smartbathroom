#include <Arduino.h>
#include <Wire.h>
#include <NewPing.h>
#include <LiquidCrystal_I2C.h>
#include <VarSpeedServo.h>
#include <TonePlayer.h>



// Servo Locker
#define PIN_SERVO_LOCKER 9
#define ANGLE_DOOR_LOCKED 90
#define ANGLE_DOOR_UNLOCKED 0
#define LOCK_SPEED 40
#define PIN_PUSH_BUTTON_DOOR1 27
#define PIN_BUZZER 5


// SHOWER
#define PIN_PIR_SHOWER 23
#define LED_SHOWER 13

// RGB DOOR
const bool led_rgb_door_anodo_comum = false;
#define PIN_RGB_DOOR_R 12
#define PIN_RGB_DOOR_G 11
#define PIN_RGB_DOOR_B 10


// SET SONARS
#define SONAR_NUM     3 // Number of sensors.
#define PING_INTERVAL_SONARS 50 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define INTERVAL_SET_LEDS_SONARS 3000 // Milliseconds
#define DELAY_MOVEMENT 1000 // Miliseconds
#define TIMER_WAIT_RESET_SONARS 10000 // ms 

// PINS SONARS
// sonar entry
#define TRIGGER_PIN_ENTRY 22
#define ECHO_PIN_ENTRY 25
#define MAX_DISTANCE_ENTRY 60 // Maximum distance (in cm) to ping.

// sonar exit
#define TRIGGER_PIN_EXIT 33
#define ECHO_PIN_EXIT 35
#define MAX_DISTANCE_EXIT 60 // Maximum distance (in cm) to ping.

// sonar toilet
#define TRIGGER_PIN_TOILET 24
#define ECHO_PIN_TOLIET 26
#define MAX_DISTANCE_TOLIET 100 // Maximum distance (in cm) to ping.
// output Toilet
#define PIN_LED_TOILET 28
#define DELAY_OFF_LED_TOILET 5000 // ms


// Display 1 Defitions
// DEFINIÇÕES
// set the LCD address to 0x27 for a 16 chars and 2 line display
#define LCD_1_ADDRESS 0x27 
#define LCD_1_COLUMNS 16
#define LCD_1_LINES 2
#define DELAY_MESSAGE_DISPLAY 2000 // ms

bool sonarEntryDetected;
bool sonarExitDetected;
 int personStateMovement;
bool isShowerOn;
bool isToiletOn;
bool isDoorlocked;
bool isAnySensorDetected;
bool isSmarthBathOn = false;

enum bath_state{ BATH_OFF,
                 BATH_ON };

enum sonars {  SONAR_ENTRY
              ,SONAR_EXIT
              ,SONAR_TOILET };

enum door_state{ DOOR_LOCKING
                ,DOOR_UNLONCKING
                ,DOOR_UNLOCKED
                ,DOOR_LOCKED
                ,DOOR_OFF
};

enum  messages{ MSG_DOOR_LOCKED
               ,MSG_DOOR_UNLOCKED
};

enum state_movements{ WAITING,
                      ENTERING,
                      LEAVING
};




NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(TRIGGER_PIN_ENTRY, ECHO_PIN_ENTRY, MAX_DISTANCE_ENTRY), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(TRIGGER_PIN_EXIT, ECHO_PIN_EXIT, MAX_DISTANCE_EXIT),
  NewPing(TRIGGER_PIN_TOILET, ECHO_PIN_TOLIET, MAX_DISTANCE_TOLIET)
};

void checkSonars();
void readShower();
void readSonars();
void resetSonars();
void showMessageDisplay1( int message );

//*************************************************
// Class DelayMillis
//*************************************************
// DelayMillis Declarations
class DelayMillis
{
private:
  unsigned long _ms;
  unsigned long timerDelayMillis; 
public:
  DelayMillis(unsigned long = 500);
  void start();
  bool isFinished();
};

// DelayMillis Implementation
DelayMillis::DelayMillis(unsigned long ms)
{
  _ms = ms;
  start();
}

void DelayMillis::start()
{
  timerDelayMillis = millis();
}

bool DelayMillis::isFinished()
{
  return millis() - timerDelayMillis > _ms;
}

DelayMillis *DelayShowMessageDisplay;

class Stopwatch
{
  private:
    unsigned long _startms, _sectotal; 
    int _hours, _min, _sec = 0; 
  public:
    Stopwatch();
    void reset();
    void updateStopwatch();
    void convert(long sectotal);
    String show();
};

Stopwatch::Stopwatch(){}

void Stopwatch::reset(){
  _hours = 0;
  _min = 0;
  _sec = 0;
  _startms = millis();
  Serial.println( "STOPWATCH RESETED");
}
void Stopwatch::updateStopwatch(){
  _sectotal = ((millis()-_startms)/1000);
}
void Stopwatch::convert(long sectotal){
  int r = 0 ;
  _hours = sectotal / 3600;
  r = sectotal % 3600;
  _min = r / 60;
  _sec = r % 60;
}
String Stopwatch::show(){
  updateStopwatch();
  convert(_sectotal);
  String stopwatch;
  stopwatch = (_hours < 10 ? "0" :"" ) + String(_hours) + ":";
  stopwatch += (_min < 10 ? "0" : "" )+ String(_min) + ":";
  stopwatch += (_sec  <  10 ? "0" : "") + String(_sec);
  return stopwatch;
 }


//*************************************************
// Class LedRgb
//*************************************************
// LedRGB Declarations
class LedRgb
{
  private:
    int pinR, pinG, pinB;
    bool _ledRgbDoorAnodoComum;

  public:
    LedRgb(int pinPwmR, int pinPwmG, int pinPwmB, bool led_rgb_door_anodo_comum = false);
    void setup();
    void test(int timerIntervalTest = 500);
    void setColor( int red, int green, int blue );
    void setColorGreen();
    void setColorRed ();
    void setColorBlue();
    void off();
};

// LedRGB Implementation
LedRgb::LedRgb(int pinPwmR, int pinPwmG, int pinPwmB, bool ledRgbDoorAnodoComum){
  pinR = pinPwmR;
  pinG = pinPwmG;
  pinB = pinPwmB;
  _ledRgbDoorAnodoComum = ledRgbDoorAnodoComum;

}

void LedRgb::setup(){
  pinMode(pinR, OUTPUT);
  pinMode(pinG, OUTPUT);
  pinMode(pinB, OUTPUT);
}

void LedRgb::test(int timerIntervalTest){
  setColorRed();
  delay(timerIntervalTest);
  off();
  delay(timerIntervalTest);
  setColorGreen();
  delay(timerIntervalTest);
  off();
  delay(timerIntervalTest);
  setColorBlue();
  delay(timerIntervalTest);
  off();
}

void LedRgb::setColor( int red, int green, int blue ){
  if (_ledRgbDoorAnodoComum){
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  }
  analogWrite(pinR, red);
  analogWrite(pinG, green);
  analogWrite(pinB, blue);
  // Serial.print("setColor( ");
  // Serial.print(red);
  // Serial.print(", ");
  // Serial.print(green);
  // Serial.print(", ");
  // Serial.print(blue);
  // Serial.println(")");
}

void LedRgb::setColorGreen (){
  //Serial.println("SET COLOR GREEN");
  this->setColor(0,255,0);
}

void LedRgb::setColorRed (){
  //Serial.println("SET COLOR RED");
  setColor(255,0,0);
}

void LedRgb::setColorBlue (){
  setColor(0,0,255);
}

void LedRgb::off(){
  analogWrite(pinR, false);
  analogWrite(pinG, false);
  analogWrite(pinB, false);
}

//*************************************************
// Class Door
//*************************************************
// Door Declarations
class Door
{
  private:
    DelayMillis *DelayDebounce;
    LedRgb* _ledRgbDoor;
    VarSpeedServo* _servo;
    TonePlayer* _tonePlayer;
    int _state;
  public:
    Door( LedRgb* ledRgbDoor, VarSpeedServo* servo, TonePlayer* tonePlayer, int doorState = DOOR_OFF);
    void lock();
    void unlock();
    void setState( int state );
     int getState();
    void playSoundUnlock();
    void checkButtons();
    void check();
};

// Door Implementation
Door::Door( LedRgb* ledRgbDoor, VarSpeedServo* servo, TonePlayer* tonePlayer, int doorState){
  _ledRgbDoor = ledRgbDoor;
  _servo = servo;
  _tonePlayer = tonePlayer;
  _state = doorState;
  DelayDebounce = new DelayMillis(50);
}


void Door::lock(){
  _servo->attach(PIN_SERVO_LOCKER);
  _servo->slowmove(ANGLE_DOOR_LOCKED,LOCK_SPEED);
  setState(DOOR_LOCKING);
}

void Door::unlock(){
    _servo->attach(PIN_SERVO_LOCKER);
    _servo->slowmove(ANGLE_DOOR_UNLOCKED,LOCK_SPEED);
    setState(DOOR_UNLONCKING);
}

void Door::setState( int state ){
  _state = state;
}

int Door::getState(){
  return _state;
}
void Door::playSoundUnlock(){
  //tone1.tone (220);  // 220 Hz
  //delay (500);
  //tone1.noTone ();

  //tone1.tone (440);
  //delay (500);
  //tone1.noTone ();
  _tonePlayer->tone(880);
  delay (200);
  _tonePlayer->noTone();
}

void Door::checkButtons(){
  if ( DelayDebounce->isFinished() ){
    if ( !digitalRead(PIN_PUSH_BUTTON_DOOR1)){
      if ( getState() == DOOR_UNLOCKED){
        lock();
        DelayDebounce->start();
      }
      if ( getState() == DOOR_LOCKED){
        unlock();
        DelayDebounce->start();
      }
      isAnySensorDetected = true;
    }
  }
}

void Door::check(){
  if (_servo->read() == ANGLE_DOOR_LOCKED && getState() != DOOR_LOCKED){
    setState(DOOR_LOCKED);
    _ledRgbDoor->setColorRed();
    _servo->detach();
    isDoorlocked = false;
    Serial.println( "DOOR LOCKED");
  }

  if (_servo->read() == ANGLE_DOOR_UNLOCKED && getState() != DOOR_UNLOCKED ){
    setState(DOOR_UNLOCKED);
    _ledRgbDoor->setColorGreen();
    _servo->detach();
    playSoundUnlock();
    isDoorlocked = false;
    showMessageDisplay1(MSG_DOOR_UNLOCKED);
    Serial.println( "DOOR UNLOCKED");
  } 
  if (!isSmarthBathOn){
    _ledRgbDoor->off();
    Serial.println( "DOOR OFF");
  } 
  checkButtons();
}

//*************************************************
// Class Bath
//*************************************************
// Bath Declarations
class Bath
{
private:
  int _state;
  Door* _bathDoor;
  Stopwatch* _stopwatch;
  LiquidCrystal_I2C* _lcd1;
  int _numberPeople = 0;
public:
  Bath(Door* bathDoor, Stopwatch* watch, LiquidCrystal_I2C* lcd1, int state = BATH_OFF);
   int getBathState();
  void setBathState(int state);
   int getNumberPeople();
  void addPerson();
  void removePerson();
  void check();
  void reset( int numberPeople = 1);
  void showDisplayExtenal( LiquidCrystal_I2C* lcd , Stopwatch* watch, bool resetWatch = true);
  void turnOffDisplayExternal(LiquidCrystal_I2C* lcd);
};

// Door Implementation
Bath::Bath(Door* bathDoor, Stopwatch* watch, LiquidCrystal_I2C* lcd1, int state){
  _bathDoor = bathDoor;
  _stopwatch = watch;
  _lcd1 = lcd1;
  setBathState(state);
}

int Bath::getBathState(){
  return _state;
}
void Bath::setBathState(int state){
  _state = state;
  if (_state == BATH_ON){
    Serial.println("SMART BATH ON");
    _bathDoor->unlock();
    showDisplayExtenal( _lcd1, _stopwatch, true);
  } else {
    _bathDoor->setState(DOOR_OFF);
    turnOffDisplayExternal(_lcd1);
    Serial.println("SMART BATH OFF");
  }

}

int Bath::getNumberPeople(){
  return _numberPeople;
}

void Bath::addPerson(){
  _numberPeople++;
  Serial.print( "ADDED PEOPLE. TOTAL: ");
  Serial.println(getNumberPeople());
  if ( getBathState() == BATH_OFF){
    setBathState(BATH_ON);
  }
}

void Bath::removePerson(){
  _numberPeople--;
  Serial.print( "REMOVED PEOPLE. TOTAL: ");
  Serial.println(getNumberPeople());
  if ( getNumberPeople() == 0 && getBathState() == BATH_ON ){
    setBathState(BATH_OFF);
  }
}

void Bath::check(){
  if (_state == BATH_OFF)
  {
    if (isAnySensorDetected)
    {
      //reset(1);
    }
    isSmarthBathOn = false;
  } 

  if (getBathState() == BATH_ON ){
    isSmarthBathOn = true;
    showDisplayExtenal( _lcd1, _stopwatch, false);
  }

  _bathDoor->check();

}

void Bath::reset( int numberPeople ){
  setBathState( BATH_ON );
  _numberPeople = numberPeople;
  resetSonars();
  showDisplayExtenal( _lcd1, _stopwatch, true );
}

void Bath::showDisplayExtenal( LiquidCrystal_I2C* lcd , Stopwatch* watch, bool resetWatch){
  // lcd->clear();
  if (resetWatch) { watch->reset(); };
  if (DelayShowMessageDisplay->isFinished()){
    lcd->backlight();
    lcd->setCursor(0,0);
    lcd->print( watch->show() );
    lcd->print("        ");
    lcd->setCursor(0, 1);
    lcd->print( "Pessoas: " );
    lcd->print( getNumberPeople() );
    lcd->print("     ");
  }
}
void Bath::turnOffDisplayExternal(LiquidCrystal_I2C* lcd){
  lcd->noBacklight();
}

LedRgb *LedRgbDoor;
Door *DoorBath;
Bath *SmartBath;
DelayMillis *DelayMovements;
DelayMillis *WaitingResetSonars;
DelayMillis *DelayOffLedToilet;

Stopwatch *Stopwatch1;

LiquidCrystal_I2C lcd1(LCD_1_ADDRESS, LCD_1_COLUMNS, LCD_1_LINES);
VarSpeedServo servoLocker;
TonePlayer tone1 (TCCR3A, TCCR3B, OCR3AH, OCR3AL, TCNT3H, TCNT3L);  // pin D5 MEGA 2560

void setup() {

  Serial.begin(9600);

   //INPUTS
  pinMode(PIN_PIR_SHOWER, INPUT);
  pinMode(PIN_PUSH_BUTTON_DOOR1, INPUT_PULLUP);

  //OUTPUTS
  pinMode(LED_SHOWER, OUTPUT);
  pinMode(PIN_LED_TOILET, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  LedRgbDoor = new LedRgb(PIN_RGB_DOOR_R , PIN_RGB_DOOR_G, PIN_RGB_DOOR_B, led_rgb_door_anodo_comum);
  DoorBath = new Door(LedRgbDoor, &servoLocker, &tone1, DOOR_UNLOCKED);
  Stopwatch1 = new Stopwatch();
  SmartBath = new Bath(DoorBath, Stopwatch1, &lcd1);
  DelayMovements = new DelayMillis(DELAY_MOVEMENT);
  WaitingResetSonars = new DelayMillis(TIMER_WAIT_RESET_SONARS);
  DelayOffLedToilet = new DelayMillis(DELAY_OFF_LED_TOILET);
  DelayShowMessageDisplay = new DelayMillis(DELAY_MESSAGE_DISPLAY);
  

  sonarEntryDetected = false;
  sonarExitDetected = false;
  personStateMovement = WAITING;
  isShowerOn = false;
  isToiletOn = false;
  isDoorlocked = false;
  isAnySensorDetected = false;

  //servoLocker.attach(PIN_SERVO_LOCKER);

  
  LedRgbDoor->setup();
  LedRgbDoor->test();

  lcd1.init(); // start comunication with display
  lcd1.backlight(); // Turno on backlight
  lcd1.clear(); // clear display

  lcd1.print("-SMART BATHROOM-");
  //delay(2000);
  //lcd1.noBacklight(); 
  lcd1.setCursor(0, 1); // set cursor primary column and line 2
  //lcd1.scrollDisplayLeft();
  lcd1.print("SETUP COMPLETED");
  delay(2000); 
  lcd1.noBacklight();
  lcd1.clear();

  //tone(PIN_BUZZER,1000,100);
  //tone(speaker, 440,1000);
  //DoorBath->lock();
  //delay(5000);
  //DoorBath->unlock();
  //delay(5000);

  //servoLocker.slowmove(0,40);
 // servoLocker.write(0);
 // delay(2000);
 // Serial.print("READ SERVER LOCKER->");
 // Serial.println(servoLocker.read());
 // delay(5000);
 // servoLocker.write(90);
 // Serial.print("READ SERVER LOCKER->");
 // Serial.println(servoLocker.read());
 // delay(5000);

}
void loop() {
  //stopwatch->start();
  readShower();
  readSonars();
  SmartBath->check();
  Serial.print("              NUMBER PEOPLE:");
  Serial.print(SmartBath->getNumberPeople());
  Serial.print( "  SONAR_ENTRY:");
  Serial.print( sonarEntryDetected);
  Serial.print( "  SONAR_EXIT: ");
  Serial.print( sonarExitDetected);
  Serial.print( "  STATE MOVMENT->");
  Serial.print( personStateMovement);
  Serial.print( "  BATH STATE->");
  Serial.print( SmartBath->getBathState());
  Serial.print( "  STOPWATCH->");
  Serial.println( Stopwatch1->show());

  //Serial.print( "PUSH BUTTON 1->");
  //Serial.println(digitalRead(PIN_PUSH_BUTTON_DOOR1));
  
}

// SHOWER
void readShower(){
    if ( digitalRead(PIN_PIR_SHOWER) ){
      digitalWrite(LED_SHOWER,HIGH);
      isShowerOn = true;
      isAnySensorDetected = true;
      Serial.println( "PRESENCE DETECTED IN SHOWER"); 
    }
    else
    { 
      digitalWrite(LED_SHOWER,LOW );
      isShowerOn = false;
    }
}


void checkSonars(){
  if ( ( personStateMovement == ENTERING || personStateMovement == LEAVING ) &&  WaitingResetSonars->isFinished() )
  {
    personStateMovement = WAITING;
    sonarEntryDetected = false;
    sonarExitDetected = false;
  }  
}

void readSonars(){
    
    checkSonars();
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
      Serial.print(i);
      Serial.print("=");
      Serial.print(sonar[i].ping_cm());
      Serial.print("cm ");
      if (personStateMovement == WAITING && DelayMovements->isFinished()){
        if ( i == SONAR_ENTRY && sonar[i].ping_cm() > 0 ){
          personStateMovement = ENTERING;
          sonarEntryDetected = true;
          WaitingResetSonars->start();
        }
      }

      if (personStateMovement == WAITING && DelayMovements->isFinished()){
        if ( i == SONAR_EXIT && sonar[i].ping_cm() > 0 ){
          personStateMovement = LEAVING;
          sonarExitDetected = true;
          WaitingResetSonars->start();
        }
      }

      if ( personStateMovement == ENTERING ) {
        if ( sonarEntryDetected && i == SONAR_EXIT && sonar[i].ping_cm() > 0 ){
          sonarExitDetected = true;
        }
        if ( sonarEntryDetected && sonarExitDetected && i == SONAR_EXIT && sonar[i].ping_cm() == 0 ){
            personStateMovement = WAITING;
            sonarEntryDetected = false;
            sonarExitDetected = false;
            SmartBath->addPerson();
            DelayMovements->start();
        }
      }

      if (personStateMovement == LEAVING){
        if ( sonarExitDetected && i == SONAR_ENTRY && sonar[i].ping_cm() > 0  ){
          sonarEntryDetected = true;
        }
        if ( sonarExitDetected && sonarEntryDetected && i == SONAR_ENTRY && sonar[i].ping_cm() == 0 ){
            personStateMovement = WAITING;
            sonarEntryDetected = false;
            sonarExitDetected = false;
            SmartBath->removePerson();
            DelayMovements->start();
        }
      }

      if (i == SONAR_TOILET && sonar[i].ping_cm() > 0 ){   
        digitalWrite ( PIN_LED_TOILET, HIGH );
        isToiletOn = true;
        isAnySensorDetected = true;
        DelayOffLedToilet->start(); 
      }

      if (i == SONAR_TOILET && sonar[i].ping_cm() == 0 && DelayOffLedToilet->isFinished() ){   
        digitalWrite ( PIN_LED_TOILET, LOW );
        isToiletOn = false;
      }
    }
    Serial.println();
}

void resetSonars(){
  personStateMovement = WAITING;
  sonarEntryDetected = false;
  sonarExitDetected = false;
}

void showMessageDisplay1( int message ){

  DelayShowMessageDisplay->start();
  switch (message)
  {
    case MSG_DOOR_UNLOCKED:
      lcd1.setCursor(0, 0);
      lcd1.print("     PORTA      ");
      lcd1.setCursor(0, 1);
      lcd1.print("   DESTRAVADA   ");
      break;  
    default:
      break;
  }
}