#include <Arduino.h>
#include <NewPing.h>

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
#define TRIGGER_PIN_EXIT 27
#define ECHO_PIN_EXIT 29
#define MAX_DISTANCE_EXIT 60 // Maximum distance (in cm) to ping.

// sonar toilet
#define TRIGGER_PIN_TOILET 24
#define ECHO_PIN_TOLIET 26
#define MAX_DISTANCE_TOLIET 100 // Maximum distance (in cm) to ping.
// output Toilet
#define PIN_LED_TOILET 28
#define DELAY_OFF_LED_TOILET 5000 // ms

unsigned long timerPingSonars;
unsigned long timerLedsSonars;
bool sonarEntryDetected;
bool sonarExitDetected;
int personStateMovement;
bool isShowerOn;
bool isToiletOn;

enum bath_state{ BATH_OFF,
                 BATH_ON };

enum sonars {  SONAR_ENTRY
              ,SONAR_EXIT
              ,SONAR_TOILET };

enum door_state{ DOOR_UNLOCKED = 1
                ,DOOR_LOCKED = 2
                ,DOOR_OFF = 3
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
    LedRgb *_ledRgbDoor;
    int _state;
  public:
    Door( LedRgb* ledRgbDoor , int doorState = DOOR_OFF);
    void setState( int state );
    void update();
};

// Door Implementation
Door::Door( LedRgb* ledRgbDoor , int doorState){
  _ledRgbDoor = ledRgbDoor;
  _state = doorState;
}

void Door::setState( int state ){
  _state = state;
  this->update();
}
void Door::update(){
  if (_state == DOOR_UNLOCKED){
    Serial.println( "DOOR UNLOCKED");
    _ledRgbDoor->setColorGreen();
  } else if (_state == DOOR_LOCKED)
  {
    Serial.println( "DOOR LOCKED");
    _ledRgbDoor->setColorRed();
  } else {
    Serial.println( "DOOR OFF");
    _ledRgbDoor->off();
  } 
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
  int _numberPeople = 0;
public:
  Bath(Door* bathDoor , int state = BATH_OFF);
  int getBathState();
  void setBathState(int state);
  int getNumberPeople();
  void addPerson();
  void removePerson();
  void check();
  void reset( int numberPeople = 1);
};

// Door Implementation
Bath::Bath(Door* bathDoor , int state){
  _bathDoor = bathDoor;
  setBathState(state);
  //_state = state;
}

int Bath::getBathState(){
  return _state;
}
void Bath::setBathState(int state){
  _state = state;
  if (_state == BATH_ON){
    Serial.println("SMART BATH ON");
    _bathDoor->setState(DOOR_UNLOCKED);
  } else {
    _bathDoor->setState(DOOR_OFF);
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
    if (isShowerOn)
    {
      reset(1);
    }
  } 
}

void Bath::reset( int numberPeople ){
  setBathState( BATH_ON );
  _numberPeople = numberPeople;
  resetSonars();
}


LedRgb *LedRgbDoor;
Door *DoorBath;
Bath *SmartBath;
DelayMillis *DelayMovements;
DelayMillis *WaitingResetSonars;
DelayMillis *DelayOffLedToilet;

void setup() {

  Serial.begin(9600);
  
   //INPUTS
  pinMode(PIN_PIR_SHOWER, INPUT);

  //OUTPUTS
  pinMode(LED_SHOWER, OUTPUT);
  pinMode(PIN_LED_TOILET, OUTPUT);
 
  LedRgbDoor = new LedRgb(PIN_RGB_DOOR_R , PIN_RGB_DOOR_G, PIN_RGB_DOOR_B, led_rgb_door_anodo_comum);
  DoorBath = new Door(LedRgbDoor, DOOR_UNLOCKED);
  SmartBath = new Bath(DoorBath);
  DelayMovements = new DelayMillis(1000);
  WaitingResetSonars = new DelayMillis(TIMER_WAIT_RESET_SONARS);
  DelayOffLedToilet = new DelayMillis(DELAY_OFF_LED_TOILET);
  timerPingSonars = millis();
  timerLedsSonars = millis();

  sonarEntryDetected = false;
  sonarExitDetected = false;
  personStateMovement = WAITING;
  isShowerOn = false;
  
  LedRgbDoor->setup();
  LedRgbDoor->test();
  
  //pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  //for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
  //  pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

}
void loop() {
  //SmartBath->setBathState(BATH_OFF);
  readShower();
  readSonars();
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
  Serial.print( "  IS FINISHED->");
  Serial.println( DelayOffLedToilet->isFinished());
  //SmartBath->check();
  //delay(1000);
  //SmartBath->setBathState(BATH_ON);
  //readShower();
  //delay(5000);
}

// SHOWER
void readShower(){
    if ( digitalRead(PIN_PIR_SHOWER) ){
      digitalWrite(LED_SHOWER,HIGH);
      isShowerOn = true;
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
    
  //if (millis() - timerPingSonars > PING_INTERVAL_SONARS){
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
            DelayOffLedToilet->start(); 
      }

      if (i == SONAR_TOILET && sonar[i].ping_cm() == 0 && DelayOffLedToilet->isFinished() ){   
            digitalWrite ( PIN_LED_TOILET, LOW );
      }


       //else {
          //if (DelayOffLedToilet->isFinished()){
          //  digitalWrite ( PIN_LED_TOILET, LOW );
         //    DelayOffLedToilet->start();
         // }
      //}
    }
    //timerPingSonars = millis();
    Serial.println();
  //}
}

void resetSonars(){
  personStateMovement = WAITING;
  sonarEntryDetected = false;
  sonarExitDetected = false;
}

