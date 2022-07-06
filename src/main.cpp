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
#define PING_INTERVAL_SONARS 29 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

// PINS SONARS
// sonar entry
#define TRIGGER_PIN_ENTRY 22
#define ECHO_PIN_ENTRY 25
#define MAX_DISTANCE_ENTRY 30 // Maximum distance (in cm) to ping.

// sonar exit
#define TRIGGER_PIN_EXIT 27
#define ECHO_PIN_EXIT 29
#define MAX_DISTANCE_EXIT 30 // Maximum distance (in cm) to ping.

// sonar toilet
//#define TRIGGER_PIN_TOILET 22
//#define ECHO_PIN_TOLIET 24
//#define MAX_DISTANCE_TOLIET 200

unsigned long timerPingSonars;

enum bath_state{ BATH_OFF,
                 BATH_ON };

enum sonars {  SONAR_ENTRY
              ,SONAR_EXIT
              ,SONAR_TOILET };

enum door_state{ DOOR_UNLOCKED = 1
                ,DOOR_LOCKED = 2
                ,DOOR_OFF = 3
};

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(TRIGGER_PIN_ENTRY, ECHO_PIN_ENTRY, MAX_DISTANCE_ENTRY), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(TRIGGER_PIN_EXIT, ECHO_PIN_EXIT, MAX_DISTANCE_EXIT),
  NewPing(0, 0, MAX_DISTANCE_EXIT)
};
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
  int _numberPeople;
  bool _entering_1;
  bool _entering_2;
  bool _going_out_1;
  bool _going_out_2;
public:
  Bath(Door* bathDoor , int state = BATH_OFF);
  int getBathState();
  void setBathState(int state);
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

void readShower();
void readSonars();

LedRgb *LedRgbDoor;
Door *DoorBath;
Bath *SmartBath;

void setup() {

  Serial.begin(9600);
  LedRgbDoor = new LedRgb(PIN_RGB_DOOR_R , PIN_RGB_DOOR_G, PIN_RGB_DOOR_B, led_rgb_door_anodo_comum);
  DoorBath = new Door(LedRgbDoor, DOOR_UNLOCKED);
  SmartBath = new Bath(DoorBath);
  timerPingSonars = millis();

  //INPUTS
  pinMode(PIN_PIR_SHOWER, INPUT);

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
  //delay(1000);
  //SmartBath->setBathState(BATH_ON);
  //readShower();
  //delay(5000);


}

// SHOWER
void readShower(){
    if ( digitalRead(PIN_PIR_SHOWER) && SmartBath->getBathState() == BATH_ON ){
      digitalWrite(LED_SHOWER,HIGH);
      Serial.println( "PRESENCE DETECTED IN SHOWER"); 
    }
    else
    { 
        digitalWrite(LED_SHOWER,LOW );
    }
}

void readSonars(){
  if (millis() - timerPingSonars > PING_INTERVAL_SONARS){
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
      //delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
      Serial.print(i);
      Serial.print("=");
      Serial.print(sonar[i].ping_cm());
      Serial.print("cm ");
    }
    timerPingSonars = millis();
    Serial.println();
  }
}

