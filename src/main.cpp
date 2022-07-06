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

// const int  PIN_RGB_DOOR_R = 12;
// const int  PIN_RGB_DOOR_G = 11;
// const int  PIN_RGB_DOOR_B = 10;


#define SONAR_NUM     4 // Number of sensors.
#define TRIGGER_PIN_SHOWER 22

#define ECHO_PIN_SHOWER 24
#define MAX_DISTANCE_SHOWER 200

#define MAX_DISTANCE_TOILET 200
#define MAX_DISTANCE_ENTRY  50
#define MAX_DISTANCE_EXIT   50

#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

enum bath_state{ BATH_OFF,
                 BATH_ON };

enum sonars {  SONAR_SHOWER
              ,SONAR_TOILET
              ,SONAR_ENTRY
              ,SONAR_EXIT };

enum door_state{ DOOR_UNLOCKED = 1
                ,DOOR_LOCKED = 2
                ,DOOR_OFF = 3
};

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

unsigned long timerSensorShower;

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(TRIGGER_PIN_SHOWER, ECHO_PIN_SHOWER, MAX_DISTANCE_SHOWER), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(9, 8, MAX_DISTANCE),
  NewPing(7, 6, MAX_DISTANCE),
  NewPing(5, 4, MAX_DISTANCE)
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
void oneSensorCycle();
void echoCheck();
void readShower();

LedRgb *LedRgbDoor;
Door *DoorBath;
Bath *SmartBath;

void setup() {

  Serial.begin(9600);
  LedRgbDoor = new LedRgb(PIN_RGB_DOOR_R , PIN_RGB_DOOR_G, PIN_RGB_DOOR_B, led_rgb_door_anodo_comum);
  DoorBath = new Door(LedRgbDoor, DOOR_UNLOCKED);
  SmartBath = new Bath(DoorBath);
  // timerSensorShower = millis();

  //INPUTS
  pinMode(PIN_PIR_SHOWER, INPUT);

  LedRgbDoor->setup();
  LedRgbDoor->test();
  
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

}
void loop() {
  //LedRgbDoor->setColorRed();
  delay(5000);
  // DoorBath->setState(DOOR_UNLOCKED);
  SmartBath->setBathState(BATH_ON);
  readShower();
  delay(5000);
  // DoorBath->setState(DOOR_LOCKED);
  SmartBath->setBathState(BATH_OFF);
  delay(5000);
  //smartBath.setBathState(BATH_ON);
  //Serial.print("BATH STATE: ");
  //Serial.println(smartBath.getBathState());
  // doorBath.update();
  //Serial.print("LOOP STATUS DOOR->");
  //Serial.println(doorBath.state);
  //LedRgbDoor.test();
  //delay(3000);
  //smartBath.setBathState(BATH_ON);
  //Serial.print("BATH STATE: ");
  //Serial.println(smartBath.getBathState());
  //delay(3000);

  // LedRgbDoor.setColor(245,66,245);


  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      // Serial.print("pingTimer[");
      // Serial.print(i);
      // Serial.print("]: ");
      // Serial.println(pingTimer[i]);
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }

}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    //Serial.print(i);
    //Serial.print("=");
    //Serial.print(cm[i]);
    //Serial.print("cm ");

    switch (i)
    {
      case SONAR_SHOWER:
        if (cm[i])
        //   digitalWrite(LED_SHOWER,HIGH);
        // else
        //   digitalWrite(LED_SHOWER,LOW );        
        // break;
        default:
          break;
    }        
  }
  //Serial.println();
}


// SHOWER
void readShower(){
  //if (millis() - timerSensorShower > DELAY_PING_SENSOR_SHOWER){
    if ( digitalRead(PIN_PIR_SHOWER) ){
      digitalWrite(LED_SHOWER,HIGH);
      Serial.println( "PRESENÇA CHUVEIRO DETECTADA"); 
    }
    else
      { 
        digitalWrite(LED_SHOWER,LOW );
    }
  //timerSensorShower = millis();
  //}
}

