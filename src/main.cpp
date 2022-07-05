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

#define SONAR_NUM     4 // Number of sensors.
#define TRIGGER_PIN_SHOWER 22

#define ECHO_PIN_SHOWER 24
#define MAX_DISTANCE_SHOWER 200

#define MAX_DISTANCE_TOILET 200
#define MAX_DISTANCE_ENTRY  50
#define MAX_DISTANCE_EXIT   50

#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

enum sonars {  SONAR_SHOWER
              ,SONAR_TOILET
              ,SONAR_ENTRY
              ,SONAR_EXIT };

enum door_state{ DOOR_OPEN
                ,DOOR_CLOSED
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

class door
{
  private:
    int state;
    /* data */
  public:
    door( int state = DOOR_OPEN){
      this->state = state;
    };

    void update_status(){
    }
};

class ledRgb
{
  private:
    int pinR, pinG, pinB;
    bool led_rgb_door_anodo_comum;

  public:
    ledRgb(int pinPwmR, int pinPwmG, int pinPwmB, bool led_rgb_door_anodo_comum = false){
      pinR = pinPwmR;
      pinG = pinPwmG;
      pinB = pinPwmB;
    };

    void setup(){
        pinMode(pinR, OUTPUT);
        pinMode(pinG, OUTPUT);
        pinMode(pinB, OUTPUT);
    }

    void test(int timerIntervalTest = 500){
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

    void setColor( int red, int green, int blue ){
      if (led_rgb_door_anodo_comum){
        red = 255 - red;
        green = 255 - green;
        blue = 255 - blue;
      }
      analogWrite(pinR, red);
      analogWrite(pinG, green);
      analogWrite(pinB, blue);
    }

    void setColorGreen (){
      setColor(0,255,0);
    }

    void setColorRed (){
      setColor(255,0,0);
    }

    void setColorBlue (){
      setColor(0,0,255);
    }


    void off(){
      analogWrite(pinR, false);
      analogWrite(pinG, false);
      analogWrite(pinB, false);
    }

};

void oneSensorCycle();
void echoCheck();
void readShower();

ledRgb ledRgbDoor( PIN_RGB_DOOR_R , PIN_RGB_DOOR_G, PIN_RGB_DOOR_B, led_rgb_door_anodo_comum  );

void setup() {
  timerSensorShower = millis();
  Serial.begin(9600);

  //INPUTS
  pinMode(PIN_PIR_SHOWER, INPUT);


  // OUTPUTS
  //pinMode(LED_SHOWER, OUTPUT);
  //pinMode(PIN_RGB_DOOR_R, OUTPUT);
  //pinMode(PIN_RGB_DOOR_G, OUTPUT);
  //pinMode(PIN_RGB_DOOR_B, OUTPUT);

  ledRgbDoor.setup();
  ledRgbDoor.test();
  //ledRgbDoor.setColor(245,66,245);
  //ledRgbDoor.setColor(255,0,0);




  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

}
void loop() {

  // Serial.print( "RGB: (");
  // Serial.print( analogRead(PIN_RGB_DOOR_R));
  // Serial.print( ", ");
  // Serial.print( analogRead(PIN_RGB_DOOR_G));
  // Serial.print( ", ");
  // Serial.print( analogRead(PIN_RGB_DOOR_B));
  // Serial.println( " )");

  readShower();
  ledRgbDoor.setColor(245,66,245);

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

