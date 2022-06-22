#include <Arduino.h>
#include <NewPing.h>

#define SONAR_NUM     4 // Number of sensors.

#define TRIGGER_PIN_SHOWER 22
#define ECHO_PIN_SHOWER 24
#define MAX_DISTANCE_SHOWER 200
#define LED_SHOWER 13
#define DELAY_PING_SONAR_SHOWER 2000 // Miliseconds..Set le timer between pings read sensor Shower


#define MAX_DISTANCE_TOILET 200
#define MAX_DISTANCE_ENTRY  50
#define MAX_DISTANCE_EXIT   50


#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).


enum sonars {  SONAR_SHOWER
              ,SONAR_TOILET
              ,SONAR_ENTRY
              ,SONAR_EXIT };

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

unsigned long timerSensorShower;

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(TRIGGER_PIN_SHOWER, ECHO_PIN_SHOWER, MAX_DISTANCE_SHOWER), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(12, 11, MAX_DISTANCE),
  NewPing(10, 9, MAX_DISTANCE),
  NewPing(8, 7, MAX_DISTANCE)
};


const int pino = 13;

void oneSensorCycle();
void echoCheck();



void setup() {
  timerSensorShower = millis();
  Serial.begin(9600);
  pinMode(LED_SHOWER, OUTPUT);

  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

}
void loop() {

  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      Serial.print("pingTimer[");
      Serial.print(i);
      Serial.print("]: ");
      Serial.println(pingTimer[i]);
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
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");

    switch (i)
    {
      case SONAR_SHOWER:
        if (millis() - timerSensorShower > DELAY_PING_SONAR_SHOWER){
          if (cm[i])
              digitalWrite(LED_SHOWER,HIGH);
          else
            { 
              digitalWrite(LED_SHOWER,LOW );
            }
          timerSensorShower = millis();
        }
        break;
        default:
          break;
    }        
  }
  Serial.println();
}