//
// Dapol class 08/09
// this model has a highly geared motor so needs a 'nudge' to overcome
// the initial inertia of the gearbox
// based on the 'blinky' EmbAJAX library example

// v1 - version for the Seed Studio XIAO ESP32C6 based hardware 

#include <Arduino.h>
#include <WiFi.h>
#include <EmbAJAX.h>
 
#define DEBUG 1 // 0 = no debug

// hardware defines
// LEDs
#define USER_LED          15   // GPIO pin for heartbeat LED
#define LED_TIMEOUT       250  //Heartbeat LED cycle time in ms

// H-bridge pins
#define IN1        18
#define IN2        20

// LED outputs
//#define LED_R      17
//#define LED_Y      19

// VBUS input
#define VBUS       1

// VBAT input
#define VBAT       0

// network credentials for AP
const char* ssid = "Dapol Class 08/09";
const char* password = "";

// motor defines
#define DEAD_BAND 120 //minimum power to move motor (used in power mapping)
#define MAX_POWER 254 // desired max power (used in power mapping)
#define INITIAL_NUDGE 120 // first cycle 'nudge' after stop
#define NUDGE_DECAY   1  // 'nudge' decay rate per cycle
// PWM init
#define PWM_BITS  8
#define PWM_FREQ  10000

#define BUFLEN 30

// Set up web server, and register it with EmbAJAX
EmbAJAXOutputDriverWebServerClass server(80);
EmbAJAXOutputDriver driver(&server);

int lastLedTime = 0;
int ledState = 0;

int motor_power = 0;
int old_speed = 0;
int nudge = 0;

// Define the main elements of interest as variables, so we can access to them later in our sketch.
const char* stopstarts[] = {"Stop        ", "Run"};
EmbAJAXRadioGroup<2> stopstart("StSt", stopstarts, 0);

const char* modes[] = {"Reverse    ", "Forward"};
EmbAJAXRadioGroup<2> mode("mode", modes, 1);

EmbAJAXSlider speed("spd", 0, 100, 0);   // slider, from 0 to 100, initial value 0
EmbAJAXMutableSpan speedvalue("speedvalue");
char speedvalue_buf[BUFLEN];

EmbAJAXMutableSpan batteryVolts("batteryVolts");
char batteryVolts_buf[BUFLEN];

const char* charging[] = {" "};
EmbAJAXRadioGroup<1> chargingState(" ", charging, 0);


// Define a page (named "page") with our elements of interest, above, interspersed by some uninteresting
// static HTML. Note: MAKE_EmbAJAXPage is just a convenience macro around the EmbAJAXPage###>-class.
MAKE_EmbAJAXPage(page, "Dapol Class 08/09", "",
                 new EmbAJAXStatic("<h1>Dapol Class 08/09</h1><p>"),
                 &stopstart,
                 new EmbAJAXStatic("</p>"),
                 &mode,
                 new EmbAJAXStatic("</p><p>Power       "),
                 &speedvalue,
                 new EmbAJAXStatic("%</p>"),
                 new EmbAJAXStatic("</p><p>0"),
                 &speed,
                 new EmbAJAXStatic("<i>100</i></p>"),
                 new EmbAJAXStatic("</p><p>Battery Volts       "),
                 &batteryVolts,
                 new EmbAJAXStatic("V</p>"),
                 new EmbAJAXStatic("</p><p>Charging      "),
                 &chargingState

                )

void handlePage() {
  if (server.method() == HTTP_POST) { // AJAX request
    page.handleRequest(updateUI);
  } else {  // Page load
    page.print();
  }
}

void setup() {

  if (DEBUG) {
      Serial.begin(115200);
      Serial.println("\nstartup...\n");
  }

  // WIFI setup as an access point. 
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig (IPAddress (192, 168, 4, 1), IPAddress (0, 0, 0, 0), IPAddress (255, 255, 255, 0));
  WiFi.softAP(ssid , password); 

  // Tell the server to serve our EmbAJAX test page on root
  server.on("/", handlePage);
  server.begin();

// Set motor connections and as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
// set up PWM
  ledcAttach(IN1, PWM_FREQ, PWM_BITS);
  ledcAttach(IN2, PWM_FREQ, PWM_BITS);
 
// Stop motor (Hi-Z)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

// set up on board 'heartbeat' LED
  pinMode(USER_LED, OUTPUT);
  ledcAttach(USER_LED, PWM_FREQ, PWM_BITS);  

  // set up loco LEDs
/*
  pinMode(LED_Y,    OUTPUT);
  pinMode(LED_R,    OUTPUT);
  ledcAttach(LED_Y,    PWM_FREQ, PWM_BITS);  
  ledcAttach(LED_R,    PWM_FREQ, PWM_BITS);
  ledcWrite(LED_Y, 255);
  ledcWrite(LED_R, 255);
*/
  // 'charge' detection is by sensing VBUS 
  pinMode(VBUS, INPUT);

}
char *ftoa(double f, char *a) 
{
  // Convert float to ascii
  char *ret = a;
  long heiltal = (long)f;
  itoa(heiltal, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  int desimal = abs((int)((f - heiltal) * 100));  // int is enough for 2 digits
  if (desimal< 10)  //are there leading zeros?
    { *a='0'; a++; }
  itoa(desimal, a, 10);
  return ret;
}

void updateUI() {
  speed.setEnabled(stopstart.selectedOption() == 1);//Speed control disabled when STOPPED.
  mode.setEnabled(speed.intValue() == 0); //Direction control disabled whilst moving.
  speedvalue.setValue(itoa(speed.intValue(), speedvalue_buf, 10));
  batteryVolts.setValue(ftoa(float((analogReadMilliVolts(VBAT) * 2))/1000, batteryVolts_buf));
  if (digitalRead(VBUS)) {
  chargingState.setEnabled(true);
  } else {
  chargingState.setEnabled(false);
  }

}

void loop() {

// it's alive! LED
  if ((millis() - lastLedTime) > LED_TIMEOUT) {
 
    lastLedTime = millis();

    if (ledState == 0) {

      ledcWrite(USER_LED, 255);
      ledState++;

    } else { // heartbeat LED 'on' is used to trigger an update of the UI

      ledcWrite(USER_LED, 0);
      ledState--;
      
      updateUI();

      if (DEBUG) {
        Serial.print("speed_control = ");
        Serial.print(speed.intValue());
        Serial.print(",     motor power = ");
        Serial.print(motor_power);
        Serial.print(",     run/stop = ");
        Serial.print(stopstart.selectedOption());
        Serial.print(",     direction = ");
        Serial.println(mode.selectedOption());
      } 

    }

  }

 // handle network events
  server.handleClient();

// motor control
  motor_power = map(speed.intValue(), 0, 100, DEAD_BAND, MAX_POWER);

  // set motor PWM channels according to direction and stop/start
  if (stopstart.selectedOption() == 0) { //stop
    ledcWrite(IN1, 0);
    ledcWrite(IN2, 0);
    
    speedvalue.setValue("0"); // update UI
    speed.setValue(0);

  }
  else if (mode.selectedOption() == 1) { // forward
  
    if (speed.intValue() == 0) { //force off if speed is 0 to reduce idle power consumption
      ledcWrite(IN1, 0);
      ledcWrite(IN2, 0);

    } else {

      if ((old_speed == 0) && (speed.intValue() != 0)) {  // 'nudge' motor on start
        nudge = INITIAL_NUDGE; 
      }
      motor_power += nudge;
      motor_power = (motor_power < 0) ? 0 : motor_power; //clamp motor_power
      motor_power = (motor_power > 254) ? 254 : motor_power;

      ledcWrite(IN1, 0);
      ledcWrite(IN2, motor_power);

      nudge -= NUDGE_DECAY; 
      nudge = (nudge < 0) ? 0 : nudge;
    }
    
    old_speed = speed.intValue();

  } else if (mode.selectedOption() == 0) { // reverse
  
    if (speed.intValue() == 0) { //force off if speed is 0 to reduce idle power consumption
      ledcWrite(IN1, 0);
      ledcWrite(IN2, 0);

    } else {

      if ((old_speed == 0) && (speed.intValue() != 0)) {  // 'nudge' motor on start
        nudge = INITIAL_NUDGE; 
      }
      motor_power += nudge;
      motor_power = (motor_power < 0) ? 0 : motor_power; //clamp motor_power
      motor_power = (motor_power > 254) ? 254 : motor_power;

      ledcWrite(IN2, 0);
      ledcWrite(IN1, motor_power);

      nudge -= NUDGE_DECAY; 
      nudge = (nudge < 0) ? 0 : nudge;
    }
    
    old_speed = speed.intValue();

  }

}
