//
// Simple POC for direct control
// based on the 'blinky' EmbAJAX library example

// v1 - version for the Seed Studio XIAO ESP32C6based hardware 

#include <Arduino.h>
#include <WiFi.h>
#include <EmbAJAX.h>
 
#define DEBUG 0 // 0 = no debug

// hardware defines
// LEDs
#define USER_LED          15   // GPIO pin for heartbeat LED
#define LED_TIMEOUT       250  //Heartbeat LED cycle time in ms
#define FLICKER_TIMEOUT   66  //Firebox LED cycle time in ms
// H-bridge pins
#define IN1        18
#define IN2        20
// Firebox LED outputs
#define LED_R      17
#define LED_Y      19
// VBUS input
#define VBUS       1
// VBAT input
#define VBAT       0

// network credentials for AP
const char* ssid = "Loco Controller";
const char* password = "";

// motor defines
#define DEAD_BAND 100
// PWM init
#define PWM_BITS  8
#define PWM_FREQ  10000

#define BUFLEN 30

// Set up web server, and register it with EmbAJAX
EmbAJAXOutputDriverWebServerClass server(80);
EmbAJAXOutputDriver driver(&server);

int lastFlickerTime = 0;
int lastLedTime = 0;
int ledState = 0;

int power = 0;
int motor_power = 0;

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
MAKE_EmbAJAXPage(page, "Simple Control", "",
                 new EmbAJAXStatic("<h1>Simple Control</h1><p>"),
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

  // set up firebox LEDs
  pinMode(LED_Y,    OUTPUT);
  pinMode(LED_R,    OUTPUT);
  ledcAttach(LED_Y,    PWM_FREQ, PWM_BITS);  
  ledcAttach(LED_R,    PWM_FREQ, PWM_BITS);

// firebox LEDs are initially off (1 = off)
  ledcWrite(LED_Y, 255);
  ledcWrite(LED_R, 255);

  // 'charge' detection is by sensing VBUS 
  pinMode(VBUS, INPUT);

}
char *ftoa(double f, char *a) 
{
  // Convert float to ascii!
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
      
      }
  }

    if ((millis() - lastFlickerTime) > FLICKER_TIMEOUT) {
 
    lastFlickerTime = millis();

// FIXME - this needs to be tied to actual charging rather than bus power present
  if (digitalRead(VBUS)) {             // only illuminate firebox if charging
    ledcWrite(LED_R, 255-(random(180)+75));
    ledcWrite(LED_Y, 255-(random(90)+65));
   } else {                            //LEDs off
    ledcWrite(LED_Y, 255);
    ledcWrite(LED_R, 255);
   }

} 

 // handle network events
  server.handleClient();

// motor control
  power = speed.intValue() * 2;
  motor_power = power + DEAD_BAND;

  if (motor_power >= 250) { // clamp to max
    motor_power = 250;
  }
  if (motor_power <= (DEAD_BAND)) { // turn off if at or below DEAD_BAND
    motor_power = 0;
  }

  // set motor PWM channels according to direction and stop/start
  if (stopstart.selectedOption() == 0) { //stop
    ledcWrite(IN1, 0);
    ledcWrite(IN2, 0);
    
    speedvalue.setValue("0");
    speed.setValue(0);

  }
  else if (mode.selectedOption() == 1) { // forward*/
    ledcWrite(IN1, 0);
    ledcWrite(IN2, motor_power);

  }
  else if (mode.selectedOption() == 0) { // reverse
    ledcWrite(IN2, 0);
    ledcWrite(IN1, motor_power);

  }


  if (DEBUG) {
    Serial.print("power = ");
    Serial.print(power);
    Serial.print("     motor power = ");
    Serial.println(motor_power);
    Serial.print("gearbox = ");
    Serial.print("run/stop = ");
    Serial.println(stopstart.selectedOption());
    Serial.print("direction = ");
    Serial.println(mode.selectedOption());
  } 

}
