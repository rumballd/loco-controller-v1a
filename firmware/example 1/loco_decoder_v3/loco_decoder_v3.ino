//
// Simple POC for control of locomotive
//  based on esp32-web-server-websocket-sliders example from
// Rui Santos & Sara Santos - Random Nerd Tutorials
// 

// v3 - version for the Seed Studio XIAO ESP32C6based hardware

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "LittleFS.h"
#include <Arduino_JSON.h>

// hardware defines
// LEDs
#define USER_LED          15   // GPIO pin for heartbeat LED
#define LED_TIMEOUT       250  //Heartbeat LED cycle time in ms
#define FLICKER_TIMEOUT   66  //Firebox LED cycle time in ms
// H-bridge control outputs
#define IN1        18
#define IN2        20
// Firebox LED outputs
#define LED_R      17
#define LED_Y      19
// VBUS input (digital)
#define VBUS        1
// VBAT input (analogue)
#define VBAT        0
// expansion connector GPIO
#define EXT_GPIO   16
 
// build configs
//#define DAPOL_B4       // build for Dapol B4
#define DAPOL_58XX     // build for Dapol 58XX
//#define PRM_14XX       // build for PRM 14XX
//#define PRM_PANNIER    // build for PRM 'Pannier'

#ifdef DAPOL_B4
// network credentials for AP
const char* ssid = "Dapol B4";
const char* password = "";
// filepath to root html in FLASH filesystem
const char* index_file = "/dapol_b4.html";
#define FIREBOX_LEDS 1   // red and yellow LEDs in firebox
// motor defines
#define DAPOL 1       // buil for Dapol 0 gauge 5 pole motor
#define DEAD_BAND 130
// PWM init
#define PWM_BITS 8
#define PWM_FREQ 10000
// physics defines
#define FRICTION 1
#endif

#ifdef DAPOL_58XX
// network credentials for AP
const char* ssid = "LocoController";
const char* password = "";
// filepath to root html in FLASH filesystem
const char* index_file = "/dapol_58xx.html";
#define FIREBOX_LEDS 1   // red and yellow LEDs in firebox
// motor defines
#define DAPOL 1       // build for Dapol 0 gauge 5 pole motor
#define DEAD_BAND 120
// PWM init
#define PWM_BITS 8
#define PWM_FREQ 10000
// physics defines
#define FRICTION 1
#endif

#ifdef PRM_14XX
// network credentials for AP
const char* ssid = "PRM 14XX";
const char* password = "";
// filepath to root html in FLASH filesystem
const char* index_file = "/prm_14xx.html";
#define FIREBOX_LEDS 0   // no firebox LEDS
// motor defines
#define N20         1  // build for N20 motor and gearbox
#define DEAD_BAND 100
// PWM init
#define PWM_BITS 8
#define PWM_FREQ 5000
// physics defines
#define FRICTION 1
#endif

#ifdef PRM_PANNIER
// network credentials for AP
const char* ssid = "PRM Pannier";
const char* password = "";
// filepath to root html in FLASH filesystem
const char* index_file = "/prm_pannier.html";
#define FIREBOX_LEDS 0   // no firebox LEDS
// motor defines
#define N20         1  // build for N20 motor and gearbox
#define DEAD_BAND 100
// PWM init
#define PWM_BITS 8
#define PWM_FREQ 5000
// physics defines
#define FRICTION 1
#endif

#define DEBUG        1 // 0 = no debug
#define ACCESS_POINT 1 // 1 = use internal access point, 0 = use external network

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
// Create a WebSocket object
AsyncWebSocket ws("/ws");

String message = "";
String sliderValue1 = "0";
String sliderValue2 = "0";
String sliderValue3 = "0";
String chargeState = " ";

int power = 0;
int motor_power = 0;
int throttle = 0;
int reverser = 0;
int brake = 100;

int lastFlickerTime = 0;
int lastLedTime = 0;
int ledState = 0;
int vbat = 0;

//Json Variable to Hold values
JSONVar sliderValues;

//Get Slider Values
String getSliderValues(){
  sliderValues["sliderValue1"] = sliderValue1;
  sliderValues["sliderValue2"] = sliderValue2;
  sliderValues["sliderValue3"] = sliderValue3;
  sliderValues["power"] = String(power);

  if (digitalRead(VBUS)) {             // add charging state to VBAT reading
  chargeState = "(charging)  "+String(float(vbat)/1000);
  sliderValues["batteryvolts"] = chargeState;
  } else {
    sliderValues["batteryvolts"] = String(float(vbat)/1000);
  }
    
  String jsonString = JSON.stringify(sliderValues);

  if (DEBUG) {
    Serial.println(jsonString);
  }
  return jsonString;}

// Initialize LittleFS
void initFS() {
  if (!LittleFS.begin()) {
    Serial.println("An error has occurred while mounting LittleFS");
  }
  else{
   Serial.println("LittleFS mounted successfully");
  }
}

// Initialize WiFi
void initWiFi() {

  if (ACCESS_POINT) { // use internal access point
    
    Serial.print("Setting AP (Access Point)…");
    WiFi.softAP(ssid /*, password*/);   // password parameter removed for open access

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    
  } else { // use external network
      
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi ..");

    while (WiFi.status() != WL_CONNECTED) {
      Serial.print('.');
      delay(1000);
      }

    Serial.println(WiFi.localIP());
  }
}


void notifyClients(String sliderValues) {
  ws.textAll(sliderValues);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    message = (char*)data;

    if (DEBUG) {
      Serial.print("message = ");     Serial.println(message);
    }

    if (message.indexOf("1s") >= 0) {
      sliderValue1 = message.substring(2);
      notifyClients(getSliderValues());
    }

    if (message.indexOf("2s") >= 0) {
      sliderValue2 = message.substring(2);
      notifyClients(getSliderValues());
    } 

    if (message.indexOf("3s") >= 0) {
      sliderValue3 = message.substring(2);
      notifyClients(getSliderValues());
    }
    if (strcmp((char*)data, "getValues") == 0) {
      notifyClients(getSliderValues());
    }
  }
}
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void setup() {
  Serial.begin(115200);
  Serial.println("startup...");

  initFS();
  initWiFi();

 // Set motor connections as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

// set up PWM
  ledcAttach(IN1, PWM_FREQ, PWM_BITS);
  ledcAttach(IN2, PWM_FREQ, PWM_BITS);
 
// Stop motor (Hi-Z)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  initWebSocket();
  
// Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, index_file, "text/html");
  });
  
  server.serveStatic("/", LittleFS, "/");

// Start server
  server.begin();

// set up on board 'heartbeat' LED
  pinMode(USER_LED, OUTPUT);
  ledcAttach(USER_LED, PWM_FREQ, PWM_BITS);  

#ifdef FIREBOX_LEDS
// set up firebox LEDs
  pinMode(LED_Y,    OUTPUT);
  pinMode(LED_R,    OUTPUT);
  ledcAttach(LED_Y,    PWM_FREQ, PWM_BITS);  
  ledcAttach(LED_R,    PWM_FREQ, PWM_BITS);

// firebox LEDs are initially off (1 = off)
  ledcWrite(LED_Y, 255);
  ledcWrite(LED_R, 255);
#endif

// 'charge' detection is by sensing VBUS 
  pinMode(VBUS, INPUT);

}

void loop() {

// it's alive! LED
  if ((millis() - lastLedTime) > LED_TIMEOUT) {
 
    lastLedTime = millis();

    if (ledState == 0) {

      ledcWrite(USER_LED, 255);
      ledState++;

    } else { // heartbeat LED 'on' is used to trigger a number of 'real time' processes

      ledcWrite(USER_LED, 0);
      ledState--;

  // read battery voltage
    vbat = analogReadMilliVolts(VBAT) * 2; // VBAT input is via a 2:1 resistive divider
    notifyClients(getSliderValues());
    
  // place power calculation here to create illusion of momentum by discretly iterating over the loop timesteps

    throttle = map(sliderValue1.toInt(), 0, 100, 0, 100);
    reverser = map(sliderValue2.toInt(), -50, 50, -95, 95);      
    brake = map(sliderValue3.toInt(), 0, 100, 0, 100);

    power = power +  throttle/8 - FRICTION - brake/8;

      if (power <= 0) { // clamp to min
        power = 0;
      }

      if (power >= abs(reverser)) { // clamp to reverser magnitude
        power = abs(reverser);
      }
      
      motor_power = power + DEAD_BAND;

      if (motor_power >= 250) { // clamp to max
        motor_power = 250;
      }
      if (motor_power <= (DEAD_BAND)) { // turn off if at DEAD_BAND
        motor_power = 0;
      }

      if (0) {
        Serial.print("VBAT (mV)) = ");     Serial.println(vbat);      
        Serial.print("throttle = ");       Serial.print(throttle);
        Serial.print("   reverser = ");    Serial.print(reverser);
        Serial.print("   brake = ");       Serial.print(brake);
        Serial.print("   power = ");       Serial.print(power); 
        Serial.print("   motor power = "); Serial.println(motor_power);
        Serial.println(); 
      }
  }
  }

#ifdef FIREBOX_LEDS
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
# endif

// set PWM channel according to direction
  if (reverser >= 0) {
    ledcWrite(IN1, 0);
    ledcWrite(IN2, motor_power);
  } else {
    ledcWrite(IN2, 0);
    ledcWrite(IN1, motor_power);
  }

  ws.cleanupClients();
 
}