//
// Simple POC for control of locomotive
//  based on WiFi_loc_PRM
//  by protofunk - Pocket Railway Museum
// 

// v1 - this version for the Seed Studio XIAO ESP32C6 based hardware

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
 
#define N20   1 // build for N20 motor

// hardware defines
// on board LED
#define USER_LED          15  // GPIO pin for LED
#define LED_TIME          2000 //LED cycle time in event loop cycles
#define FLICKER_TIMEOUT   40 //LED cycle time in event loop cycles
// H-bridge pins
#define IN1        18
#define IN2        20
// Firebox LEDs
#define LED_Y      17
#define LED_R      19
// VBUS 
#define VBUS        1
// VBAT 
#define VBAT        0
// expansion connector GPIO
#define EXT_GPIO   16

// network credentials for AP
const char* ssid = "PRM LocoController";
const char* password = "";

#ifdef N20
// motor defines
#define DEAD_BAND 120
// PWM init
#define PWM_BITS 8
#define PWM_FREQ 10000
// physics defines
#define FRICTION 1
#endif

#define DEBUG 1 // 0 = no debug

int power        = 0; 
int motor_power  = 0; 
int led_pwm      = 0;

bool Stopped = true;
bool Forward = false;
bool Reverse = false;

int led_on = 0;
int flicker_time = 0;

// Create WebServer object on port 80
WebServer server(80);

void handleRoot() {
  server.send(200, "text/html", "<h1>You are connected</h1>");
}

void handle_OnConnect() {

  Serial.println("GPIO7 Status: OFF | GPIO6 Status: OFF");
  server.send(200, "text/plain", "connected");
}

void handle_forward5() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 13;
  server.send(200, "text/plain", "Forward13"); 
}

void handle_forward10() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 16;
  server.send(200, "text/plain", "Forward16"); 
}

void handle_forward15() {
  Stopped = false;
  Forward = true;
  Reverse = false;
    power = 18;
  server.send(200, "text/plain", "Forward18"); 
}
void handle_forward20() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 22;
  server.send(200, "text/plain", "Forward22"); 
}
void handle_forward25() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 27;
  server.send(200, "text/plain", "Forward25"); 
}
void handle_forward30() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 30;
  server.send(200, "text/plain", "Forward30"); 
}
void handle_forward35() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 35;
  server.send(200, "text/plain", "Forward35"); 
}
void handle_forward40() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 40;
  server.send(200, "text/plain", "Forward40"); 
}
void handle_forward45() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 45;
  server.send(200, "text/plain", "Forward45"); 
}

void handle_forward50() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 50;
  server.send(200, "text/plain", "Forward50"); 
}
void handle_forward55() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 55;
  server.send(200, "text/plain", "Forward55"); 
}
void handle_forward60() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 60;
  server.send(200, "text/plain", "Forward60"); 
}
void handle_forward65() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 65;
  server.send(200, "text/plain", "Forward65"); 
}
void handle_forward70() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 70;
  server.send(200, "text/plain", "Forward70"); 
}
void handle_forward75() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 75;
  server.send(200, "text/plain", "Forward75"); 
}
void handle_forward80() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 80;
  server.send(200, "text/plain", "Forward80"); 
}
void handle_forward85() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 85;
  server.send(200, "text/plain", "Forward85"); 
}
void handle_forward90() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 90;
  server.send(200, "text/plain", "Forward90"); 
}
void handle_forward95() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 95;
  server.send(200, "text/plain", "Forward95"); 
}

void handle_forward100() {
  Stopped = false;
  Forward = true;
  Reverse = false;
        power = 100;
  server.send(200, "text/plain", "Forward100"); 
}

//REVERSE

void handle_reverse5() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 13;
  server.send(200, "text/plain", "Reverse13"); 
}
void handle_reverse10() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 16;
  server.send(200, "text/plain", "Reverse16"); 
}
void handle_reverse15() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 18;
  server.send(200, "text/plain", "Reverse18"); 
}
void handle_reverse20() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 22;
  server.send(200, "text/plain", "Reverse22"); 
}
void handle_reverse25() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 25;
  server.send(200, "text/plain", "Reverse25"); 
}
void handle_reverse30() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 30;
  server.send(200, "text/plain", "Reverse30"); 
}
void handle_reverse35() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 35;
  server.send(200, "text/plain", "Reverse35"); 
}
void handle_reverse40() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 40;
  server.send(200, "text/plain", "Reverse40"); 
}
void handle_reverse45() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 45;
  server.send(200, "text/plain", "Reverse45"); 
}
void handle_reverse50() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 50;
  server.send(200, "text/plain", "Reverse50"); 
}
void handle_reverse55() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 55;
  server.send(200, "text/plain", "Reverse55"); 
}
void handle_reverse60() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 60;
  server.send(200, "text/plain", "Reverse60"); 
}
void handle_reverse65() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 65;
  server.send(200, "text/plain", "Reverse65"); 
}
void handle_reverse70() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 70;
  server.send(200, "text/plain", "Reverse70"); 
}
void handle_reverse75() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 75;
  server.send(200, "text/plain", "Reverse75"); 
}
void handle_reverse80() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 80;
  server.send(200, "text/plain", "Reverse80"); 
}
void handle_reverse85() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 85;
  server.send(200, "text/plain", "Reverse85"); 
}
void handle_reverse90() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 90;
  server.send(200, "text/plain", "Reverse90"); 
}
void handle_reverse95() {
  Stopped = false;
  Forward = false;
  Reverse = true;
  power = 95;
  server.send(200, "text/plain", "Reverse95"); 
}
void handle_reverse100() {
  Stopped = false;
  Forward = false;
  Reverse = true;
 power = 100;
  server.send(200, "text/plain", "Reverse100"); 
}

void handle_stopping() {
   Stopped = true;
  Forward = false;
  Reverse = false;
  power = 0;
   server.send(200, "text/plain", "Stopping");
}

void handle_lighton() {
  
  Serial.println("lights on");
    server.send(200, "text/plain", "Lights are on");
   led_pwm = 100;
}

void handle_lightoff() {
  
  Serial.println("lightsoff");
    server.send(200, "text/plain", "Lights are off");
        led_pwm = 0;
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

void setup() {
  Serial.begin(115200);
  Serial.println("startup...");

  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.on("/", handleRoot);
  server.begin();
  Serial.println("HTTP server started");

 // Set motor connections as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

// set up PWM
  ledcAttach(IN1, PWM_FREQ, PWM_BITS);
  ledcAttach(IN2, PWM_FREQ, PWM_BITS);
 
// Stop motor (Hi-Z)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // set up LEDs
  pinMode(USER_LED, OUTPUT);
  pinMode(LED_Y,    OUTPUT);
  pinMode(LED_R,    OUTPUT);
  ledcAttach(USER_LED, PWM_FREQ, PWM_BITS);  
  ledcAttach(LED_Y,    PWM_FREQ, PWM_BITS);  
  ledcAttach(LED_R,    PWM_FREQ, PWM_BITS);

  // firebox LEDs are initially off (1 = off)
  ledcWrite(LED_Y, 255);
  ledcWrite(LED_R, 255);

  // charge detection is by sensing VBUS
  pinMode(VBUS, INPUT);

  server.on("/", handle_OnConnect);
  server.on("/lightson", handle_lighton);
 server.on("/lightsoff", handle_lightoff);
  server.on("/forward/5", handle_forward5);
    server.on("/forward/10", handle_forward10);
      server.on("/forward/15", handle_forward15);
        server.on("/forward/20", handle_forward20);
          server.on("/forward/25", handle_forward25);
            server.on("/forward/30", handle_forward30);
              server.on("/forward/35", handle_forward35);
                server.on("/forward/40", handle_forward40);
                  server.on("/forward/45", handle_forward45);
                    server.on("/forward/50", handle_forward50);
                      server.on("/forward/55", handle_forward55);
                        server.on("/forward/60", handle_forward60);
                          server.on("/forward/65", handle_forward65);
                            server.on("/forward/70", handle_forward70);
                              server.on("/forward/75", handle_forward75);
                                server.on("/forward/80", handle_forward80);
                                  server.on("/forward/85", handle_forward85);
                                    server.on("/forward/90", handle_forward90);
                                      server.on("/forward/95", handle_forward95);
                                        server.on("/forward/100", handle_forward100);
  server.on("/reverse/5", handle_reverse5);
    server.on("/reverse/10", handle_reverse10);
      server.on("/reverse/15", handle_reverse15);
        server.on("/reverse/20", handle_reverse20);
          server.on("/reverse/25", handle_reverse25);
            server.on("/reverse/30", handle_reverse30);
              server.on("/reverse/35", handle_reverse35);
                server.on("/reverse/40", handle_reverse40);
                  server.on("/reverse/45", handle_reverse45);
                    server.on("/reverse/50", handle_reverse50);
                      server.on("/reverse/55", handle_reverse55);
                        server.on("/reverse/60", handle_reverse60);
                          server.on("/reverse/65", handle_reverse65);
                            server.on("/reverse/70", handle_reverse70);
                              server.on("/reverse/75", handle_reverse75);
                                server.on("/reverse/80", handle_reverse80);
                                  server.on("/reverse/85", handle_reverse85);
                                    server.on("/reverse/90", handle_reverse90);
                                      server.on("/reverse/95", handle_reverse95);
                                        server.on("/reverse/100", handle_reverse100);

  server.on("/stopped", handle_stopping);
  server.onNotFound(handle_NotFound);

}

void loop() {

// it's alive! LED
  led_on++;
  if (led_on == LED_TIME/2) {
    ledcWrite(USER_LED, 255);
  } 
   if (led_on == LED_TIME) {
    ledcWrite(USER_LED, 0);
    led_on = 0;
    
    Serial.print("VBAT value: ");
    Serial.println(analogRead(VBAT));

  }

  // firebox LEDs
  flicker_time++;
   if (flicker_time == FLICKER_TIMEOUT) {

  if (digitalRead(VBUS)) {             // only illuminate firebox if charging
    ledcWrite(LED_Y, random(245)+10);
    ledcWrite(LED_R, random(245)+10);
   } else {                            //LEDs off
    ledcWrite(LED_Y, 255);
    ledcWrite(LED_R, 255);
   }

    flicker_time = 0;
  } 

  server.handleClient();

 if (DEBUG) {

  Serial.print("\power = ");
  Serial.print(power);
  Serial.print("    motor_power = ");
  Serial.println(motor_power);
/*  Serial.print("Forward = ");
  Serial.println(Forward);
  Serial.print("Reverse = ");
  Serial.println(Reverse);
  Serial.print("Stopped = ");
  Serial.println(Stopped);*/
  
  }
     
  motor_power = power + DEAD_BAND;

  if (motor_power >= 250) { // clamp to max
    motor_power = 250;
  }
  if (motor_power <= (DEAD_BAND)) { // turn off if at DEAD_BAND
    motor_power = 0;
  }

// set PWM according to direction
  if (Forward) {
    ledcWrite(IN1, 0);
    ledcWrite(IN2, motor_power);
  } else if (Reverse) {
    ledcWrite(IN2, 0);
    ledcWrite(IN1, motor_power);
  } else { // stopped
    ledcWrite(IN2, 0);
    ledcWrite(IN1, 0);  
  }

}