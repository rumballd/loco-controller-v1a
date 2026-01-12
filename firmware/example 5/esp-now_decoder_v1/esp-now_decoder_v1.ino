//
// Simple POC for control of locomotive usung the ESP-NOW protocol
// based on target example by VolosR 
// github.com/VolosR/Knob18Meters/tree/main/KnobRGBControl
// 

// v1 - initial version

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

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

#include <loco_profiles.h> // build configs and specific loco profiles for motor control etc

#define DEBUG        0 // 0 = no debug

int8_t rxPower[4] = {0,0,0,0};   // (power for the selected loco in the UI), loco1, loco2, loco3
int8_t txStatus[4] = {0,0,0,0};   // LOCO_ID, spare, spare, vbat as % plus charging state

// motor
int power = 0;
int motor_power = 0;
int old_speed = 0;
float nudge = 0.0;

//LEDs
int lastFlickerTime = 0;
int lastLedTime = 0;
int ledState = 0;

int vbat = 0;

// handheld control MAC address
extern uint8_t targetAdress[] = {0xD0, 0xCF, 0x13, 0x1E, 0x12, 0x50}; // control unit address

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


// callback for ESP-NOW packet Rx
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&rxPower, incomingData, sizeof(rxPower)); // copy the 4 bytes we use
}

void setup() {
  Serial.begin(115200);
  Serial.println("startup");

   WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent)); // cast to fix ESP_NOW API change 
  memcpy(peerInfo.peer_addr, targetAdress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
 // Set motor connections as outputs
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

    } else { 

      ledcWrite(USER_LED, 0);
      ledState--;

  // read battery voltage (average over 16 readings)
      vbat = 0;
      for (int i=0; i<16; i++) {
        vbat += (analogReadMilliVolts(VBAT) * 2); // VBAT input is via a 2:1 resistive divider
      }
      vbat /= 16;

// build status packet      
      txStatus[3] = map(vbat, 3000, 4200, 0, 100); //3.00V->4.2V maps to 0->100%
      if (digitalRead(VBUS)) {
        txStatus[3] |= 0x80; // set msb if charging
      }
      txStatus[0] = LOCO_ID;

// send the status packet
esp_err_t result = esp_now_send(targetAdress, (uint8_t *) &txStatus, sizeof(txStatus));

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

// calculate motor power and set PWM channel according to direction
  power = (int)abs(rxPower[LOCO_ID]); // note cast from CHAR to INT

  if (abs(rxPower[LOCO_ID]) == rxPower[LOCO_ID]) { // -ve values are reverse
  
    motor_power = map(power, 0, 100, FWD_DEAD_BAND, MAX_POWER);
  
    if ((old_speed == 0) && power != 0) {  // 'nudge' motor on start
      nudge = INITIAL_NUDGE;
    }
    motor_power += int(nudge);
    motor_power = (motor_power < 0) ? 0 : motor_power; //clamp motor_power
    motor_power = (motor_power > MAX_POWER) ? MAX_POWER : motor_power;
    
    nudge -= NUDGE_DECAY;
    nudge = (nudge < 0.0) ? 0.0 : nudge;

    motor_power = (motor_power <= FWD_DEAD_BAND) ? 0 : motor_power; // force a stop at DEAD_BAND value

    ledcWrite(IN1, 0);
    ledcWrite(IN2, motor_power);

  } else {
  
    motor_power = map(power, 0, 100, REV_DEAD_BAND, MAX_POWER);
  
    if ((old_speed == 0) && power != 0) {  // 'nudge' motor on start
      nudge = INITIAL_NUDGE;
    }
        motor_power += int(nudge);
    motor_power = (motor_power < 0) ? 0 : motor_power; //clamp motor_power
    motor_power = (motor_power > MAX_POWER) ? MAX_POWER : motor_power;
    
    nudge -= NUDGE_DECAY;
    nudge = (nudge < 0.0) ? 0.0 : nudge;

    motor_power = (motor_power <= REV_DEAD_BAND) ? 0 : motor_power; // force a stop at DEAD_BAND value

    ledcWrite(IN2, 0);
    ledcWrite(IN1, motor_power);

  }
  
  old_speed = power;

  if (DEBUG) {
  Serial.print("power = ");
  Serial.print(power, DEC);
  Serial.print("   motor_power = ");
  Serial.print(motor_power, DEC);
  Serial.print("   nudge = ");
  Serial.print(nudge, DEC);
  Serial.print("   old_speed = ");
  Serial.println(old_speed, DEC);
  }

//delay(1);

}
