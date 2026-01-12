//
// Simple POC for a graphical based handheld control usung the ESP-NOW protocol
// based on the example by VolosR 
// github.com/VolosR/Knob18Meters/tree/main/KnobRGBControl
// This firmware runs on the Waveshare ESP32-S3 1.8inch Knob Display Development Board
// https://www.waveshare.com/esp32-s3-knob-touch-lcd-1.8.htm
// 

// v1 - initial version
#include "lcd_bsp.h"
#include "cst816.h"
#include "lcd_bl_pwm_bsp.h"
#include "lcd_config.h"
#include "ui.h"
#include "bidi_switch_knob.h"
#include "ui_events.h"

#include <esp_now.h>
#include <WiFi.h>

// I2C
#define I2C_SCL 12
#define I2C_SDA 11

// haptic effects
#define DRV2605_REG_LIBRARY 0x03
#define CLICK         1
#define DOUBLE_CLICK 10
#define PULSE        52

#define EXAMPLE_ENCODER_ECA_PIN    8
#define EXAMPLE_ENCODER_ECB_PIN    7

#define SET_BIT(reg,bit) (reg |= ((uint32_t)0x01<<bit))
#define CLEAR_BIT(reg,bit) (reg &= (~((uint32_t)0x01<<bit)))
#define READ_BIT(reg,bit) (((uint32_t)reg>>bit) & 0x01)
#define BIT_EVEN_ALL (0x00ffffff)

// target MAC address
extern uint8_t targetAdress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // broadcast address

esp_now_peer_info_t peerInfo;

static const char *TAG = "encoder";

static lv_obj_t * meter;
lv_meter_indicator_t * needle; 

static lv_obj_t * meter2;
lv_meter_indicator_t * needle2; 

static lv_obj_t * meter3;
lv_meter_indicator_t * needle3; 

static lv_obj_t * meter4;
lv_meter_indicator_t * needle4; 

EventGroupHandle_t knob_even_ = NULL;

static knob_handle_t s_knob = 0;

SemaphoreHandle_t mutex; 

extern int8_t power[4]= {0,0,0,0};   // power for the selected loco, loco1, loco2, loco3
int8_t status[4] = {0,0,0,0};   //  returned status - LOCO_ID, spare, spare, vbat as % plus charging state
int vbat[4]; // batttery percentages

int chosen = 2; // select first loco in UI

extern uint8_t I2C_writr_buff(uint8_t addr,uint8_t reg,uint8_t *buf,uint8_t len);

// play haptic effect from the DRV2605's internal lib
extern void hapticEffect(int effect) {

  // setWaveform(0, effect);  // select effect -> writeRegister(DRV2605_REG_WAVESEQ1 + slot, w);
  // setWaveform(1, 0);       // select end waveform -> writeRegister(DRV2605_REG_WAVESEQ1 + slot, w);
  uint8_t data = effect;
  I2C_writr_buff(0x5A,0x04,&data,1);
  data = 0x00;
  I2C_writr_buff(0x5A,0x05,&data,1);

  // run(); // play the effect -> writeRegister(DRV2605_REG_GO, 1);
  data = 0x01;
  I2C_writr_buff(0x5A,0x0C,&data,1);
  
}
  
// callback for data tx
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback for data rx
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {  
  memcpy(&status, incomingData, sizeof(status));

  vbat[status[0]] = status[3];
}

void lv_example_meter_1(void)
{
     extern lv_obj_t *ui_Screen1;
     meter = lv_meter_create(ui_Screen1);
     lv_obj_add_event_cb(meter, meter_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_center(meter);
    lv_obj_set_size(meter, 204, 204);
    lv_obj_set_pos(meter, -71, 0);
    lv_obj_set_style_bg_opa(meter, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(meter, 0, LV_PART_MAIN);

    // Add a scale first
    lv_meter_scale_t * scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_range(meter, scale, -100, 100, 270, 135); // Set range from -100 to 100
    lv_meter_set_scale_ticks(meter, scale, 31, 3, 10, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter, scale, 6, 5, 15, lv_color_white(), 10);

    lv_meter_indicator_t * indic;

    // Add a blue arc to the start
    indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_meter_set_indicator_start_value(meter, indic, -100);
    lv_meter_set_indicator_end_value(meter, indic, -80);

    // Make the tick lines blue at the start of the scale
    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_BLUE), false, 0);
    lv_meter_set_indicator_start_value(meter, indic, -100);
    lv_meter_set_indicator_end_value(meter, indic, -80);

    // Add a red arc to the end
    indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_indicator_start_value(meter, indic, 80);
    lv_meter_set_indicator_end_value(meter, indic, 100);

    // Make the tick lines red at the end of the scale
    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
    lv_meter_set_indicator_start_value(meter, indic, 80);
    lv_meter_set_indicator_end_value(meter, indic, 100);

    // Add a needle line indicator
    needle = lv_meter_add_needle_line(meter, scale, 4, lv_color_hex(0xFFFFFF), -10); 
  
}

void lv_example_meter_2(void)
{
     extern lv_obj_t *ui_Screen1;
     meter2 = lv_meter_create(ui_Screen1);
     lv_obj_add_event_cb(meter2, meter2_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_center(meter2);
    lv_obj_set_size(meter2, 128, 128);
    lv_obj_set_pos(meter2, 109, 0);
    lv_obj_set_style_bg_opa(meter2, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(meter2, 0, LV_PART_MAIN);

    // Add a scale first
    lv_meter_scale_t * scale = lv_meter_add_scale(meter2);
    lv_meter_set_scale_range(meter2, scale, -100, 100, 270, 135); // Set range from -100 to 100
    lv_meter_set_scale_ticks(meter2, scale, 31, 2, 5, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter2, scale, 6, 3, 10, lv_color_white(), 10);

    lv_meter_indicator_t * indic;

    // Add a blue arc to the start
    indic = lv_meter_add_arc(meter2, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_meter_set_indicator_start_value(meter2, indic, -100);
    lv_meter_set_indicator_end_value(meter2, indic, -80);

    // Make the tick lines blue at the start of the scale
    indic = lv_meter_add_scale_lines(meter2, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_BLUE), false, 0);
    lv_meter_set_indicator_start_value(meter2, indic, -100);
    lv_meter_set_indicator_end_value(meter2, indic, -80);

    // Add a red arc to the end
    indic = lv_meter_add_arc(meter2, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_indicator_start_value(meter2, indic, 80);
    lv_meter_set_indicator_end_value(meter2, indic, 100);

    // Make the tick lines red at the end of the scale
    indic = lv_meter_add_scale_lines(meter2, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
    lv_meter_set_indicator_start_value(meter2, indic, 80);
    lv_meter_set_indicator_end_value(meter2, indic, 100);

    /*Add a needle line indicator*/
    needle2 = lv_meter_add_needle_line(meter2, scale, 3, lv_color_hex(0xFFFFFF), -10); 
}

void lv_example_meter_3(void)
{
     extern lv_obj_t *ui_Screen1;
     meter3 = lv_meter_create(ui_Screen1);
     lv_obj_add_event_cb(meter3, meter3_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_center(meter3);
    lv_obj_set_size(meter3, 128, 128);
    lv_obj_set_pos(meter3, 43, -107);
    lv_obj_set_style_bg_opa(meter3, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(meter3, 0, LV_PART_MAIN);

    // Add a scale first
    lv_meter_scale_t * scale = lv_meter_add_scale(meter3);
    lv_meter_set_scale_range(meter3, scale, -100, 100, 270, 135); // Set range from -1000 to 100
    lv_meter_set_scale_ticks(meter3, scale, 31, 2, 5, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter3, scale, 6, 3, 10, lv_color_white(), 10);

    lv_meter_indicator_t * indic;

    // Add a blue arc to the start
    indic = lv_meter_add_arc(meter3, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_meter_set_indicator_start_value(meter3, indic, -100);
    lv_meter_set_indicator_end_value(meter3, indic, -80);

    // Make the tick lines blue at the start of the scale
    indic = lv_meter_add_scale_lines(meter3, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_BLUE), false, 0);
    lv_meter_set_indicator_start_value(meter3, indic, -100);
    lv_meter_set_indicator_end_value(meter3, indic, -80);

    // Add a red arc to the end
    indic = lv_meter_add_arc(meter3, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_indicator_start_value(meter3, indic, 80);
    lv_meter_set_indicator_end_value(meter3, indic, 100);

    // Make the tick lines red at the end of the scale
    indic = lv_meter_add_scale_lines(meter3, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
    lv_meter_set_indicator_start_value(meter3, indic, 80);
    lv_meter_set_indicator_end_value(meter3, indic, 100);

    /*Add a needle line indicator*/
    needle3 = lv_meter_add_needle_line(meter3, scale, 3, lv_color_hex(0xFFFFFF), -10); 
}

void lv_example_meter_4(void)
{
     extern lv_obj_t *ui_Screen1;
     meter4 = lv_meter_create(ui_Screen1);
     lv_obj_add_event_cb(meter4, meter4_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_center(meter4);
    lv_obj_set_size(meter4, 128, 128);
    lv_obj_set_pos(meter4, 43, 107);
    lv_obj_set_style_bg_opa(meter4, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(meter4, 0, LV_PART_MAIN);

    // Add a scale first
    lv_meter_scale_t * scale = lv_meter_add_scale(meter4);
    lv_meter_set_scale_range(meter4, scale, -100, 100, 270, 135); // Set range from -1000 to 100
    lv_meter_set_scale_ticks(meter4, scale, 31, 2, 5, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter4, scale, 6, 3, 10, lv_color_white(), 10);

    lv_meter_indicator_t * indic;

    // Add a blue arc to the start
    indic = lv_meter_add_arc(meter4, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_meter_set_indicator_start_value(meter4, indic, -100);
    lv_meter_set_indicator_end_value(meter4, indic, -80);

    // Make the tick lines blue at the start of the scale
    indic = lv_meter_add_scale_lines(meter4, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_BLUE), false, 0);
    lv_meter_set_indicator_start_value(meter4, indic, -100);
    lv_meter_set_indicator_end_value(meter4, indic, -80);

    // Add a red arc to the end
    indic = lv_meter_add_arc(meter4, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_indicator_start_value(meter4, indic, 80);
    lv_meter_set_indicator_end_value(meter4, indic, 100);

    // Make the tick lines red at the end of the scale
    indic = lv_meter_add_scale_lines(meter4, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
    lv_meter_set_indicator_start_value(meter4, indic, 80);
    lv_meter_set_indicator_end_value(meter4, indic, 100);

    /*Add a needle line indicator*/
    needle4 = lv_meter_add_needle_line(meter4, scale, 3, lv_color_hex(0xFFFFFF), -10); 
}

static void _knob_left_cb(void *arg, void *data)
{
  uint8_t eventBits_ = 0;
  SET_BIT(eventBits_,0);
  xEventGroupSetBits(knob_even_,eventBits_);
}
static void _knob_right_cb(void *arg, void *data)
{
  uint8_t eventBits_ = 0;
  SET_BIT(eventBits_,1);
  xEventGroupSetBits(knob_even_,eventBits_);
}

void setup()
{
  mutex = xSemaphoreCreateMutex();

  Serial.begin(115200);

  Touch_Init();

  lcd_lvgl_Init();

  lv_example_meter_1();
  lv_example_meter_2();
  lv_example_meter_3();
  lv_example_meter_4();

  set_active_meter(chosen); // select first loco
  chosen = 2;

  lcd_bl_pwm_bsp_init(128); // brightness up to 255

  knob_even_ = xEventGroupCreate();
  // create knob
  knob_config_t cfg = 
  {
    .gpio_encoder_a = EXAMPLE_ENCODER_ECA_PIN,
    .gpio_encoder_b = EXAMPLE_ENCODER_ECB_PIN,
  };
  s_knob = iot_knob_create(&cfg);

  iot_knob_register_cb(s_knob, KNOB_LEFT, _knob_left_cb, NULL);
  iot_knob_register_cb(s_knob, KNOB_RIGHT, _knob_right_cb, NULL);
  
  xTaskCreate(user_encoder_loop_task, "user_encoder_loop_task", 3000, NULL, 2, NULL);
  xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

   WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

// register ESP-NOW callbacks
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
  // haptic init (horrible direct I2C write hack to get round library clash for I2C devices!)  
  // selectLibrary(1); // select library 1 -> writeRegister(DRV2605_REG_LIBRARY, lib);
  uint8_t data = 0x01;
  I2C_writr_buff(0x5A,0x03,&data,1);
  
  // I2C trigger by sending 'go' command, internal trigger 
  // setMode(MODE_INTTRIG); -> writeRegister(DRV2605_REG_MODE, mode);
  data = 0x00;
  I2C_writr_buff(0x5A,0x01,&data,1);

  hapticEffect(PULSE);
  
  }

// update value of selected dial on knob movement and send message packet
static void user_encoder_loop_task(void *arg)
{
  for(;;)
  {
    EventBits_t even = xEventGroupWaitBits(knob_even_,BIT_EVEN_ALL,pdTRUE,pdFALSE,pdMS_TO_TICKS(5000)); //wait for event 
    if(READ_BIT(even,0))
    { 
       if (xSemaphoreTake(mutex, portMAX_DELAY)) { // rotate CCW

       power[0] = (power[0] < -100) ? -100 : power[0] -= 2; // clamp to -100
       power[chosen] = power[0];
       if(power[0] == 0) hapticEffect(CLICK);

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(targetAdress, (uint8_t *) &power, sizeof(power));
      
              xSemaphoreGive(mutex); 
    }
    }
    if(READ_BIT(even,1)) // rotate CW
    {
       if (xSemaphoreTake(mutex, portMAX_DELAY)) { 

       power[0] = (power[0] > 100) ? 100 : power[0] += 2; // clamp to 100
       power[chosen] = power[0];
       if(power[0] == 0) hapticEffect(CLICK);

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(targetAdress, (uint8_t *) &power, sizeof(power));
                xSemaphoreGive(mutex); 
    }
    }
  }
}

// calculate charge state arc colour
uint vbat2Colour(int vbat) {

  uint arcColour = 0xFF0000; // default is red

  if ((vbat & 0x7F) >= 20 ) arcColour = 0xFFFF00; // greater than 20% is yellow
  if ((vbat & 0x7F) >= 50 ) arcColour = 0x00FF00; // greater than 50% is green
  if (vbat & 0x80 ) arcColour = 0xFFFFFF; // if charging is white
 
  return arcColour;
}

static void example_lvgl_port_task(void *arg)
{
   
  for(;;)
  {
   lv_timer_handler(); // wait for timer

    if (xSemaphoreTake(mutex, portMAX_DELAY)) { 
        
        lv_label_set_text(ui_power,String(power[0]).c_str());
        lv_meter_set_indicator_value(meter, needle, power[0]);
        lv_obj_set_style_arc_color(ui_Arc1, lv_color_hex(vbat2Colour(vbat[chosen])), LV_PART_INDICATOR | LV_STATE_DEFAULT);
        lv_arc_set_value(ui_Arc1, vbat[chosen] & 0x7F);

        lv_label_set_text(ui_power1,String(power[1]).c_str());
        lv_meter_set_indicator_value(meter2, needle2, power[1]);
        lv_obj_set_style_arc_color(ui_Arc2, lv_color_hex(vbat2Colour(vbat[1])), LV_PART_INDICATOR | LV_STATE_DEFAULT);
        lv_arc_set_value(ui_Arc2, vbat[1] & 0x7F);

        lv_label_set_text(ui_power3,String(power[2]).c_str());
        lv_meter_set_indicator_value(meter3, needle3, power[2]);
        lv_obj_set_style_arc_color(ui_Arc3, lv_color_hex(vbat2Colour(vbat[2])), LV_PART_INDICATOR | LV_STATE_DEFAULT);
        lv_arc_set_value(ui_Arc3, vbat[2] & 0x7F);

        lv_label_set_text(ui_power2,String(power[3]).c_str());
        lv_meter_set_indicator_value(meter4, needle4, power[3]);
        lv_obj_set_style_arc_color(ui_Arc4, lv_color_hex(vbat2Colour(vbat[3])), LV_PART_INDICATOR | LV_STATE_DEFAULT);
        lv_arc_set_value(ui_Arc4, vbat[3] & 0x7F);


        xSemaphoreGive(mutex); 
        }
     vTaskDelay(pdMS_TO_TICKS(1));  
    }
  
  }

void meter_event_cb(lv_event_t * e) // stop selected loco 
{
    if(lv_event_get_code(e) == LV_EVENT_CLICKED) {
        power[0] = 0;
        power[chosen] = 0;

        hapticEffect(PULSE);

  esp_err_t result = esp_now_send(targetAdress, (uint8_t *) &power, sizeof(power));

    }
}

void meter2_event_cb(lv_event_t * e)
{
    if(lv_event_get_code(e) == LV_EVENT_CLICKED) {
        chosen = 1;
        set_active_meter(chosen);
        power[0] = power[chosen];

        hapticEffect(DOUBLE_CLICK);
    }
}

void meter3_event_cb(lv_event_t * e)
{
    if(lv_event_get_code(e) == LV_EVENT_CLICKED) {
        chosen = 2;
        set_active_meter(chosen);
        power[0] = power[chosen];

        hapticEffect(DOUBLE_CLICK);
    }
}

void meter4_event_cb(lv_event_t * e)
{
    if(lv_event_get_code(e) == LV_EVENT_CLICKED) {
        chosen = 3;
        set_active_meter(chosen);
        power[0] = power[chosen];

        hapticEffect(DOUBLE_CLICK);        
    }
}

void set_active_meter(int index)
{
    // disable all meter borders
    lv_obj_set_style_border_width(meter,  0, LV_PART_MAIN);
    lv_obj_set_style_border_width(meter2, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(meter3, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(meter4, 0, LV_PART_MAIN);

    // enable active meter border
    switch(index) {
        case 0:
            break;
        case 1:
            lv_obj_set_style_border_width(meter2, 2, LV_PART_MAIN);
            lv_obj_set_style_border_color(meter2, lv_color_hex(0xC0C000), LV_PART_MAIN);
            break;
        case 2:
            lv_obj_set_style_border_width(meter3, 2, LV_PART_MAIN);
            lv_obj_set_style_border_color(meter3, lv_color_hex(0xC0C000), LV_PART_MAIN);
            break;
        case 3:
            lv_obj_set_style_border_width(meter4, 2, LV_PART_MAIN);
            lv_obj_set_style_border_color(meter4, lv_color_hex(0xC0C000), LV_PART_MAIN);
            break;
    }
}


void loop() {

}
