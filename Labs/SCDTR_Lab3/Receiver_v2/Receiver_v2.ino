#include <hardware/flash.h> //for flash_get_unique_id
#include "mcp2515.h"

uint8_t this_pico_flash_id[8], node_address;
struct can_frame canMsgTx, canMsgRx;
unsigned long counterTx {0}, counterRx {0};
MCP2515::ERROR err;
unsigned long time_to_write;
unsigned long write_delay {3000};
const byte interruptPin {20};
volatile byte data_available {false};
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};
float incomingByte = 0.0;
const int LED_PIN = 15;

//the interrupt service routine
void read_interrupt(uint gpio, uint32_t events) {
  can0.readMessage(&canMsgRx);
  data_available = true;
}

void setup() {
  flash_get_unique_id(this_pico_flash_id);
  node_address = this_pico_flash_id[7];
  Serial.begin();
  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode();
  gpio_set_irq_enabled_with_callback(
  interruptPin, GPIO_IRQ_EDGE_FALL,
  true, &read_interrupt );
}
void loop() {
    if( data_available ) {
      noInterrupts();
      can_frame frm {canMsgRx}; //local copy
      interrupts();
      Serial.print("Received message number ");
      Serial.print( counterRx++ );
      Serial.print(" from node ");
      Serial.print( frm.can_id , HEX);
      Serial.print(" : ");
      for (int i=0 ; i < frm.can_dlc ; i++)
        Serial.print((char) frm.data[ i ]);
      Serial.println(" ");
      Serial.print(frm.data.toInt());
      pwm = frm.data.toInt();
      data_available = false;
      analogWrite(LED_PIN, pwm);
  }
}
