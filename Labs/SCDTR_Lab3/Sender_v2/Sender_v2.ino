
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
  time_to_write = millis() + write_delay;
}
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int r = input.toInt();
    canMsgTx.can_id = node_address;
    canMsgTx.can_dlc = 8;
    unsigned long div = r*10;
    for( int i = 0; i < 8; i++ )
      canMsgTx.data[7-i]='0'+((div/=10)%10);
      noInterrupts();
      err = can0.sendMessage(&canMsgTx);
      interrupts();
      Serial.print("Sending message ");
      Serial.print( counterTx );
      Serial.print(" from node ");
      Serial.println( node_address, HEX );
      counterTx++;
    }
}
