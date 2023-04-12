#include <hardware/flash.h>
#include "pid.h"
#include "mcp2515.h"
#include "circular_buffer.h"
#include "buffered_data_struct.h"

// Variables for the PID Controller
const int LED_PIN = 15;
const int DAC_RANGE = 4096;
int sensor_value = 0;
float V_adc = 0.0;
float R_ldr = 0.0;
float lux = 0.0;
float duty_cycle = 0;
int pwm = 0;
float p_max = 0.108f;
float delta_t = 0.01;
float restart_time = 0;
bool occupancy = true;

bool is_streaming_lux = false;
bool is_streaming_dtc = false;
bool is_streaming_all = false;

const int NUM_MEASUREMENTS = 6000;
circular_buffer<NUM_MEASUREMENTS> buffer;

//     pid (h,    K,   b,   Ti,   Td, N,  Tt)
pid my_pid{0.01, 0.4, 0.1, 0.01, 0, 10, 5};
float r{7.0};

// Variables for CAN Bus
MCP2515 can0{spi0, 17, 19, 16, 18, 10000000};
uint8_t pico_flash_id[8];
uint8_t node_address;
unsigned long counterTx{0}, counterRx{0};
const uint8_t interruptPin{20};
volatile bool got_irq{false};
int number_of_detected_nodes = 0;
int redundancy_writings = 0;
int net_addresses[3] = {0, 0, 0};
int node_id = -1;
uint8_t b_read_message[4];

// Variables for the state machine
enum States
{
    START,
    WAITING_OTHERS_NODES,
    CALIBRATION,
    MANUAL,
    AUTO
};

States current_state = START;
bool enter_auto_state = false;
bool enter_manual_state = false;

enum inter_core_cmds
{
    ICC_READ_DATA = 1,  // From core1 to core0: contains data read (16 bit)
    ICC_WRITE_DATA = 2, // From core0 to core1: contains data to write (16 bit)
    ICC_ERROR_DATA = 3  // From core1 to core0: contains regs CANINTF, EFLG
};

// Interrupt to receive messages from the CAN Bus
void read_interrupt(uint gpio, uint32_t events)
{
    got_irq = true;
}

// Timer responsible for the sampling time of the ADC
volatile unsigned long int timer_time{0};
volatile bool timer_fired{false};
struct repeating_timer timer;

bool my_repeating_timer_callback(struct repeating_timer *t)
{
    if (!timer_fired)
    {
        timer_time = micros();
        timer_fired = true;
    }
    return true;
}

//////////////////////////////
// Core0:                   //
// - Serial communication   //
// - State machine          //
// - Control algorithm      //
//////////////////////////////

float sensor_value_to_lux(int sensor_value)
{
    V_adc = 3.3 * sensor_value / DAC_RANGE;
    R_ldr = 10000 * (3.3 - V_adc) / V_adc;
    lux = pow(pow(10, 6.15) / R_ldr, 1.25);
    return lux;
}

void setup()
{
    restart_time = millis();
    rp2040.idleOtherCore();
    flash_get_unique_id(pico_flash_id);
    rp2040.resumeOtherCore();
    node_address = pico_flash_id[7];
    net_addresses[0] = node_address;

    Serial.begin();
    analogReadResolution(12);
    add_repeating_timer_ms(-1000, my_repeating_timer_callback,
                           NULL, &timer);
}

// Protocolo de comunicação entre placas
// Val = 0 => WakeUp

void loop()
{
    can_frame frm;
    uint32_t msg;
    uint8_t b_write_message[4];
    process_user_request();
    run_state_machine();

    if (timer_fired)
    {
        if (current_state == START)
        {
            b_write_message[3] = ICC_WRITE_DATA;
            b_write_message[2] = node_address;
            b_write_message[0] = counterTx;
            b_write_message[1] = 0;
            rp2040.fifo.push(bytes_to_msg(b_write_message));
            Serial.print("s Sending");
            print_message(counterTx, b_write_message[2], b_write_message[2], 0);
            counterTx++;

            if ((net_addresses[1] != b_read_message[2]))
            {
                net_addresses[number_of_detected_nodes + 1] = b_read_message[2];
                number_of_detected_nodes++;
            }
            Serial.print(node_id);
            Serial.print(" ");
            Serial.print(net_addresses[0]);
            Serial.print(" ");
            Serial.print(net_addresses[1]);
            Serial.print(" ");
            Serial.print(net_addresses[2]);
            Serial.print(" ");
            Serial.println();
        }
        if (current_state == WAITING_OTHERS_NODES)
        {
            b_write_message[3] = ICC_WRITE_DATA;
            b_write_message[2] = node_address;
            b_write_message[0] = counterTx;
            b_write_message[1] = 0;
            rp2040.fifo.push(bytes_to_msg(b_write_message));
            Serial.print("w Sending");
            print_message(counterTx, b_write_message[2], b_write_message[2], 0);
            counterTx++;
            redundancy_writings++;
            Serial.print(node_id);
            Serial.print(" ");
            Serial.print(net_addresses[0]);
            Serial.print(" ");
            Serial.print(net_addresses[1]);
            Serial.print(" ");
            Serial.print(net_addresses[2]);
            Serial.print(" ");
            Serial.println();
        }
        if (current_state == CALIBRATION)
        {
            Serial.print(node_id);
            Serial.print(" ");
            Serial.print(net_addresses[0]);
            Serial.print(" ");
            Serial.print(net_addresses[1]);
            Serial.print(" ");
            Serial.print(net_addresses[2]);
            Serial.print(" ");
            Serial.println();
        }
        if (current_state == MANUAL || current_state == AUTO)
        {
            sensor_value = analogRead(A0);
            lux = sensor_value_to_lux(sensor_value);

            buffered_data tmp = buffered_data{
                (float)millis(), (float)pwm / 255.0f,
                r, lux};
            buffer.put(tmp);

            if (current_state == AUTO)
            {
                float y = lux;
                float u = my_pid.compute_control(r, y);
                pwm = (int)u;
                my_pid.housekeep(r, y);

                analogWrite(LED_PIN, pwm);
            }

            if (is_streaming_lux)
            {
                Serial.print("s l ");
                Serial.print(lux);
                Serial.print(" ");
                Serial.print((int)millis());
                Serial.println("");
            }
            if (is_streaming_dtc)
            {
                Serial.print("s d ");
                Serial.print((float)pwm / 255.0f);
                Serial.print(" ");
                Serial.print((int)millis());
                Serial.println("");
            }
            if (is_streaming_all)
            {
                Serial.print((int)millis());
                Serial.print(", ");
                Serial.print((float)pwm / 255.0f);
                Serial.print(", ");
                Serial.print(r);
                Serial.print(", ");
                Serial.print(lux);

                Serial.println("");
            }

            analogWrite(LED_PIN, pwm);
        }
        timer_fired = false;
    }

    // Interrupção para o FIFO
    if (rp2040.fifo.pop_nb(&msg))
    {
        msg_to_bytes(msg, b_read_message);
        if (b_read_message[3] == ICC_READ_DATA)
        {
            uint16_t val = msg;

            Serial.print("Received");
            print_message(counterRx, node_address, b_read_message[2], val);

            counterRx++;
        }
        else if (b_read_message[3] == ICC_ERROR_DATA)
        {
            print_can_errors(b_read_message[1], b_read_message[0]);
            can0.clearRXnOVRFlags();
            can0.clearInterrupts();
        }
    }
}

//////////////////////////////
// Core1:                   //
// - CAN Bus communication  //
//////////////////////////////

void setup1()
{ // core1 initialization
    can0.reset();
    can0.setBitrate(CAN_1000KBPS);
    can0.setNormalMode();
    gpio_set_irq_enabled_with_callback(interruptPin,
                                       GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &read_interrupt);
}

void loop1()
{
    can_frame frm;
    uint32_t msg;
    uint8_t b[4];

    // reading the can-bus and writing the fifo
    if (got_irq)
    {
        got_irq = false;
        uint8_t irq = can0.getInterrupts();
        if (irq & MCP2515::CANINTF_RX0IF)
        {
            can0.readMessage(MCP2515::RXB0, &frm);
            rp2040.fifo.push_nb(can_frame_to_msg(&frm));
        }
        if (irq & MCP2515::CANINTF_RX1IF)
        {
            can0.readMessage(MCP2515::RXB1, &frm);
            rp2040.fifo.push_nb(can_frame_to_msg(&frm));
        }
        if (can0.checkError())
        {
            uint8_t err = can0.getErrorFlags();
            rp2040.fifo.push_nb(error_flags_to_msg(irq, err));
        }
    }

    if (rp2040.fifo.pop_nb(&msg))
    { // read fifo write bus
        msg_to_bytes(msg, b);
        if (b[3] == ICC_WRITE_DATA)
        {
            frm.can_id = b[2];
            frm.can_dlc = 2;
            frm.data[1] = b[1];
            frm.data[0] = b[0];
            can0.sendMessage(&frm);
        }
    }
}