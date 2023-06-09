#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H
#include "buffered_data_struct.h"

template <int buffer_size = 16>
class circular_buffer
{
    buffered_data buffer[buffer_size]; // storage
    int head;                          // index of next element to write
    int tail;                          // index of next element to read
    int count;                         // counter of valid elements in the buffer
public:
    circular_buffer() : head{0}, tail{0}, count{0} {};
    ~circular_buffer(){};
    int capacity() { return buffer_size; }
    int size() { return count; }
    bool is_empty() { return count == 0; }
    bool is_full() { return count == buffer_size; }
    buffered_data peak() { return buffer[tail]; } // inspect but keep
    // value invalid if buffer empty

    void put(buffered_data elem)
    {
        buffer[head++] = elem;
        if (head == buffer_size)
            head = 0;
        if (!is_full())
            count++;
    }
    buffered_data take()
    {                                          // remove an element from the buffer
        buffered_data tmp_tail = buffer[tail]; // invalid if buffer empty
        if (!is_empty())
        {
            count--;
            tail++;
            if (tail == buffer_size)
                tail = 0;
        }
        return tmp_tail;
    }

    void print_illuminances()
    {
        if (!is_full())
        {
            for (int i = 0; i < head - 1; i++)
            {
                Serial.print(buffer[i].measured_illuminance);
                Serial.print(", ");
            }
            Serial.print(buffer[head].measured_illuminance);
        }
        else
        {
            int ptr = head;
            for (int i = 0; i < buffer_size - 1; i++)
            {
                Serial.print(buffer[ptr].measured_illuminance);
                Serial.print(", ");
                ptr++;
                if (ptr == buffer_size)
                    ptr = 0;
            }
            Serial.print(buffer[ptr].measured_illuminance);
        }
        Serial.println();
    }

    void print_duty_cycles()
    {
        if (!is_full())
        {
            for (int i = 0; i < head - 1; i++)
            {
                Serial.print(buffer[i].duty_cycle);
                Serial.print(", ");
            }
            Serial.print(buffer[head].duty_cycle);
        }
        else
        {
            int ptr = head;
            for (int i = 0; i < buffer_size - 1; i++)
            {
                Serial.print(buffer[ptr].duty_cycle);
                Serial.print(", ");
                ptr++;
                if (ptr == buffer_size)
                    ptr = 0;
            }
            Serial.print(buffer[ptr].duty_cycle);
        }
    }
    float get_accumulated_energy_consumption(float delta_t, float p_max)
    {
        float accumulated_energy_consumption;
        for (int i = 0; i < buffer_size; i++)
        {
            accumulated_energy_consumption += p_max * buffer[i].duty_cycle * delta_t;
        }
        return accumulated_energy_consumption;
    }

    float get_accumulated_visibility_error()
    {
        float accumulated_visibility_error;
        for (int i = 0; i < buffer_size; i++)
        {
            accumulated_visibility_error += max(
                0,
                buffer[i].reference_illuminance - buffer[i].measured_illuminance);
        }
        return accumulated_visibility_error;
    }

    float get_accumulated_flicker_error()
    {
        float accumulated_flicker_error;

        if (!is_full())
        {
            for (int i = 2; i < head; i++)
            {
                float condition = (buffer[i].duty_cycle - buffer[i - 1].duty_cycle) * (buffer[i - 1].duty_cycle - buffer[i - 2].duty_cycle);
                if (condition < 0)
                {
                    accumulated_flicker_error += (abs(buffer[i].duty_cycle - buffer[i - 1].duty_cycle) +
                                                  abs(buffer[i - 1].duty_cycle - buffer[i - 2].duty_cycle));
                }
            }
        }
        else
        {
            int ptr1 = tail;
            int ptr2;
            int ptr3;

            if (tail + 2 == buffer_size)
            {
                ptr2 = tail + 1;
                ptr3 = 0;
            }
            else if (tail + 1 == buffer_size)
            {
                ptr2 = 0;
                ptr3 = 1;
            }
            else
            {
                ptr2 = tail + 1;
                ptr3 = tail + 2;
            }

            for (int i = 0; i < buffer_size - 2; i++)
            {
                float condition = (buffer[i].duty_cycle - buffer[i - 1].duty_cycle) * (buffer[i - 1].duty_cycle - buffer[i - 2].duty_cycle);
                if (condition < 0)
                {
                    accumulated_flicker_error += (abs(buffer[i].duty_cycle - buffer[i - 1].duty_cycle) +
                                                  abs(buffer[i - 1].duty_cycle - buffer[i - 2].duty_cycle));
                }
                ptr1 = ptr2;
                ptr2 = ptr3;
                ptr3++;
                if (ptr3 == buffer_size)
                    ptr3 = 0;
            }
        }

        return accumulated_flicker_error;
    }
};
#endif // CIRCULAR_BUFFER_H