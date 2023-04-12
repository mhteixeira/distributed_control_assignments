void process_user_request()
{
    if (Serial.available())
    {
        bool is_command_valid = true;
        String request = Serial.readStringUntil('\n');
        String argument = request.substring(2);
        uint8_t b[4];
        switch (request.charAt(0))
        {
        case 'a':
            switch (request.charAt(2))
            {
            case '0':
                if (my_pid.get_anti_windup_status())
                {
                    my_pid.set_anti_windup_status(false);
                    Serial.println("ack");
                }
                else
                {
                    Serial.println("err: Already inactive");
                }
                break;
            case '1':
                if (!my_pid.get_anti_windup_status())
                {
                    my_pid.set_anti_windup_status(true);
                    Serial.println("ack");
                }
                else
                {
                    Serial.println("err: Already active");
                }
                break;
            default:
                is_command_valid = false;
                break;
            }
            break;
        case 'f':
            switch (request.charAt(2))
            {
            case '0':
                if (my_pid.get_feedforward_status())
                {
                    my_pid.set_feedforward_status(false);
                    Serial.println("ack");
                }
                else
                {
                    Serial.println("err: Already inactive");
                }
                break;
            case '1':
                if (!my_pid.get_feedforward_status())
                {
                    my_pid.set_feedforward_status(true);
                    Serial.println("ack");
                }
                else
                {
                    Serial.println("err: Already active");
                }
                break;
            default:
                is_command_valid = false;
                break;
            }
            break;
        case 'd':
            if (current_state == MANUAL)
            {
                pwm = (int)(request.substring(2).toFloat() * 255);
                Serial.println("ack");
            }
            else
            {
                Serial.println("err: Not in manual mode (request 'k 0')");
            }
            break;
        case 'r':
            r = request.substring(2).toFloat();
            Serial.println("ack");
            break;
        case 'g':
            switch (request.charAt(2))
            {
            case 'd':
                duty_cycle = pwm / 255.0;
                Serial.print("d ");
                Serial.println(duty_cycle);
                break;
            case 't':
                Serial.print("t ");
                Serial.println(millis() - restart_time);
                break;
            case 'e':
                Serial.print("e ");
                Serial.println(buffer.get_accumulated_energy_consumption(delta_t, p_max), 10);
                break;
            case 'v':
                Serial.print("v ");
                Serial.println(buffer.get_accumulated_visibility_error(), 10);
                break;
            case 'f':
                Serial.print("f ");
                Serial.println(buffer.get_accumulated_flicker_error(), 10);
                break;
            case 'p':
                Serial.print("p ");
                Serial.println(p_max * pwm / 255.0f);
                break;
            case 'r':
                Serial.print("r ");
                Serial.println(r);
                break;
            case 'l':
                Serial.print("l ");
                Serial.println(lux);
                break;
            case 'o':
                Serial.print("o ");
                Serial.println(occupancy);
                break;
            case 'b':
                switch (request.charAt(4))
                {
                case 'l':
                    Serial.print("b l ");
                    buffer.print_illuminances();
                    Serial.println();
                    break;
                case 'd':
                    Serial.print("b d ");
                    buffer.print_duty_cycles();
                    Serial.println();
                    break;
                default:
                    is_command_valid = false;
                    break;
                }
                break;
            default:
                is_command_valid = false;
                break;
            }
            break;
        case 'k':
            switch (request.charAt(2))
            {
            case '0':
                if (current_state == MANUAL)
                    Serial.println("err: Already on manual mode");
                else
                {
                    enter_manual_state = true;
                    Serial.println("ack");
                }
                break;
            case '1':
                if (current_state == AUTO)
                    Serial.println("err: Already on auto mode");
                else
                {
                    enter_auto_state = true;
                    Serial.println("ack");
                }
                break;
            default:
                is_command_valid = false;
                break;
            }
            break;
        case 'm':
            b[3] = ICC_WRITE_DATA;
            b[2] = node_address;
            b[0] = counterTx;
            b[1] = (request.substring(2).toInt() >> 8);
            rp2040.fifo.push(bytes_to_msg(b));
            Serial.print("Sending");
            print_message(counterTx, b[2], b[2], request.substring(2).toInt());
            counterTx++;
            break;
        case 'o':
            switch (request.charAt(2))
            {
            case '0':
                if (occupancy == 1)
                {
                    r = 3;
                    occupancy = false;
                    Serial.println("ack");
                }
                else
                {
                    Serial.println("err: Already unoccupied");
                }
                break;
            case '1':
                if (occupancy == 0)
                {
                    r = 7;
                    occupancy = true;
                    Serial.println("ack");
                }
                else
                {
                    Serial.println("err: Already occupied");
                }
                break;
            default:
                is_command_valid = false;
                break;
            }
            break;
        case 's':
            switch (request.charAt(2))
            {
            case 'l':
                if (is_streaming_lux == false)
                {
                    is_streaming_lux = true;
                    Serial.println("ack");
                }
                else
                {
                    Serial.println("err: Already streaming");
                }
                break;
            case 'd':
                if (is_streaming_dtc == false)
                {
                    is_streaming_dtc = true;
                    Serial.println("ack");
                }
                else
                {
                    Serial.println("err: Already streaming");
                }
                break;
            case 'a':
                if (is_streaming_all == false)
                {
                    is_streaming_all = true;
                    Serial.println("ack");
                }
                else
                {
                    Serial.println("err: Not streaming");
                }
                break;
            default:
                is_command_valid = false;
                break;
            }
            break;
        case 'S':
            switch (request.charAt(2))
            {
            case 'l':
                if (is_streaming_lux == true)
                {
                    is_streaming_lux = false;
                    Serial.println("ack");
                }
                else
                {
                    Serial.println("err: Not streaming");
                }
                break;
            case 'd':
                if (is_streaming_dtc == true)
                {
                    is_streaming_dtc = false;
                    Serial.println("ack");
                }
                else
                {
                    Serial.println("err: Not streaming");
                }
                break;
            case 'a':
                if (is_streaming_all == true)
                {
                    is_streaming_all = false;
                    Serial.println("ack");
                }
                else
                {
                    Serial.println("err: Not streaming");
                }
                break;
            default:
                is_command_valid = false;
                break;
            }
            break;
        default:
            is_command_valid = false;
            break;
        }

        if (!is_command_valid)
            Serial.println("Invalid command");
    }
}

void run_state_machine()
{
    switch (current_state)
    {
    case START:
        if (number_of_detected_nodes >= 2)
        {
            current_state = WAITING_OTHERS_NODES;
            node_id = (net_addresses[0] < node_address) + (net_addresses[1] < node_address) + (net_addresses[2] < node_address);
        }
        break;
    case WAITING_OTHERS_NODES:
        if (redundancy_writings >= 100)
        {
            current_state = CALIBRATION;
        }
        break;
    case AUTO:
        if (enter_manual_state)
        {
            current_state = MANUAL;
            enter_manual_state = false;
        }
        break;
    case MANUAL:
        if (enter_auto_state)
        {
            current_state = AUTO;
            enter_auto_state = false;
        }
        break;
    }
}