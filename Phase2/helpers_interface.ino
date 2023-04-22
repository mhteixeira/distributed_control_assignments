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
            if (request.substring(2) == "")
            {
                is_calibrated = false;
                Serial.println("reset");
            }
            else
            {
                r = request.substring(2).toFloat();
                finished_consensus = false;
                control_agent.L = r;
                send_message_to_bus(REDO_CONSENSUS);
                Serial.println("ack");
            }
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
            case '2':
                if (current_state == CONSENSUS)
                    Serial.println("err: Already on auto mode");
                else
                {
                    enter_consensus_state = true;
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
                    finished_consensus = false;
                    control_agent.L = r;
                    send_message_to_bus(REDO_CONSENSUS);
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
                    finished_consensus = false;
                    control_agent.L = r;
                    send_message_to_bus(REDO_CONSENSUS);
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

void reset_values()
{
    restart_time = millis();
    occupancy = true;

    is_streaming_lux = false;
    is_streaming_dtc = false;
    is_streaming_all = false;
    r = 7.0;
    number_of_detected_nodes = 0;
    net_addresses[1] = 0;
    net_addresses[2] = 0;
    k_ij[0] = 0;
    k_ij[1] = 0;
    k_ij[2] = 0;
    number_of_acks_received = 0;
    last_ack_id = -1;
    node_id = -1;
}

void run_state_machine()
{
    switch (current_state)
    {
    case START:
        if (number_of_detected_nodes >= 2)
        {
            current_state = WAITING_OTHERS_NODES;
            current_time = millis();
            node_id = get_node_id_from_address(node_address);
            Serial.println("\n##########################");
            Serial.println("Finished wake up procedure");
            Serial.println("##########################\n");
            Serial.print("Net addresses: ");
            Serial.print(net_addresses[0]);
            Serial.print(" (mine), ");
            Serial.print(net_addresses[1]);
            Serial.print(" and ");
            Serial.print(net_addresses[2]);
            Serial.println();
            Serial.print("My node id: ");
            Serial.println(node_id);
            if (node_id == 0)
            {
                Serial.println("I'm the Hub");
            }
        }
        break;
    case WAITING_OTHERS_NODES:
        if (millis() - current_time > 1000)
        {
            current_state = CALIBRATION;
            Serial.println("\n#######################");
            Serial.println("Starting calibration...");
            Serial.println("#######################\n");
        }
        break;
    case CALIBRATION:
        if (is_calibrated)
        {

            control_agent.index = node_id;
            control_agent.k[0] = k_ij[0];
            control_agent.k[1] = k_ij[1];
            control_agent.k[2] = k_ij[2];
            control_agent.n = pow(k_ij[0], 2) + pow(k_ij[1], 2) + pow(k_ij[2], 2);
            control_agent.m = control_agent.n - pow(k_ij[node_id], 2);
            control_agent.c[node_id] = c;
            control_agent.o = external_illuminance;
            control_agent.L = r;

            current_state = CONSENSUS;
            Serial.println("\n#######################");
            Serial.println("Calibration finished");
            Serial.println("#######################\n");
            print_calibration();
        }
        break;
    case CONSENSUS:
        if (!is_calibrated)
        {
            reset_values();
            current_state = START;
        }
        if (enter_auto_state)
        {
            current_state = AUTO;
            enter_auto_state = false;
        }
        if (enter_manual_state)
        {
            current_state = MANUAL;
            enter_manual_state = false;
        }
        break;
    case AUTO:
        if (enter_manual_state)
        {
            current_state = MANUAL;
            enter_manual_state = false;
        }
        if (enter_consensus_state)
        {
            current_state = CONSENSUS;
            enter_consensus_state = false;
        }
        if (!is_calibrated)
        {
            reset_values();
            current_state = START;
        }
        break;
    case MANUAL:
        if (enter_consensus_state)
        {
            current_state = CONSENSUS;
            enter_consensus_state = false;
        }
        if (enter_auto_state)
        {
            current_state = AUTO;
            enter_auto_state = false;
        }
        if (!is_calibrated)
        {
            reset_values();
            current_state = START;
        }
        break;
    }
}