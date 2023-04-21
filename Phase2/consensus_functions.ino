float evaluate_cost(struct nodes node, float d[], float rho, int vector_size)
{
    float norm = 0;
    float sum1 = 0;
    float sum2 = 0;
    for (int i = 0; i < vector_size; i++)
    {
        sum1 += node.c[i] * d[i];
        sum2 += node.y[i] * (d[i] - node.d_av[i]);
        norm += (d[i] - node.d_av[i]) * (d[i] - node.d_av[i]);
    }
    return sum1 + sum2 + rho / 2 * norm;
}

bool check_feasibility(struct nodes node, float d[], int vector_size)
{
    float tol = 0.001; // tolerance for rounding errors
    if (d[node.index] < 0 - tol)
        return false;
    if (d[node.index] > 100 + tol)
        return false;
    float d_k{0};
    float L_o_t{0};
    for (int i = 0; i < vector_size; i++)
    {
        d_k += d[i] * node.k[i];
    }
    L_o_t += node.L - node.o - tol;
    if (d_k < L_o_t)
        return false;
    return true;
}

float consensus_iterate(struct nodes node, float rho, int size_array, float d[])
{
    float d_best[size_array];
    float cost_best{1000000};
    bool sol_unconstrained{true};
    bool sol_boundary_linear{true};
    bool sol_boundary_0{true};
    bool sol_boundary_100{true};
    bool sol_linear_0{true};
    bool sol_linear_100{true};
    float z[size_array];
    float d_u[size_array];
    float cost_unconstrained{10000};
    float cost_boundary_linear{10000};
    float cost_boundary_0{10000};
    float cost_boundary_100{1000};
    float cost_linear_0{1000};
    float d_b0[size_array];
    float d_bl[size_array];
    float d_b1[size_array];
    float d_l0[size_array];
    float d_l1[size_array];

    for (int i = 0; i < size_array; i++)
    {
        z[i] = rho * node.d_av[i] - node.y[i] - node.c[i];
    }
    for (int i = 0; i < size_array; i++)
    {
        d_u[i] = (1 / rho) * z[i];
    }
    sol_unconstrained = check_feasibility(node, d_u, size_array);
    if (sol_unconstrained)
    {
        // REVISE: IF UNCONSTRAINED SOLUTION EXISTS, THEN IT IS OPTIMAL
        // NO NEED TO COMPUTE THE OTHER
        cost_unconstrained = evaluate_cost(node, d_u, rho, size_array);
        if (cost_unconstrained < cost_best)
        {
            for (int i = 0; i < size_array; i++)
            {
                d_best[i] = d_u[i];
            }
            cost_best = cost_unconstrained;
        }
    }
    // compute minimum constrained to linear boundary
    float prod_int{0};
    for (int i = 0; i < size_array; i++)
    {
        prod_int += z[i] * node.k[i];
    }
    for (int i = 0; i < size_array; i++)
    {
        d_bl[i] = 1 / rho * z[i] - node.k[i] / node.n * (node.o - node.L + (1 / rho) * prod_int);
    }
    // check feasibility of minimum constrained to linear boundary
    sol_boundary_linear = check_feasibility(node, d_bl, size_array);
    // compute cost and if best store new optimum
    if (sol_boundary_linear)
    {
        cost_boundary_linear = evaluate_cost(node, d_bl, rho, size_array);
        if (cost_boundary_linear < cost_best)
        {
            for (int i = 0; i < size_array; i++)
            {
                d_best[i] = d_bl[i];
            }
            cost_best = cost_boundary_linear;
        }
    }
    // compute minimum constrained to 0 boundary

    for (int i = 0; i < size_array; i++)
    {
        d_b0[i] = (1 / rho) * z[i];
    }
    d_b0[node.index] = 0;
    // check feasibility of minimum constrained to 0 boundary
    sol_boundary_0 = check_feasibility(node, d_b0, size_array);
    if (sol_boundary_0)
    {
        cost_boundary_0 = evaluate_cost(node, d_b0, rho, size_array);
        if (cost_boundary_0 < cost_best)
        {
            for (int i = 0; i < size_array; i++)
            {
                d_best[i] = d_b0[i];
            }
            cost_best = cost_boundary_0;
        }
    }
    // compute minimum constrained to 100 boundary
    for (int i = 0; i < size_array; i++)
    {
        d_b1[i] = (1 / rho) * z[i];
    }
    d_b1[node.index] = 100;
    // check feasibility of minimum constrained to 100 boundary
    sol_boundary_100 = check_feasibility(node, d_b1, size_array);
    if (sol_boundary_100)
    {
        cost_boundary_100 = evaluate_cost(node, d_b1, rho, size_array);
        if (cost_boundary_100 < cost_best)
        {
            for (int i = 0; i < size_array; i++)
            {
                d_best[i] = d_b1[i];
            }
            cost_best = cost_boundary_100;
        }
    }
    // compute minimum constrained to linear and 0 boundary
    float sub{0};
    for (int i = 0; i < size_array; i++)
    {
        sub += z[i] * node.k[i];
    }
    for (int i = 0; i < size_array; i++)
    {
        d_l0[i] = (1 / rho) * z[i] - (1 / node.m) * node.k[i] * (node.o - node.L) + (1 / rho / node.m) * node.k[i] * (node.k[node.index] * z[node.index] - sub);
    }
    d_l0[node.index] = 0;
    // check feasibility of minimum constrained to linear and 0 boundary
    sol_linear_0 = check_feasibility(node, d_l0, size_array);
    // compute cost and if best store new optimum
    if (sol_linear_0)
    {
        cost_linear_0 = evaluate_cost(node, d_l0, rho, size_array);
        if (cost_linear_0 < cost_best)
        {
            for (int i = 0; i < size_array; i++)
            {
                d_best[i] = d_l0[i];
            }
            cost_best = cost_linear_0;
        }
    }
    // compute minimum constrained to linear and 100 boundary
    for (int i = 0; i < size_array; i++)
    {
        d_l1[i] = (1 / rho) * z[i] - (1 / node.m) * node.k[i] * (node.o - node.L + 100 * node.k[node.index]) + (1 / rho / node.m) * node.k[i] * (node.k[node.index] * z[node.index] - sub);
    }
    d_l1[node.index] = 100;
    // check feasibility of minunum constrained to linear and 0 boundary
    sol_linear_100 = check_feasibility(node, d_l1, size_array);
    // compute cost and if best store new optimum
    if (sol_linear_100)
    {
        cost_linear_0 = evaluate_cost(node, d_l1, rho, size_array);
        if (cost_linear_0 < cost_best)
        {
            for (int i = 0; i < size_array; i++)
            {
                d_best[i] = d_l1[i];
            }
            cost_best = cost_linear_0;
        }
    }
    for (int i = 0; i < size_array; i++)
    {
        d[i] = d_best[i];
    }
    return cost_best;
}

// void transfer_data_consensus(int j, int current_node_sending_data, float data)
void transfer_data_consensus(int j, int current_node_sending_data)
{
    if (node_id == current_node_sending_data)
    {
        uint8_t *array;

        array = reinterpret_cast<uint8_t *>(&control_agent.d[j]);
        send_message_to_bus(FLOAT_PT1, array[0]);
        send_message_to_bus(FLOAT_PT2, array[1]);
        send_message_to_bus(FLOAT_PT3, array[2]);
        send_message_to_bus(FLOAT_PT4, array[3]);
        // Serial.print("Sending ");
        // Serial.println(control_agent.d[j]);
        if (number_of_acks_received >= 2)
        {
            step_to_consensus++;
        }
    }
    else
    {
        if (uint8_to_float_receptions[0] && uint8_to_float_receptions[1] && uint8_to_float_receptions[2] && uint8_to_float_receptions[3])
        {
            d_matrix[current_node_sending_data][j] = *reinterpret_cast<float *>(uint8_to_float_aux_array);
            // Serial.print("Received ");
            // Serial.println(d_matrix[current_node_sending_data][j]);
            send_message_to_bus(ACKNOWLEDGE);
            step_to_consensus++;
        };
    }
}

void syncronize_data_transfer(int current_node_sending_data)
{
    number_of_acks_received = 0;
    reset_receptions_array();
    // float test = *reinterpret_cast<float *>(uint8_to_float_aux_array);
    // Serial.prinSerial.println(test);
    if (node_id == current_node_sending_data)
    {
        send_message_to_bus(NEXT_CONSENSUS_DC, 0);
        step_to_consensus++;
    }
}

void print_d_matrix(int iteration)
{
    Serial.println(" ");
    Serial.print("Consensus iteration: ");
    Serial.println(iteration);
    Serial.print(d_matrix[0][0]);
    Serial.print(" ");
    Serial.print(d_matrix[0][1]);
    Serial.print(" ");
    Serial.print(d_matrix[0][2]);
    Serial.println(" ");
    Serial.print(d_matrix[1][0]);
    Serial.print(" ");
    Serial.print(d_matrix[1][1]);
    Serial.print(" ");
    Serial.print(d_matrix[1][2]);
    Serial.println(" ");
    Serial.print(d_matrix[2][0]);
    Serial.print(" ");
    Serial.print(d_matrix[2][1]);
    Serial.print(" ");
    Serial.print(d_matrix[2][2]);
    Serial.println(" ");
}

void complete_consensus()
{
    switch (step_to_consensus)
    {
    case 0:
        cost = consensus_iterate(control_agent, rho, 3, d_matrix[node_id]);

        for (int j = 0; j < 3; j++)
        {
            control_agent.d[j] = d_matrix[node_id][j];
        }
        number_of_acks_received = 0;
        step_to_consensus++;
        current_node = 0;
        break;
    // Now the 9 data transfer (all gains)
    case 1:
        transfer_data_consensus(0, current_node);
        break;
    case 2:
        syncronize_data_transfer(current_node);
        break;
    case 3:
        transfer_data_consensus(1, current_node);
        break;
    case 4:
        syncronize_data_transfer(current_node);
        break;
    case 5:
        transfer_data_consensus(2, current_node);
        break;
    case 6:
        syncronize_data_transfer(current_node);
        break;
    case 7:
        current_node = 1;
        transfer_data_consensus(0, current_node);
        break;
    case 8:
        syncronize_data_transfer(current_node);
        break;
    case 9:
        transfer_data_consensus(1, current_node);
        break;
    case 10:
        syncronize_data_transfer(current_node);
        break;
    case 11:
        transfer_data_consensus(2, current_node);
        break;
    case 12:
        syncronize_data_transfer(current_node);
        break;
    case 13:
        current_node = 2;
        transfer_data_consensus(0, current_node);
        break;
    case 14:
        syncronize_data_transfer(current_node);
        break;
    case 15:
        transfer_data_consensus(1, current_node);
        break;
    case 16:
        syncronize_data_transfer(current_node);
        break;
    case 17:
        transfer_data_consensus(2, current_node);
        break;
    case 18:
        syncronize_data_transfer(current_node);
        break;
    case 19:
        print_d_matrix(consensus_iteration);
        for (int j = 0; j < 3; j++)
        {
            control_agent.d_av[j] = (d_matrix[0][j] + d_matrix[1][j] + d_matrix[2][j]) / 3;
            control_agent.y[j] = control_agent.y[j] + rho * (control_agent.d[j] - control_agent.d_av[j]);
        }
        step_to_consensus = 0;
        consensus_iteration++;
        break;
    default:
        Serial.println("???");
        break;
    }
}
