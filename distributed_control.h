#ifndef DIST_CONTR_H
#define DIST_CONTR_H

class distributed_controller
{
public:
    float I, D, K, Ti, Td, b, h, y_old, N, Tt, es, K_old, b_old;
    explicit distributed_controller(float h_, float K_, float b_, float Ti_, float Td_, float N_, float Tt_);
    ~distributed_controller() {}
    bool check_feasibility(struct nodes node, float d[], int vector_size);
    float evaluate_cost(struct nodes node, float d[], float rho, int vector_size);
    float consensus_iterate(struct nodes node, float rho, int size_array, float d[]);
};

#endif // DIST_CONTR_H
