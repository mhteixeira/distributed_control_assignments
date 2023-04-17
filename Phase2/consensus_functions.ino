
// float evaluate_cost(struct nodes node, float d[], float rho, int vector_size)
// {
//     float norm = 0;
//     float sum1 = 0;
//     float sum2 = 0;
//     for (int i = 0; i < vector_size; i++)
//     {
//         sum1 += node.c[i] * d[i];
//         sum2 += node.y[i] * (d[i] - node.d_av[i]);
//         norm += (d[i] - node.d_av[i]) * (d[i] - node.d_av[i]);
//     }
//     return sum1 + sum2 + rho / 2 * norm;
// }

// bool check_feasibility(struct nodes node, float d[], int vector_size)
// {
//     float tol = 0.001; // tolerance for rounding errors
//     if (d[node.index] < 0 - tol)
//         return false;
//     if (d[node.index] > 100 + tol)
//         return false;
//     float d_k{0};
//     float L_o_t{0};
//     for (int i = 0; i < vector_size; i++)
//     {
//         d_k += d[i] * node.k[i];
//     }
//     L_o_t += node.L - node.o - tol;
//     if (d_k < L_o_t)
//         return false;
//     return true;
// }

// float consensus_iterate(struct nodes node, float rho, int size_array, float d[])
// {
//     float d_best[size_array];
//     float cost_best{1000000};
//     bool sol_unconstrained{true};
//     bool sol_boundary_linear{true};
//     bool sol_boundary_0{true};
//     bool sol_boundary_100{true};
//     bool sol_linear_0{true};
//     bool sol_linear_100{true};
//     float z[size_array];
//     float d_u[size_array];
//     float cost_unconstrained{10000};
//     float cost_boundary_linear{10000};
//     float cost_boundary_0{10000};
//     float cost_boundary_100{1000};
//     float cost_linear_0{1000};
//     float d_b0[size_array];
//     float d_bl[size_array];
//     float d_b1[size_array];
//     float d_l0[size_array];
//     float d_l1[size_array];

//     for (int i = 0; i < size_array; i++)
//     {
//         z[i] = rho * node.d_av[i] - node.y[i] - node.c[i];
//     }
//     for (int i = 0; i < size_array; i++)
//     {
//         d_u[i] = (1 / rho) * z[i];
//     }
//     sol_unconstrained = check_feasibility(node, d_u, size_array);
//     if (sol_unconstrained)
//     {
//         // REVISE: IF UNCONSTRAINED SOLUTION EXISTS, THEN IT IS OPTIMAL
//         // NO NEED TO COMPUTE THE OTHER
//         cost_unconstrained = evaluate_cost(node, d_u, rho, size_array);
//         if (cost_unconstrained < cost_best)
//         {
//             for (int i = 0; i < size_array; i++)
//             {
//                 d_best[i] = d_u[i];
//             }
//             cost_best = cost_unconstrained;
//         }
//     }
//     // compute minimum constrained to linear boundary
//     float prod_int{0};
//     for (int i = 0; i < size_array; i++)
//     {
//         prod_int += z[i] * node.k[i];
//     }
//     for (int i = 0; i < size_array; i++)
//     {
//         d_bl[i] = 1 / rho * z[i] - node.k[i] / node.n * (node.o - node.L + (1 / rho) * prod_int);
//     }
//     // check feasibility of minimum constrained to linear boundary
//     sol_boundary_linear = check_feasibility(node, d_bl, size_array);
//     // compute cost and if best store new optimum
//     if (sol_boundary_linear)
//     {
//         cost_boundary_linear = evaluate_cost(node, d_bl, rho, size_array);
//         if (cost_boundary_linear < cost_best)
//         {
//             for (int i = 0; i < size_array; i++)
//             {
//                 d_best[i] = d_bl[i];
//             }
//             cost_best = cost_boundary_linear;
//         }
//     }
//     // compute minimum constrained to 0 boundary

//     for (int i = 0; i < size_array; i++)
//     {
//         d_b0[i] = (1 / rho) * z[i];
//     }
//     d_b0[node.index] = 0;
//     // check feasibility of minimum constrained to 0 boundary
//     sol_boundary_0 = check_feasibility(node, d_b0, size_array);
//     if (sol_boundary_0)
//     {
//         cost_boundary_0 = evaluate_cost(node, d_b0, rho, size_array);
//         if (cost_boundary_0 < cost_best)
//         {
//             for (int i = 0; i < size_array; i++)
//             {
//                 d_best[i] = d_b0[i];
//             }
//             cost_best = cost_boundary_0;
//         }
//     }
//     // compute minimum constrained to 100 boundary
//     for (int i = 0; i < size_array; i++)
//     {
//         d_b1[i] = (1 / rho) * z[i];
//     }
//     d_b1[node.index] = 100;
//     // check feasibility of minimum constrained to 100 boundary
//     sol_boundary_100 = check_feasibility(node, d_b1, size_array);
//     if (sol_boundary_100)
//     {
//         cost_boundary_100 = evaluate_cost(node, d_b1, rho, size_array);
//         if (cost_boundary_100 < cost_best)
//         {
//             for (int i = 0; i < size_array; i++)
//             {
//                 d_best[i] = d_b1[i];
//             }
//             cost_best = cost_boundary_100;
//         }
//     }
//     // compute minimum constrained to linear and 0 boundary
//     float sub{0};
//     for (int i = 0; i < size_array; i++)
//     {
//         sub += z[i] * node.k[i];
//     }
//     for (int i = 0; i < size_array; i++)
//     {
//         d_l0[i] = (1 / rho) * z[i] - (1 / node.m) * node.k[i] * (node.o - node.L) + (1 / rho / node.m) * node.k[i] * (node.k[node.index] * z[node.index] - sub);
//     }
//     d_l0[node.index] = 0;
//     // check feasibility of minimum constrained to linear and 0 boundary
//     sol_linear_0 = check_feasibility(node, d_l0, size_array);
//     // compute cost and if best store new optimum
//     if (sol_linear_0)
//     {
//         cost_linear_0 = evaluate_cost(node, d_l0, rho, size_array);
//         if (cost_linear_0 < cost_best)
//         {
//             for (int i = 0; i < size_array; i++)
//             {
//                 d_best[i] = d_l0[i];
//             }
//             cost_best = cost_linear_0;
//         }
//     }
//     // compute minimum constrained to linear and 100 boundary
//     for (int i = 0; i < size_array; i++)
//     {
//         d_l1[i] = (1 / rho) * z[i] - (1 / node.m) * node.k[i] * (node.o - node.L + 100 * node.k[node.index]) + (1 / rho / node.m) * node.k[i] * (node.k[node.index] * z[node.index] - sub);
//     }
//     d_l1[node.index] = 100;
//     // check feasibility of minunum constrained to linear and 0 boundary
//     sol_linear_100 = check_feasibility(node, d_l1, size_array);
//     // compute cost and if best store new optimum
//     if (sol_linear_100)
//     {
//         cost_linear_0 = evaluate_cost(node, d_l1, rho, size_array);
//         if (cost_linear_0 < cost_best)
//         {
//             for (int i = 0; i < size_array; i++)
//             {
//                 d_best[i] = d_l1[i];
//             }
//             cost_best = cost_linear_0;
//         }
//     }
//     for (int i = 0; i < size_array; i++)
//     {
//         d[i] = d_best[i];
//     }
//     return cost_best;
// }