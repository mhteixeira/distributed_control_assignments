#include <iostream>
using namespace std;
#include <bitset>

struct nodes{
    int index;
    float d[3];
    float d_av[3];
    float y[3];
    float k[3];
    float n;
    float m;
    float c[3];
    float o;
    float L;
};

float evaluate_cost(struct nodes node, float d[], float rho, int vector_size){
    float cost = 0;
    float norm = 0;
    float prod_int = 0;
    float prod_int2=0;
    for(int i = 0; i< vector_size; i++){
        prod_int += node.c[i]*d[i];
        prod_int2 += node.y[i]*(d[i]-node.d_av[i]);
        norm += (d[i]-node.d_av[i])*(d[i]-node.d_av[i]);
    }
    cost = prod_int + prod_int2+rho/2*norm;
    return cost;
}

bool check_feasibility(struct nodes node, float d[], int vector_size){
    float tol = 0.001; //tolerance for rounding errors
    if (d[node.index] < 0-tol) return false;
    if (d[node.index] > 100 + tol) return false;
    float d_k{0};
    float L_o_t{0};
    for(int i=0; i<vector_size;i++){
        d_k += d[i]*node.k[i];
    }
    L_o_t += node.L - node.o - tol; 
    if(d_k < L_o_t) return false;
    return true;
}

float consensus_iterate(struct nodes node, float rho, int size_array, float d[]){
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

    for(int i=0; i<size_array;i++){
        z[i]= rho*node.d_av[i]-node.y[i] - node.c[i];
    }
    for(int i=0; i<size_array;i++){
        d_u[i]=  (1/rho)*z[i];
    }
    sol_unconstrained = check_feasibility(node, d_u, size_array);
    if(sol_unconstrained){
        //REVISE: IF UNCONSTRAINED SOLUTION EXISTS, THEN IT IS OPTIMAL
        //NO NEED TO COMPUTE THE OTHER
            cost_unconstrained = evaluate_cost(node, d_u, rho, size_array);
            if (cost_unconstrained < cost_best){
                for(int i =0; i<size_array; i++){
                   d_best[i] = d_u[i];
                }
                cost_best = cost_unconstrained;
            }
    }
    //compute minimum constrained to linear boundary
    float prod_int{0};
    for(int i =0; i<size_array; i++){
        prod_int+= z[i]*node.k[i];
    }
    for(int i =0; i<size_array;i++){
        d_bl[i] = 1/rho*z[i] - node.k[i]/node.n*(node.o - node.L + (1/rho)*prod_int);
    }
    //check feasibility of minimum constrained to linear boundary
    sol_boundary_linear = check_feasibility(node, d_bl, size_array);
    //compute cost and if best store new optimum
    if(sol_boundary_linear){
        cost_boundary_linear = evaluate_cost(node, d_bl, rho, size_array);
        if(cost_boundary_linear < cost_best){
            for(int i =0; i<size_array; i++){
              d_best[i] = d_bl[i];
            }
            cost_best = cost_boundary_linear;
        }
    }
    //compute minimum constrained to 0 boundary

    for(int i =0; i<size_array; i++){
        d_b0[i] = (1/rho)*z[i];
    }
    d_b0[node.index] = 0;
    //check feasibility of minimum constrained to 0 boundary
    sol_boundary_0 = check_feasibility(node, d_b0, size_array);
    if(sol_boundary_0){
        cost_boundary_0 = evaluate_cost(node, d_b0, rho, size_array);
        if(cost_boundary_0 < cost_best){
            for(int i =0; i<size_array; i++){
              d_best[i] = d_b0[i];
            }
            cost_best = cost_boundary_0;
        }
    }
    //compute minimum constrained to 100 boundary
    for(int i =0; i<size_array; i++){
        d_b1[i] = (1/rho)*z[i];
    }
    d_b1[node.index] = 100;
     //check feasibility of minimum constrained to 100 boundary
    sol_boundary_100 = check_feasibility(node, d_b1, size_array);
    if(sol_boundary_100){
        cost_boundary_100 = evaluate_cost(node, d_b1, rho, size_array);
        if(cost_boundary_100 < cost_best){
            for(int i =0; i<size_array; i++){
              d_best[i] = d_b1[i];
            }            
            cost_best = cost_boundary_100;
        }
    }
    //compute minimum constrained to linear and 0 boundary
    float sub{0};
    for(int i=0;i<size_array;i++){
        sub+=z[i]*node.k[i];
    }
    for(int i =0; i<size_array; i++){
        d_l0[i] = (1/rho)*z[i] - (1/node.m)*node.k[i]*(node.o - node.L) + (1/rho/node.m)*node.k[i]*(node.k[node.index]*z[node.index]- sub);
    }
    d_l0[node.index] = 0;
    //check feasibility of minimum constrained to linear and 0 boundary
    sol_linear_0 = check_feasibility(node, d_l0, size_array);
    //compute cost and if best store new optimum
    if(sol_linear_0){
        cost_linear_0 = evaluate_cost(node,d_l0,rho,size_array);
        if(cost_linear_0 < cost_best){
            for(int i =0; i<size_array; i++){
              d_best[i] = d_l0[i];
            }
            cost_best = cost_linear_0;
        }
    }
    // compute minimum constrained to linear and 100 boundary
    for(int i =0; i<size_array; i++){
        d_l1[i] = (1/rho)*z[i] - (1/node.m)*node.k[i]*(node.o - node.L + 100*node.k[node.index]) + (1/rho/node.m)*node.k[i]*(node.k[node.index]*z[node.index]- sub);
    }
    d_l1[node.index] = 100;
    //check feasibility of minunum constrained to linear and 0 boundary
    sol_linear_100 = check_feasibility(node, d_l1, size_array);
    //compute cost and if best store new optimum
    if(sol_linear_100){
        cost_linear_0 = evaluate_cost(node,d_l1,rho, size_array);
        if(cost_linear_0 < cost_best){
            for(int i =0; i<size_array; i++){
              d_best[i] = d_l1[i];
            }
            cost_best = cost_linear_0;
        }
    }
    for(int i =0; i<size_array; i++){
       d[i] = d_best[i];
    }
    return cost_best;
}

//case of test
float L1 = 80; 
float o1 = 0.3153306; 
float L2 = 80; 
float o2 = 0.3894527; 
float L3 = 80; 
float o3=0.4335906;

//individual cost
float c1 = 1;
float c2 = 1; 
float c3 = 1;

//system calibration parameters
float k11 = 3.3256989;
float k12 = 0.1449770;
float k13 = 0.0232854; 
float k21 = 0.3541722; 
float k22 = 1.6310002;
float k23=0.0533489; 
float k31 = 0.0716771;
float k32=1.1261380;
float k33= 1.3077351;

float rho = 1;
const int maxiter = 50;

//history of distrbuting solution
//float d11[maxiter]{0}, d12[maxiter]{0}, d21[maxiter]{0}, d22[maxiter]{0}, av1[maxiter]{0}, av2[maxiter]{0};

//distributed node initialization
struct nodes node1 = {{0},{0,0},{0,0},{0,0},{k11,k12,k13},{11.0818},{0.0216},{c1,0,0},{o1},{L1}};
struct nodes node2 = {{1},{0,0},{0,0},{0,0},{k21,k22,k23},{2.7884},{0.1283},{0,c2,0},{o2},{L2}};
struct nodes node3 = {{2},{0,0},{0,0},{0,0},{k31,k32,k33},{2.9835},{1.2733},{0,0,c3},{o3},{L3}};

float l[3]{0,0,0};
float d[3]{0,0,0};


//variaveis de teste
//float teste_arr[2]{0,0};
//bool testand = consensus_iterate(node1,rho,2,teste_arr);
float d1[3]{0,0,0};

int main(){
//   d11[0] = node1.d[0];
//   d12[0] = node1.d[1];
//   d21[0] = node2.d[0];
//   d22[0] = node2.d[1];
//   av1[0] = (d11[0]+d21[0]+d31[0])/3;
//   av2[0] = (d12[0]+d22[0]+d32[0])/3;
//   av3[0] = (d13[0]+d23[0]+d33[0])/3;
  for(int i =1; i<500; i++){
    //computation of the primal solutions
    //node 1
    float cost1 = consensus_iterate(node1, rho, 3, d1);
    for(int j=0; j<3; j++) node1.d[j] = d1[j];

    //node 2
    float d2[3]{0,0,0};
    float cost2 = consensus_iterate(node2, rho, 3, d2);
    for(int j=0; j<3; j++){ node2.d[j] = d2[j];}
    
    //node 3
    float d3[3]{0,0,0};
    float cost3 = consensus_iterate(node3, rho, 3, d3);
    for(int j=0; j<3; j++){ node3.d[j] = d3[j];}   

    for(int j=0; j<3; j++){
      node1.d_av[j] = (node1.d[j]+node2.d[j]+node3.d[j])/3;
      node2.d_av[j] = (node1.d[j]+node2.d[j]+node3.d[j])/3;
      node3.d_av[j] = (node1.d[j]+node2.d[j]+node3.d[j])/3;
    }
    //compuation of the lagrangian updates
    for(int j=0; j<3; j++){
      node1.y[j] = node1.y[j] + rho*(node1.d[j]-node1.d_av[j]);
      node2.y[j] = node2.y[j] + rho*(node2.d[j]-node1.d_av[j]);
      node3.y[j] = node3.y[j] + rho*(node3.d[j]-node3.d_av[j]);
    }

  }

  for(int i=0;i<3;i++){
    d[i] = node1.d_av[i];
    cout<<" d[ "<<i<<"]= "<<d[i]<<endl;
  }
  //mulitiplicar K*d+o
  l[0] = k11*node1.d_av[0] + k12*node1.d_av[1] + k31*node1.d_av[2] + o1;
  l[1] = k21*node1.d_av[0] + k22*node1.d_av[1] + k23*node1.d_av[2] + o2;
  l[2] = k31*node1.d_av[0] + k32*node1.d_av[1] + k33*node1.d_av[2] + o3;
   for(int i=0;i<3;i++){
    cout<<" l[ "<<i<<"]= "<<l[i]<<endl;
  }
 
return 0;
}
