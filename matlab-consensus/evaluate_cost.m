%FUNCTION TO COMPUTE THE AUGMENTED LAGRANGIAN COST AT A POSSIBLE SOLUTION
%USED BY CONSENSUS_ITERATE
function cost = evaluate_cost(node,d,rho)
    cost =  node.c'*d + node.y'*(d-node.d_av) + ...
            rho/2*norm(d-node.d_av)^2;
end