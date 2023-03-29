%FUNCTION TO CHECK SOLUTION FEASIBILITY
function check = check_feasibility(node, d)
   tol = 0.001; %%tolerance for rounding errors
   if (d(node.index) < 0-tol), check = 0; return; end;
   if (d(node.index) > 100+tol), check = 0; return; end;
   if (d'*node.k < node.L-node.o-tol), check = 0; return; end;
   check = 1;
end