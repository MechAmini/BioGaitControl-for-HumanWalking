function Etotal=CostFun(xd)

e_optimal = xd(1:7);   % balance distribution coefficient(BDC)
K_optimal =  xd(8:21); % gains of gait controller 

Etotal  = ErrorOptimize(e_optimal,K_optimal);

end
