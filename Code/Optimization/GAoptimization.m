clc
close all

% Define constraints
Aeq = [];
beq = [];
A =[1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
b = [1;1];
LB = zeros(1, 21); % Lower bounds
UB = 10000*ones(1, 21);  % Upper bounds

% Initial guess
x0 = ones(1,21);

% GA options
options = gaoptimset('PlotFcns', @gaplotbestf, ...
    'UseParallel', 0, ...
    'PopulationSize', 21, ...
    'Generations', 6, ...
    'TolFun', 1e-2, ...
    'EliteCount', 2, ...
    'InitialPopulation', x0);

% Run genetic algorithm
[x_total, fval, exitflag] = ga(@CostFun, length(x0), A, b, Aeq, beq, LB, UB, [], options);

% Extract variables

e_optimal = x_total(1:7);   % balance distribution coefficient(BDC)
K_optimal =  x_total(8:21); % gains of gait controller 


% Save results
save('Result_Optimization');

% Display elapsed time
elapsed_time = toc;
fprintf('Elapsed time: %.2f seconds\n', elapsed_time);