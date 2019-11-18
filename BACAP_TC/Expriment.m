clc;clear;close all;

%% Choose the scale of problem and BAP Initialization
Small = 0;
Middle = 0;
Large = 1;

if Small == 1
    B = 10;
    Q = 8;
    N = 7; 
    I = 8;
elseif Middle == 1
    B = 15;
    Q = 14;
    N = 11; 
    I = 15;
else
    B = 20;
    Q = 14;
    N = 18;
    I = 40;
end

% duration of tide
V=6;

% total time 
T = 2 * V * N;
% every kinds of vessels account for 1/3
a = round(rand(1,I) * T * 2 /3);

% feeder 1-3 medium 4-6 jumbo 7-8
l_1 = round(rand(1,round(I/3)) * 2) + 1;
l_2 = round(rand(1,I - 2 * round(I/3)) * 2) + 4;
l_3 = round(rand(1,round(I/3))) + 7;
l = [l_1, l_2, l_3];

% workload, feeder 5-15 medium 15-30 jumbo 30-45
c_1 = round(rand(1,round(I/3)) * 10) + 5;
c_2 = round(rand(1,I - 2 * round(I/3)) * 15) + 15;
c_3 = round(rand(1,round(I/3)) * 15) + 30;
c = [c_1, c_2, c_3];

% tide impact
tide = round(I/2):I;
free = 1:round(I/2)-1;

% passing channel time
C = 1;
% large number
M = 1000;

% the min and max number of QCs
q_min_1 = ones(1,round(I/3));
q_max_1 = q_min_1 + 1;
q_min_2 = ones(1,I - 2 * round(I/3)) * 2;
q_max_2 = q_min_2 + 2;
q_min_3 = ones(1,round(I/3)) * 4;
q_max_3 = q_min_3 + 2;
q_min = [q_min_1, q_min_2, q_min_3];
q_max = [q_max_1, q_max_2, q_max_3];

%% GA Initialization
POP_NUM = 100;
CHROM_SIZE = [3 I];
ITER_NUM_GA = 1000;
CROSSOVER_RATE = 0.7;
SELECT_RATE = 0.5;
MUTATION_RATE = 0.5;
Visual = 0;

%% Simulate Anneal Initialization
Temperature = 50000;
T_end = 1e-8;
L = 100;
d = 0.95;
SOL_SIZE = [3 I];

%% Particle Swarm Optimization Initialization
PARTICLE_NUM = 100;
ITER_NUM_PSO = 100;
C1 = 1.5;
C2 = 1.5;
W = 0.8;
PARTICLE_SIZE = [3 I];
Visual = 0;

%% Start GA
disp('----------------Start Genetic Algorithm---------------------')
tic;
[OBJ_GA, RESULT_GA,dist_t1_GA, dist_t2_GA, dist_t3_GA, t_out_GA, rect_GA, remain_sum_GA, remain_time_GA] ...
    = GeneticAlgorithm(POP_NUM, CHROM_SIZE,ITER_NUM_GA,CROSSOVER_RATE,...
                       SELECT_RATE,MUTATION_RATE,Visual,a,B,l,c,Q,tide,...
                       free,I,V,N,T,C,M,q_min,q_max);
disp('Genetic Algorithm Running Time:')
toc;                   

%% Start SA
disp('----------------Start Simulate Anneal Algorithm---------------------')
tic;
[OBJ_SA, RESULT_SA,dist_t1_SA, dist_t2_SA, dist_t3_SA, t_out_SA, rect_SA, remain_sum_SA, remain_time_SA] ...
    = SimulateAnnealAlgorithm(Temperature, T_end, L, d, SOL_SIZE,...
                              Visual, a,B,l,c,Q,tide,free,I,V,N,T,C,M,...
                              q_min,q_max);
disp('Simulate Anneal Algorithm Running Time:')
toc;

%% Start PSO
disp('----------------Start Particle Swarm Optimization Algorithm---------------------')
tic;
[OBJ_PSO, RESULT_PSO,dist_t1_PSO, dist_t2_PSO, dist_t3_PSO, t_out_PSO, rect_PSO, remain_sum_PSO, remain_time_PSO] ...
    = ParticleSwarmOptimizationAlgorithm(PARTICLE_NUM, PARTICLE_SIZE,ITER_NUM_PSO,C1,...
                                         C2, W, Visual,a,B,l,c,Q,tide,...
                                         free,I,V,N,T,C,M,q_min,q_max);
disp('Particle Swarm Optimization Algorithm Running Time:')
toc;
                          
%% Start CPLEX or Gurobi Solver
disp('----------------Start Solver---------------------')
if Small == 1
    tic;
    [OBJ_Solver, solution_Solver] = Solver(a,B,l,c,Q,tide,free,I,V,N,T,C,M,q_min,q_max);
    disp('Solver Running Time:')
    toc;
end

