function [OBJ, RESULT,dist_t1, dist_t2, dist_t3, t_out, ...
          rect, remain_sum, remain_time] ...
    = SimulateAnnealAlgorithm(Temperature, T_end, L, d, SOL_SIZE,...
                              Visual,a,B,l,c,Q,tide,free,I,V,N,T,C,M,...
                              q_min,q_max)

% Initial solution
sol = zeros(SOL_SIZE);
sol(1,:) = randperm(SOL_SIZE(2));
    for j = 1:SOL_SIZE(2)
        sol(2,j) = round(rand()*(B-l(sol(1,j))));
        tmp2 = ceil(c ./ q_max);
        tmp3 = floor(c ./ q_min);
        sol(3,j) = round(rand()*(tmp3(sol(1,j))-tmp2(sol(1,j)))) + tmp2(sol(1,j));
    end
sol = reshape(sol,1,SOL_SIZE(1),SOL_SIZE(2));
    
iter = 0;
t = Temperature;
best_value = [];
BEST_VALUE = 0;
BEST_SOL = zeros(SOL_SIZE);

while t >= T_end
    for i = 1:L
        sol_new = CreateNewSol(sol,l,B,q_max,q_min,c);
        [fitness1, ~, ~, ~, ~, ~, ~, ~] ...
              = Fitness(sol,tide,free,a,l,B,M,N,V,T,C,c,Q,q_min,q_max,Visual,'SA');
        [fitness2, ~, ~, ~, ~, ~, ~, ~] ...
              = Fitness(sol_new,tide,free,a,l,B,M,N,V,T,C,c,Q,q_min,q_max,Visual,'SA');
        delta = fitness2 - fitness1;
        result = fitness1;
        if delta > 0
            sol = sol_new;
            result = fitness2;
        elseif exp(-delta/Temperature) > rand()
            sol = sol_new;
            result = fitness2;
        end
        
        if result > BEST_VALUE
            BEST_VALUE = result;
            BEST_SOL = sol;
        end
    end
    
    iter = iter + 1;
    best_value(iter) = result; 
    
    t = t * d;
    
    if mod(iter,100) == 0
        disp(['iteration:',num2str(iter),'  Temperature:', num2str(t)]);
    end
end

BEST_SOL = reshape(BEST_SOL, 1, SOL_SIZE(1), SOL_SIZE(2));
[RESULT,dist_t1, dist_t2, dist_t3, t_out, rect, remain_sum, remain_time] ...
    = Fitness(BEST_SOL,tide,free,a,l,B,M,N,V,T,C,c,Q,q_min,q_max,1,'SA');
OBJ = 1 / BEST_VALUE;

%% Visualization of Iteration
gcf = figure;
set(gcf,'Position',[0 0 12000 6762]);
xlabel('Iteration');
ylabel('Best Fitness of Current Iteration');
title('Fitness Curve/SA');
grid on;
hold on;
plot(best_value,'LineWidth',2);
figure_name = ['Experiment/SA_Curve_' num2str(I)];
print(gcf,'-djpeg','-r600',figure_name);
                   
end