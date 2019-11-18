function [OBJ, RESULT,dist_t1, dist_t2, dist_t3, t_out, ...
          rect, remain_sum, remain_time] ...
    = ParticleSwarmOptimizationAlgorithm(PARTICLE_NUM, PARTICLE_SIZE,ITER_NUM,C1,...
                       C2, W, Visual,a,B,l,c,Q,tide,...
                       free,I,V,N,T,C,M,q_min,q_max)

%% Initialization of particles' position and velocity
% attention: the indices need to be the same order
particle_x = InitPop(PARTICLE_NUM, PARTICLE_SIZE, B,c,q_max,q_min,l);
particle_v = InitPop(PARTICLE_NUM, PARTICLE_SIZE, B,c,q_max,q_min,l);
for i = 1:PARTICLE_NUM
    particle_v(i,1,:) = particle_x(i,1,:);
end

% pBest and gBest Initialization
pBest = particle_x;
[pBest_Value, ~, ~, ~, ~, ~, ~, ~] ...
        = Fitness(particle_x,tide,free,a,l,B,M,N,V,T,C,c,Q,q_min,q_max,Visual,'PSO');

[gBest_Value, index] = max(pBest_Value);
gBest = particle_x(index,:,:);

% record the process of iteration
best_value = zeros(ITER_NUM,1);
cnt = 1;

%% Start Iteration
for iter = 1:ITER_NUM
    for i = 1:PARTICLE_NUM
        current_particle_x = reshape(particle_x(i,:,:), PARTICLE_SIZE(1), PARTICLE_SIZE(2));
        current_particle_v = reshape(particle_v(i,:,:), PARTICLE_SIZE(1), PARTICLE_SIZE(2));
        current_gBest = reshape(gBest, PARTICLE_SIZE(1), PARTICLE_SIZE(2));
        current_pBest = reshape(pBest(i,:,:),PARTICLE_SIZE(1), PARTICLE_SIZE(2));
        for j = 1:PARTICLE_SIZE(2)
            % update the position and velocity along the column
            index = current_particle_x(1,j);
            part1 = W * current_particle_v(2:3,j);
            part2 = C1 * rand() * (current_pBest(2:3,find(current_pBest(1,:) == index)) - current_particle_x(2:3,j));
            part3 = C2 * rand() * (current_gBest(2:3,find(current_gBest(1,:) == index)) - current_particle_x(2:3,j));
            tmp = part1 + part2 + part3;
            current_particle_v(2:3,j) = tmp;
            current_particle_x(2:3,j) = current_particle_x(2:3,j) + current_particle_v(2:3,j);
            
            % process the boundary of second row
            if current_particle_x(2,j)  > B - l(index) 
                current_particle_x(2,j) = rem(current_particle_x(2,j),B - l(index));
            elseif current_particle_x(2,j) < 0
                current_particle_x(2,j) = rem(-current_particle_x(2,j),B - l(index));
            end
            current_particle_x(2,j) = floor(current_particle_x(2,j));
            
            % process the boundary of third row
            tmp1 = ceil(c ./ q_max);
            tmp2 = floor(c ./ q_min);
            
            if current_particle_x(3,j)  > tmp2(index) 
                current_particle_x(3,j) = rem(current_particle_x(2,j),tmp2(index));
            end
            if current_particle_x(3,j) < tmp1(index)
                current_particle_x(3,j) = tmp1(index);
            end
            
            current_particle_x(3,j) = floor(current_particle_x(3,j));
        end
        
        % evaluate the new particle 
        current_particle_x = reshape(current_particle_x,1,PARTICLE_SIZE(1),PARTICLE_SIZE(2));
        particle_x(i,:,:) = current_particle_x;
        [new_particle_value, ~, ~, ~, ~, ~, ~, ~] ...
                        = Fitness(current_particle_x,tide,free,a,l,B,M,N,V,T,C,c,Q,q_min,q_max,Visual,'PSO');
        
        % update the pBest and gBest
        if new_particle_value > pBest_Value(i)
            pBest_Value(i) = new_particle_value;
            pBest(i,:,:) = current_particle_x;
        end
        
        if new_particle_value > pBest_Value(i)
            pBest_Value(i) = new_particle_value;
            pBest(i,:,:) = current_particle_x;
        end
        
        if new_particle_value > gBest_Value
            gBest_Value = new_particle_value;
            gBest = current_particle_x;
        end
        
        % recording for visualization
        best_value(cnt) = gBest_Value;
        cnt = cnt + 1;
    end
end

[RESULT,dist_t1, dist_t2, dist_t3, t_out, rect, remain_sum, remain_time] ...
    = Fitness(gBest,tide,free,a,l,B,M,N,V,T,C,c,Q,q_min,q_max,1,'PSO');
OBJ = 1 / RESULT;

%% Visualization of Iteration
gcf = figure;
set(gcf,'Position',[0 0 12000 6762]);
xlabel('Iteration');
ylabel('Best Fitness of Current Iteration');
title('Fitness Curve/PSO');
grid on;
hold on;
plot(best_value,'LineWidth',2);
figure_name = ['Experiment/PSO_Curve_' num2str(I)];
print(gcf,'-djpeg','-r600',figure_name);
end