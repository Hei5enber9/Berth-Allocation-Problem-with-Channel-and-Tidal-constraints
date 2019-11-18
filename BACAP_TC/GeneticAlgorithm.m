function [OBJ, RESULT,dist_t1, dist_t2, dist_t3, t_out, ...
          rect, remain_sum, remain_time] ...
    = GeneticAlgorithm(POP_NUM, CHROM_SIZE,ITER_NUM,CROSSOVER_RATE,...
                       SELECT_RATE,MUTATION_RATE,Visual,a,B,l,c,Q,tide,...
                       free,I,V,N,T,C,M,q_min,q_max)

 %% Initial Population
pop = InitPop(POP_NUM,CHROM_SIZE,B,c,q_max,q_min,l);

best_value = zeros(ITER_NUM,1);
best_individual = zeros(ITER_NUM,CHROM_SIZE(1),CHROM_SIZE(2));
BEST = 0;
BEST_CHROM = zeros(CHROM_SIZE);

%% Start Iteration
for iter = 1:ITER_NUM
    if mod(iter,100) == 0
        disp(['iteration:',num2str(iter)]);
    end
    
    [fitness, ~, ~, ~, ~, ~, ~, ~] ...
        = Fitness(pop,tide,free,a,l,B,M,N,V,T,C,c,Q,q_min,q_max,Visual,'GA');
    best_value(iter) = max(fitness);
    index = find(fitness == best_value(iter));
    best_individual(iter,:,:) = pop(index(1),:,:);
    if best_value(iter) > BEST
        BEST = best_value(iter);
        index = find(fitness==BEST);
        BEST_CHROM = reshape(pop(index(1),:,:),size(pop,2),size(pop,3));
    end
    
    pop_select = Select(fitness, pop, SELECT_RATE);
    pop_crossover = Crossover(pop_select, POP_NUM, CROSSOVER_RATE);
    pop_mutation = Mutation(pop_crossover,MUTATION_RATE,l,B,q_max,q_min,c);
    pop = cat(1,pop_mutation,pop_select);
end

BEST_CHROM = reshape(BEST_CHROM, 1, CHROM_SIZE(1), CHROM_SIZE(2));
[RESULT,dist_t1, dist_t2, dist_t3, t_out, rect, remain_sum, remain_time] ...
    = Fitness(BEST_CHROM,tide,free,a,l,B,M,N,V,T,C,c,Q,q_min,q_max,1,'GA');
OBJ = 1 / BEST;

%% Visualization of Iteration
gcf = figure;
set(gcf,'Position',[0 0 12000 6762]);
xlabel('Generation');
ylabel('Best Fitness of Current Generation');
title('Fitness Curve/GA');
grid on;
hold on;
plot(best_value,'LineWidth',2);
figure_name = ['Experiment/GA_Curve_' num2str(I)];
print(gcf,'-djpeg','-r600',figure_name);
end