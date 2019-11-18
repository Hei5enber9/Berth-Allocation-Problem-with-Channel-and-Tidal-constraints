function pop_mutation = Mutation(pop,MUTATION_RATE,l,B,q_max,q_min,c)
for i=1:size(pop,1)
    if rand() <= MUTATION_RATE
        individual = reshape(pop(i,:,:),size(pop,2),size(pop,3));
        
        % first row operation
        pos1 = unidrnd(size(pop,3));
        pos2 = unidrnd(size(pop,3));
        temp2 = individual(1,pos1);
        individual(1,pos1)=individual(1,pos2);
        individual(1,pos2)=temp2;
        
        % second row operation
        individual(2,pos1)=round(rand()*(B-l(individual(1,pos1))));
        individual(2,pos2)=round(rand()*(B-l(individual(1,pos2))));
        
        % third row operation
        tmp2 = ceil(c ./ q_max);
        tmp3 = floor(c ./ q_min);
        individual(3,pos1)=round(rand()*(tmp3(individual(1,pos1))-tmp2(individual(1,pos1)))) + tmp2(individual(1,pos1));
        individual(3,pos2)=round(rand()*(tmp3(individual(1,pos2))-tmp2(individual(1,pos2)))) + tmp2(individual(1,pos2));
        
        pop(i,:,:) = individual;
    end
end
pop_mutation = pop;
end