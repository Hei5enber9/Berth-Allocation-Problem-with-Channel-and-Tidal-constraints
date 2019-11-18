function pop_crossover = Crossover(pop, POP_NUM, CROSSOVER_RATE)
current_size = size(pop,1);
iter = 1;
while  iter <= POP_NUM - current_size
    father = reshape(pop(ceil(current_size * rand()),:,:),size(pop,2),size(pop,3));
    mother = reshape(pop(ceil(current_size * rand()),:,:),size(pop,2),size(pop,3));
    pos1 = unidrnd(size(pop,3));
    pos2 = unidrnd(size(pop,3));
    if pos1 >= pos2
        tmp = pos1;
        pos1 = pos2;
        pos2 = tmp; 
    end
    
    offspring1 = zeros(size(pop,2),size(pop,3));
    offspring2 = zeros(size(pop,2),size(pop,3));
    
    index_father = [];
    index_mother = [];
    
    for i = pos1:pos2
        offspring1(:,i) = father(:,i);
        offspring2(:,i) = mother(:,i);
        index_father = [index_father father(1,i)];
        index_mother = [index_mother mother(1,i)];
    end
    
    k=1;
    for i = 1:size(pop,3)
        if k == size(pop,3) + 1
            break;
        end
        while k>= pos1 && k<=pos2
            k = k+1;
        end
        if ismember(mother(1,i),index_father)
            continue;
        end
        offspring1(:,k) = mother(:,i);
        k = k + 1;
    end
    
    k=1;
    for i = 1:size(pop,3)
        if k == size(pop,3) + 1
            break;
        end
        while k>= pos1 && k<=pos2
            k = k+1;
        end
        if ismember(father(1,i),index_mother)
            continue;
        end
        offspring2(:,k) = father(:,i);
        k = k + 1;
    end
    
    pop_crossover(iter,:,:) = offspring1;
    iter = iter + 1;
    
    if iter <= POP_NUM - current_size
        pop_crossover(iter,:,:) = offspring2;
        iter = iter + 1;
    end 
end