function pop_select = Select(fitness, pop, SELECT_RATE)
sumP = sum(fitness);
accP = cumsum(fitness/sumP);
for i = 1:round((size(fitness,1) * SELECT_RATE))
    tmp = find(accP>rand());
    if isempty(tmp)
        continue;
    end
    pop_select(i,:,:) = pop(tmp(1),:,:);
end
end