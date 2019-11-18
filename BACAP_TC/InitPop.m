function pop = InitPop(POP_NUM,CHROM_SIZE,B,c,q_max,q_min,l)
pop = zeros(POP_NUM,CHROM_SIZE(1),CHROM_SIZE(2));
for i = 1:POP_NUM
    pop_tmp = zeros(CHROM_SIZE);
    pop_tmp(1,:) = randperm(CHROM_SIZE(2));
    for j = 1:CHROM_SIZE(2)
        pop_tmp(2,j) = round(rand()*(B-l(pop_tmp(1,j))));
        tmp2 = ceil(c ./ q_max);
        tmp3 = floor(c ./ q_min);
        pop_tmp(3,j) = round(rand()*(tmp3(pop_tmp(1,j))-tmp2(pop_tmp(1,j)))) + tmp2(pop_tmp(1,j));
    end
    pop(i,:,:) = pop_tmp;
end
end