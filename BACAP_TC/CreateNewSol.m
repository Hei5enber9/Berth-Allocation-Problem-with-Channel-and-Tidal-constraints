function sol_new = CreateNewSol(sol,l,B,q_max,q_min,c)

sol_new = reshape(sol,size(sol,2),size(sol,3));
% first row operation
pos1 = unidrnd(size(sol,3));
pos2 = unidrnd(size(sol,3));
temp2 = sol_new(1,pos1);
sol_new(1,pos1)=sol_new(1,pos2);
sol_new(1,pos2)=temp2;

% second row operation
sol_new(2,pos1)=round(rand()*(B-l(sol_new(1,pos1))));
sol_new(2,pos2)=round(rand()*(B-l(sol_new(1,pos2))));

% third row operation
tmp2 = ceil(c ./ q_max);
tmp3 = floor(c ./ q_min);
sol_new(3,pos1)=round(rand()*(tmp3(sol_new(1,pos1))-tmp2(sol_new(1,pos1)))) + tmp2(sol_new(1,pos1));
sol_new(3,pos2)=round(rand()*(tmp3(sol_new(1,pos2))-tmp2(sol_new(1,pos2)))) + tmp2(sol_new(1,pos2));

sol_new = reshape(sol_new, 1, size(sol_new,1), size(sol_new, 2));
end