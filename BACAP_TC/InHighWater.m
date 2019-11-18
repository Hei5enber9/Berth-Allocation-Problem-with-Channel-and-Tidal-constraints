function [earliest_time, result] = InHighWater(t,V,N)
result=0;
earliest_time = 0;

% boundary process
if t >= 2*V*N
    earliest_time = 2*V*N;
%return the boolen value and the next high water time    
else
    for j = 0:N-1
       if  t >= 1 + j * 2 * V && t <= j * 2 * V + V
           result = 1;
           break;
       elseif t > j * 2 * V + V && t < 2 * V + 1 + 2 * j *V
           result = 0;
           earliest_time = 2 * V + 1 + 2 * j *V;
           break;
       end
    end
end    
end