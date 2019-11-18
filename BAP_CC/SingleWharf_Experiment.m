clc;clear;close all;
%% 单码头参数输入

T = 200;   % 最大规划时间
L = 800;   % 岸线长度
I = 30;    %船只数量

% 到港时间
a = round(rand(1,I) * T * 0.5);

% feeder 50-150 medium 200-300 jumbo 350-00
l_1 = round(rand(1,round(I/3)) * 100) + 50;
l_2 = round(rand(1,I - 2 * round(I/3)) * 100) + 100;
l_3 = round(rand(1,round(I/3)) * 50 ) + 300;
l = [l_1, l_2, l_3];

% workload feeder 300-1000   midium 1000-1500  jumbo 1500-2200
H_1 = round(rand(1,round(I/3)) * 700) + 300;
H_2 = round(rand(1,I - 2 * round(I/3)) * 500) + 1000;
H_3 = round(rand(1,round(I/3)) * 700) + 1500;
H = [H_1, H_2, H_3] / 150;

num = length(a);
M = 100000;     % 惩罚因子
t_out = 0.75;   % 出港时间
t_in = 0.5;     % 进港时间
max_wait = 80;  % 最大等待时间


temp = (5*num+1):(5*num+num*num);
X_index = reshape(temp,num,num);
temp = (5*num+1+num*num):(5*num+num*num+num*num);
Y_index = reshape(temp,num,num);
temp = (5*num+1+num*num+num*num):3*num*num+5*num;
Z_index = reshape(temp,num,num);
%-------------------------------目标函数输入--------------------------------------------------
obj = zeros(1,3*num*num + 5*num);
obj(1,4*num+1:5*num) = 1;
%obj(1,num+1:2*num) = 1;
%-------------------------------不等式约束输入------------------------------------------
%A = zeros(5.5*num*num-0.5*num,3*num*num+5*num);
%b = zeros(5.5*num*num-0.5*num,1);
A = spalloc(3000,3*num*num+5*num,300000);
b = zeros(3000,1);
count = 1;
% t_i_out <= T-t_out
for i = num*4+1:num*5 
    A(count,i) = 1;
    b(count,1) = T - t_out;
	count = count + 1;
end
% -t_i_in <= -a_i
for i = num+1:2*num
   A(count,i) = -1;
   b(count,1) = -a(i-num);
   count = count + 1;
end
% t_i_in <= a_i + max_wait 设置最大等待时间 效果不明显
for i = num+1:2*num
   A(count,i) = 1;
   b(count,1) = a(i-num)+ max_wait;
   count = count + 1;
end
% t_i_end - t_i_out <=0
for i = 4*num+1:5*num
   A(count,i) = -1;
   A(count,i-num) = 1;
   count = count + 1;
end
% b_i <=L-l_i
for i = 1:num
   A(count,i) = 1;
   b(count,1) = L - l(i);
   count = count + 1;
end
% Xij+Xji <= 1
for i = 1:num
   for j = i+1:num
       A(count,X_index(i,j)) = 1;
       A(count,X_index(j,i)) = 1;
       b(count,1) = 1;
       count = count + 1;
   end
end
% Yij + Yji <= 1
for i = 1:num
    for j = i+1:num
        A(count,Y_index(i,j)) = 1;
        A(count,Y_index(j,i)) = 1;
        b(count,1) = 1;
        count = count + 1;
    end
end
% Xij+Xji+Yij+Yji >= 1
for i = 1:num
    for j = i+1:num
        A(count,X_index(i,j)) = -1;
        A(count,X_index(j,i)) = -1;
        A(count,Y_index(i,j)) = -1;
        A(count,Y_index(j,i)) = -1;
        b(count,1) = -1;
        count = count + 1;
    end
end

% b_i - b_j + LYij <= L - l_i
for i = 1:num
    for j = 1:num
        if (i == j)
            continue; 
        end
        A(count,i) = 1;
        A(count,j) = -1;
        A(count,Y_index(i,j)) = L;
        b(count,1) = L-l(i);
        count = count + 1;
    end
end
% t_i_start - t_j_start + TXij <= T - H_i
for i = 1:num
    for j = 1:num
        if (i == j)
            continue; 
        end
        A(count,i + 2*num) = 1;
        A(count,j + 2*num) = -1;
        A(count,X_index(i,j)) = T;
        b(count,1) = T-H(i);
        count = count + 1;
    end
end
% 不等式约束6，或语句要用惩罚因子表示
for i = 1:num
    for j = 1:num
        if (i == j)
            continue; 
        end
        A(count,i + 4*num) = -1;
        A(count,j + num) = 1;
        A(count,Z_index(i,j)) = -M;
        b(count,1) = -t_in;
        count = count + 1;
    end
end
for i = 1:num
    for j = 1:num
        if (i == j)
            continue; 
        end
        A(count,i + 4*num) = 1;
        A(count,j + num) = -1;
        A(count,Z_index(i,j)) = M;
        b(count,1) = M - t_out;
        count = count + 1;
    end
end
%---------------------------------------等式约束输入---------------------------
Aeq = zeros(2*num,3*num*num+5*num);
beq = zeros(2*num,1);
count2 = 1;
% t_i_start - t_i_in = t_in
for i = 1:num
    Aeq(count2,i+2*num) = 1;
    Aeq(count2,i+num) = -1;
    beq(count2,1) = t_in;
    count2 = count2 + 1;
end
% t_i_end - t_i_start = H_i
for i = 1:num
    Aeq(count2,i+3*num) = 1;
    Aeq(count2,i+2*num) = -1;
    beq(count2,1) = H(i);
    count2 = count2 + 1;
end
% -------------------------------决策变量上下限-------------------------------
lb = zeros(1,3*num*num+5*num);
ub = zeros(1,3*num*num+5*num);
ub(5*num+1:3*num*num+5*num) = 1;   % 0-1变量约束
ub(1:5*num) = inf;                 
intcon = 5*num+1:3*num*num+5*num;  % 整数约束

%求解
tic;
options = optimoptions('intlinprog');
[x,fval,exitflag,output] = intlinprog(obj,intcon,A,b,Aeq,beq,lb,ub,options);
toc;
%--------------------------------结果输出-----------------------------------------
position = x(1:num);
t_i_in = x(num+1:num*2);
t_i_start = x(num*2+1:num*3);
t_i_end = x(num*3+1:num*4);
t_i_out = x(num*4+1:num*5);

MinTime = max(t_i_out) + t_out - min(a);

figure;
xlabel('Wharf/Meter');
ylabel('Time/Hour');
title('BAP-Channel Constraint and Single Wharf')
axis([0 L 0 max(t_i_out) * 1.5]);

for i = 1:num
    hold on;
    rectangle('Position',[position(i),t_i_start(i),l(i),H(i)],'EdgeColor','black','LineWidth',2); 
    text(position(i)+l(i)/2,t_i_start(i)+H(i)/2,{num2str(i)},'Fontsize',20);
    text(position(i)+l(i) * 0.8, t_i_start(i)+H(i)/2,{ num2str(a(i)), num2str(round(t_i_in(i)-a(i)))});
end

figure;
xlabel('Time/Hour');
ylabel('Channel');
title('Channel Situation')

for i = 1:num
   hold on;
   rectangle('Position',[t_i_in(i),0,t_in,0],'EdgeColor','blue','LineWidth',2);
   rectangle('Position',[t_i_out(i),0,t_out,0],'EdgeColor','red','LineWidth',2);
end