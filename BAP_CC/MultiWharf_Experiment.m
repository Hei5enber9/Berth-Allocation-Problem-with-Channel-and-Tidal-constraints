clc;clear;close all;
%% 多码头参数输入

T = 100;   % 最大规划时间
L = 1800;   % 岸线长度
I = 40;    %船只数量

FREE = 1;

% 到港时间
a = round(rand(1,I) * T * 0.5);

% 不再限制了，为了鲁棒性，小船也可能很多货运
% feeder 50-150 medium 200-300 jumbo 350-00
l_1 = round(rand(1,round(I/3)) * 100) + 50;
l_2 = round(rand(1,I - 2 * round(I/3)) * 100) + 100;
l_3 = round(rand(1,round(I/3)) * 50 ) + 300;
l = [l_1, l_2, l_3];
l = l(randperm(I));

% workload feeder 300-1000   midium 1000-1500  jumbo 1500-2200
H_1 = round(rand(1,round(I/3)) * 700) + 300;
H_2 = round(rand(1,I - 2 * round(I/3)) * 500) + 1000;
H_3 = round(rand(1,round(I/3)) * 700) + 1500;
H = [H_1, H_2, H_3] / 150;
H = H(randperm(I));

num = length(a);

if FREE == 1
    num1 = round(num/4);             %第一码头的船只 1~num1
    num2 = num1 + round(num/4);      %第二码头的船只 num1+1~num2
    num3 = num2 + round(num/4);      %第三码头的船只 num2+1~num3
    num4 = num;                      % 无约束
else
    num1 = round(num/3);
    num2 = num1 + round(num/3);      
    num3 = num; 
end

len1 = 600;    %第一码头岸线 0~len1
len2 = 1200;    %第二码头的岸线 len1~len2
len3 = L;       %第三码头的船只 len2~len3


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
obj(1,num+1:2*num) = 2;
%-------------------------------不等式约束输入------------------------------------------
A = zeros(5.5*num*num+0.5*num,3*num*num+5*num);
b = zeros(5.5*num*num+0.5*num,1);
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
% b_i <=L-l_i 多码头规划
% 第一码头0~1000
for i = 1:num1
   A(count,i) = 1;
   b(count,1) = len1 - l(i);
   count = count + 1;
end
for i = 1:num1
   A(count,i) = -1;
   b(count,1) = 0;
   count = count + 1;
end
% 第二码头1000~1800
for i = num1+1:num2
   A(count,i) = 1;
   b(count,1) = len2 - l(i);
   count = count + 1;
end
for i = num1+1:num2
   A(count,i) = -1;
   b(count,1) = -len1;
   count = count + 1;
end
% 第三码头1800~2400
for i = num2+1:num3
   A(count,i) = 1;
   b(count,1) = len3 - l(i);
   count = count + 1;
end
for i = num2+1:num3
   A(count,i) = -1;
   b(count,1) = -len2;
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
%不等式约束6
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
options = optimoptions('intlinprog','MaxTime',270);
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
ylabel('Time/Hour');
title('BAP-Channel Constraint and Multi-Wharf')
axis([0 L 0 max(t_i_out) * 1.5]);
text(0 + len1/2,-3 ,{'Wharf 1'},'FontSize',15, 'LineWidth',7);
text((len1 + len2)/2, -3 ,{'Wharf 2'},'FontSize',15, 'LineWidth',7);
text((len2 + len3)/2, -3 ,{'Wharf 3'},'FontSize',15, 'LineWidth',7);

rectangle('Position',[0,0,len1,1],'EdgeColor','red','LineWidth',2,'FaceColor','red'); 
rectangle('Position',[len1,0,len2,1],'EdgeColor','green','LineWidth',2,'FaceColor','green'); 
rectangle('Position',[len2,0,len3,1],'EdgeColor','blue','LineWidth',2,'FaceColor','blue'); 
for i = 1:num1
    hold on;
    rectangle('Position',[position(i),t_i_start(i),l(i),H(i)],'EdgeColor','red','LineWidth',2);
    wait_time = t_i_in(i)-a(i);
    if (wait_time <= 0.01) 
       wait_time = 0; 
    end
    text(position(i)+l(i)/2,t_i_start(i)+H(i)/2,[num2str(i) ],'FontSize',20);
    text(position(i)+l(i) * 0.8, t_i_start(i)+H(i)/2,{ num2str(a(i)), num2str(round(wait_time))});
    
end
for i = num1+1:num2
    hold on;
    rectangle('Position',[position(i),t_i_start(i),l(i),H(i)],'EdgeColor','green','LineWidth',2); 
    wait_time = t_i_in(i)-a(i);
    if (wait_time <= 0.01) 
       wait_time = 0; 
    end
    text(position(i)+l(i)/2,t_i_start(i)+H(i)/2,{num2str(i)},'FontSize',20);
    text(position(i)+l(i) * 0.8, t_i_start(i)+H(i)/2,{ num2str(a(i)), num2str(round(wait_time))});
end
for i = num2+1:num3
    hold on;
    rectangle('Position',[position(i),t_i_start(i),l(i),H(i)],'EdgeColor','blue','LineWidth',2); 
    wait_time = t_i_in(i)-a(i);
    if (wait_time <= 0.01) 
       wait_time = 0; 
    end
    text(position(i)+l(i)/2,t_i_start(i)+H(i)/2,{num2str(i)},'FontSize',20);
    text(position(i)+l(i) * 0.8, t_i_start(i)+H(i)/2,{ num2str(a(i)), num2str(round(wait_time))});
end
for i = num3+1:num4
    hold on;
    rectangle('Position',[position(i),t_i_start(i),l(i),H(i)],'EdgeColor','black','LineWidth',2); 
    wait_time = t_i_in(i)-a(i);
    if (wait_time <= 0.01) 
       wait_time = 0; 
    end
    text(position(i)+l(i)/2,t_i_start(i)+H(i)/2,{num2str(i)},'FontSize',20);
    text(position(i)+l(i) * 0.8, t_i_start(i)+H(i)/2,{ num2str(a(i)), num2str(round(wait_time))});
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