function [OBJ, solution] = Solver(a,B,l,c,Q,tide,free,I,V,N,T,C,M,q_min,q_max)

prob = optimproblem;

%% Define the Decision and Auxiliary Variables
tic;
% time of passing through channel, including cons 2 
lamda_i = optimvar('lamda_i',1, I, 'Type', 'integer', 'LowerBound', a);

% time of beginning processinig
epsilon_i = optimvar('epsilon_i', 1, I, 'Type', 'integer', 'LowerBound', 1);

% processing time
rho_i = optimvar('rho_i', 1, I, 'Type', 'integer', 'LowerBound', 1);

% time of ending processing
sigma_i = optimvar('sigma_i', 1, I, 'Type', 'integer', 'LowerBound', 1);

% time of departuring berth 
psi_i = optimvar('psi_i', 1, I, 'Type', 'integer', 'LowerBound', 1);

% time of departuring port after passing through channel
ksi_i = optimvar('ksi_i', 1, I, 'Type', 'integer', 'LowerBound', 1);

% the berth segment occupied by the stem of vessels
% attention that the correspongding stem  coordinate = beta_i - 1  
beta_i = optimvar('beta_i', 1, I, 'Type', 'integer', 'LowerBound', 1);

% binvar, equal 1 if QC q is assigned to vessel i in time step t
eta_qit = optimvar('eta_qit', Q, I, T, 'Type', 'integer', 'LowerBound', 0, 'UpperBound', 1);

% binvar, equal 1 if vessel i starts processng at time step t
pi_it = optimvar('pi_it', I, T, 'Type', 'integer', 'LowerBound', 0, 'UpperBound', 1);

% binvar equal 1 if vessel i finishes processing at time step t
phi_it = optimvar('phi_it', I, T, 'Type', 'integer', 'LowerBound',0, 'UpperBound', 1);

% binvar equal 1 if vessel i is processing in time step t
theta_it = optimvar('theta_it', I, T, 'Type', 'integer', 'LowerBound', 0, 'UpperBound', 1);

% binvar, equal 1 if the vi moors segment b in time step t
w_bit = optimvar('w_bit', B, I, T, 'Type', 'integer', 'LowerBound', 0, 'UpperBound', 1);

% binvar, equal 1 if vi moors below vi'
delta_ii = optimvar('delta_it', I, I, 'Type', 'integer', 'LowerBound', 0, 'UpperBound', 1);

% binvar, equal 1 if vi enters the approached channel in nth high water
u_in = optimvar('u_in', I, N, 'Type', 'integer', 'LowerBound', 0, 'UpperBound', 1);

% binvar, equal 1 if vi enters the approached channel in nth high water
v_in = optimvar('v_in', I, N, 'Type', 'integer', 'LowerBound', 0, 'UpperBound', 1);

%% Objective Function and Constraints
% objective function
prob.Objective = sum(ksi_i-a);

% constraints3
prob.Constraints.cons1 = epsilon_i == lamda_i + C;

% constraints4
prob.Constraints.cons2 = psi_i >= sigma_i;

% constraints5
prob.Constraints.cons3 = ksi_i == psi_i + C;

% constraints6
prob.Constraints.cons4 = sum(u_in(tide,:),2)==1;

% constraints8
prob.Constraints.cons5 = sum(v_in(tide,:),2)==1;

% constraints7
prob.Constraints.cons6 = 2*repmat(0:N-1,length(tide),1)*V+1-M*(1-u_in(tide,:))<=repmat(lamda_i(tide)',1,N);
prob.Constraints.cons7 = repmat(2*(1:N)-1,length(tide),1)*V+M*(1-u_in(tide,:))>=repmat(lamda_i(tide)',1,N);

% constraints9
prob.Constraints.cons8 = 2*repmat(0:N-1,length(tide),1)*V+1-M*(1-v_in(tide,:))<=repmat(psi_i(tide)',1,N);
prob.Constraints.cons9 = repmat(2*(1:N)-1,length(tide),1)*V+M*(1-v_in(tide,:))>=repmat(psi_i(tide)',1,N);

prob.Constraints.cons10 = sum(pi_it,2) == 1;
disp('constraint 1~10 Done');

prob.Constraints.cons11 = sum(phi_it,2) == 1;

prob.Constraints.cons12 = sum(pi_it.*repmat(1:T,I,1),2)==epsilon_i';

prob.Constraints.cons13 = sum(theta_it,2)==rho_i';

prob.Constraints.cons14 = sum(phi_it.*repmat(1:T,I,1),2)==sigma_i';

prob.Constraints.cons15 = sigma_i == epsilon_i + rho_i;

cons16 = optimconstr(I,T-1);
for t=1:T-1
    cons16(:,t)=sum(theta_it(:,1:t),2)<=M*(1-pi_it(:,t+1));
end
prob.Constraints.cons16 = cons16;

cons17 = optimconstr(I,T-1);
for t=1:T-1
    cons17(:,t)=sum(theta_it(:,t+1:T),2)<=M*(1-phi_it(:,t));
end
prob.Constraints.cons17 = cons17;

prob.Constraints.cons18 = theta_it>=1-M*(1-pi_it);

cons19 = optimconstr(I,T-1,T-2);
for t=2:T
    for t2=t+1:T
        cons19(:,t-1,t2-2)=theta_it(:,t2)+theta_it(:,t-1)-theta_it(:,t)<=1;
    end
end
prob.Constraints.cons19 = cons19;

prob.Constraints.cons20 = squeeze(sum(w_bit,1))==theta_it;
disp('constraint 1~20 Done');

prob.Constraints.cons21 = repmat(beta_i',1,T)>=squeeze(sum(repmat((1:B)',1,I,T).*w_bit,1))-M*(1-theta_it);

prob.Constraints.cons22 = repmat(beta_i',1,T)<=squeeze(sum(repmat((1:B)',1,I,T).*w_bit,1))+M*(1-theta_it);

prob.Constraints.cons23 = w_bit(:,:,2:end)-w_bit(:,:,1:end-1)>=-M*repmat(reshape((2-theta_it(:,2:end)-theta_it(:,1:end-1)),1,I,T-1),B,1,1);

prob.Constraints.cons24 = w_bit(:,:,2:end)-w_bit(:,:,1:end-1)<=M*repmat(reshape((2-theta_it(:,2:end)-theta_it(:,1:end-1)),1,I,T-1),B,1,1);

prob.Constraints.cons25 = repmat((beta_i+l-1)',1,T)<=B+M*(1-theta_it);

prob.Constraints.cons26 = sum(theta_it.*repmat(l',1,T),1)<=B;

cons27 = optimconstr(I,I,T,B-min(l)+1);
for i=1:I
    for i2=1:I
        if i~=i2
            for t=1:T
                for b=l(i2):B-l(i)+1
                    cons27(i,i2,t,b)=sum(w_bit(b-l(i2)+1:b+l(i)-1,i2,t),1)<=M*(1-w_bit(b,i,t));
                end
            end
        end
    end
end
prob.Constraints.cons27 = cons27;
disp('constraint 27 Done');

cons28 = optimconstr(I,I);
for i=1:I
    for i2=1:I
        if i~=i2
            cons28(i,i2)=beta_i(i)+l(i)<=beta_i(i2)+M*(1-delta_ii(i,i2));
        end
    end
end
prob.Constraints.cons28 = cons28;

cons29 = optimconstr(I,I,T);
cons30 = optimconstr(I,I,T);
for i=1:I
    for i2=1:I
        if i~=i2
            cons29(i,i2,:)=delta_ii(i,i2)+delta_ii(i2,i)<=1+M*(2-theta_it(i,:)-theta_it(i2,:));
            cons30(i,i2,:)=delta_ii(i,i2)+delta_ii(i2,i)>=1-M*(2-theta_it(i,:)-theta_it(i2,:));
        end
    end
end
prob.Constraints.cons29 = cons29;
prob.Constraints.cons30 = cons30;
disp('constraint 1~30 Done');

prob.Constraints.cons31 = sum(sum(eta_qit,1),2)<=Q;

prob.Constraints.cons32 = sum(sum(eta_qit,1),3)>=c;

prob.Constraints.cons33 = sum(eta_qit,2)<=1;

prob.Constraints.cons34 = squeeze(sum(eta_qit,1))<=repmat(q_max',1,T)+M*(1-theta_it);

prob.Constraints.cons35 = squeeze(sum(eta_qit,1))>=repmat(q_min',1,T)-M*(1-theta_it);

prob.Constraints.cons36 = squeeze(sum(eta_qit,1))<=M*theta_it;

cons37 = optimconstr(I,T,Q-1,Q-2);
for q=2:Q
    for q2=q+1:Q
        cons37(:,:,q-1,q2-2)=squeeze(eta_qit(q2,:,:)+eta_qit(q-1,:,:)-eta_qit(q,:,:)<=1);
    end
end
prob.Constraints.cons37 = cons37;

cons38 = optimconstr(I,I,T,Q,Q);
for i=1:I
    for i2=1:I
        if i~=i2
            for q=1:Q
                for q2=1:Q
                    if q~=q2
                        cons38(i,i2,:,q,q2)=q<=q2+M*(3-eta_qit(q,i,:)-eta_qit(q2,i2,:)-repmat(delta_ii(i,i2),1,1,T))+1;
                    end
                end
            end
        end
    end
end
prob.Constraints.cons38 = cons38;

cons39 = optimconstr(I,T,T);
cons40 = optimconstr(I,T,T);
for t=1:T
    for t2=1:T
        if t~=t2
            cons39(:,t,t2)=sum(eta_qit(:,:,t),1)-sum(eta_qit(:,:,t2),1)<=1+M*(2-theta_it(:,t)-theta_it(:,t2))';
            cons40(:,t,t2)=sum(eta_qit(:,:,t),1)-sum(eta_qit(:,:,t2),1)>=-1-M*(2-theta_it(:,t)-theta_it(:,t2))';
        end
    end
end
prob.Constraints.cons39 = cons39;
prob.Constraints.cons40 = cons40;
disp('constraint 1~40 Done');
toc;

%% Solve the Optimal Problem with Gurobi
tic;
disp('Solving Problem');
solution = solve(prob);
toc;

OBJ = sum(solution.ksi_i - a);

%% Visualization
figure;
set(gcf,'Position',[0 0 12000 6762])
xlabel('Berth Segment/50m');
ylabel('T/Hour');
title('BACASP-TC/Solver')
axis([0 B+1 0 2 * N * V]);
grid on;

for i = 1:N
   hold on;
   rectangle('Position',[0, 1+(i-1) * 2 * V, B+1, V], 'LineWidth', 2, 'FaceColor', 'R', 'EdgeColor','R');
   rectangle('Position',[0, 1+(2 * i-1) * V, B+1, V], 'LineWidth', 2, 'FaceColor', 'G', 'EdgeColor','G');
end

for i = 1:I
   hold on;
   if ismember(i,tide)
       rectangle('Position',[solution.beta_i(i) - 1,solution.epsilon_i(i),l(i),solution.rho_i(i)], 'LineWidth', 2, 'FaceColor', 'B');
   end
   if ismember(i,free)
       rectangle('Position',[solution.beta_i(i) - 1,solution.epsilon_i(i),l(i),solution.rho_i(i)], 'LineWidth', 2, 'FaceColor', 'Y');
   end
   
   text(solution.beta_i(i) - 1 + l(i)/2, solution.epsilon_i(i)+ solution.rho_i(i)/2, {num2str(i)},'FontSize',24,'Color', 'Black');
   text(solution.beta_i(i) + l(i) - 1.25, solution.epsilon_i(i) + solution.rho_i(i)/2, {num2str((solution.lamda_i(i)-a(i)))},'FontSize',10,'Color','Black');
   
   
   for t = solution.epsilon_i(i):solution.sigma_i(i)-1
      tmp = [];
      for q = 1:Q
          if solution.eta_qit(q,i,t) == 1
              tmp = [tmp,q];
          end
      end
      
      len = length(tmp);
      if len ~= 0
          for j = 1:len
              x_coor = solution.beta_i(i) - 1 + (j-1) * (l(i)/len);
              y_coor = t;
              width = l(i)/len;
              height = 1;
              rectangle('Position',[x_coor,y_coor , width, height], 'LineWidth', 0.1);
              text(x_coor + width / 2 , y_coor + height / 2, ['QC',num2str(tmp(j))],'FontSize',10);
          end
      end
   end
   
end
figure_name = ['Experiment/Solver_' num2str(I)];
print('-djpeg','-r600',figure_name);