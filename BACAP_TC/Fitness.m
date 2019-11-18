function [fitness,dist_t1, dist_t2, dist_t3, t_out, rect, remain_sum, remain_time]...
    = Fitness(pop, tide, free, a, l, B, M, N,...
              V, T, C, c,Q,q_min,q_max,Visual,Method)
%initial
fitness = zeros(size(pop,1),1);
I = length(l);

for k = 1:size(pop,1)
    individual = reshape(pop(k,:,:),3,size(pop,3));
    
    %% Berth Allocation
    rect = [];
    
    for i = 1:size(individual,2)
       berthed = 0;
       if ismember(individual(1,i),tide)
           [earliest_time, result] = InHighWater(a(individual(1,i)),V,N);
           if result
               t_i_e = a(individual(1,i));
           else
               t_i_e = earliest_time;
           end
       else
           t_i_e = a(individual(1,i));
       end
       
       x_rect = individual(2,i);
       y_rect = t_i_e + C;
       p_i = individual(3,i);
       l_i = l(individual(1,i));

       while berthed == 0
           [~, tmp2] = InHighWater(y_rect-C,V,N);
           if (ismember(individual(1,i),free) && NotOverlap(x_rect,y_rect,l_i,p_i,rect)) && x_rect + l_i <= B ||...
              (ismember(individual(1,i),tide) && NotOverlap(x_rect,y_rect,l_i,p_i,rect) && tmp2 && x_rect + l_i <= B)
              
              berthed = 1;
              break;
              
           elseif x_rect + l_i > B
               x_rect = B - l_i;
           else
               y_rect = y_rect + 1;
               %disp('intersection')
               if y_rect + p_i >= T
                   y_rect = 2 * V * N;
                   berthed = 1;
               end
           end 
       end
       rect = [rect; x_rect y_rect, l_i, p_i];
    end

    %% Visualization of initial solution
    if Visual
        figure;
        set(gcf,'Position',[0 0 12000 6762])
        xlabel('Berth Segment/50m');
        ylabel('T/Hour');
        title(['BACASP-TC/' Method '-Initial'])
        axis([0, B + 1, 0, 2 * N * V]);
        grid on;

        for i = 1:N
           hold on;
           rectangle('Position',[0, 1+(i-1) * 2 * V, B+1, V], 'LineWidth', 2, 'FaceColor', 'R', 'EdgeColor','R');
           rectangle('Position',[0, 1+(2 * i-1) * V, B+1, V], 'LineWidth', 2, 'FaceColor', 'G', 'EdgeColor','G');
        end

        for i = 1:size(individual,2)
           hold on;
           if ismember(individual(1,i),tide)
               rectangle('Position',[rect(i,1),rect(i,2),rect(i,3),rect(i,4)], 'LineWidth', 2, 'FaceColor', 'B');
           end
           if ismember(individual(1,i),free)
               rectangle('Position',[rect(i,1),rect(i,2),rect(i,3),rect(i,4)], 'LineWidth', 2, 'FaceColor', 'Y');
           end

           text(rect(i,1) + rect(i,3)/2 , rect(i,2) + rect(i,4)/2, {num2str(individual(1,i))},'FontSize',24,'Color', 'Black');
        end
        figure_name = ['Experiment/' Method '_' num2str(I) '_1'];
        print('-djpeg','-r600',figure_name);
    end

    %% Quay Assignment
    dist_t1 = zeros(2*N*V,size(individual,2));
    dist_t2 = zeros(2*N*V,1);
    dist_t3 = zeros(2*N*V,2,size(individual,2));
    
    for t=1:T
       for i = 1:size(individual,2)
          if t >= rect(i,2) && t < rect(i,2)+rect(i,4)
             dist_t1(t,i) = individual(1,i);
             dist_t2(t) = dist_t2(t) + 1;
          end
       end
    end

    remain_sum = 0;
    remain_time = 0;
    for t=1:T
        sum = 0;
        avg = [];
        percent = [];
        for j = 1:dist_t2(t)
            temp = dist_t1(t,dist_t1(t,:)>0);
            i = temp(j);
            if rect(individual(1,:)==i,2) == t
               remain_sum(i) = c(i);
               remain_time(i) = rect(individual(1,:)==i,4);
            end
            avg(j) = remain_sum(i) / remain_time(i);
            sum = sum + avg(j);
        end

        for j = 1:dist_t2(t)
            temp = dist_t1(t,dist_t1(t,:)>0);
            i = temp(j);
            
            % if sum=0, not assign quay to vessel i
            if sum == 0
                dist_t3(t,1,j) = i;
                dist_t3(t,2,j) = 0;
            else
                percent(j) = avg(j) / sum;
                dist_t3(t,1,j) = i;
                %temp2 just for me to check 
                temp2 = round(Q*percent(j));
                dist_t3(t,2,j) = round(Q*percent(j));
                
                if dist_t3(t,2,j) < q_min(i) && ~dist_t3(t,2,j) == 0
                    dist_t3(t,2,j) = q_min(i);
                end
                if dist_t3(t,2,j) > q_max(i)
                   dist_t3(t,2,j) = q_max(i);
                end
                
                if t >= rect(individual(1,:)==i,2) + 1 && dist_t3(t,2,j) > dist_t3(t,2,squeeze(dist_t3(t-1,1,:)==i)) + 1
                    dist_t3(t,2,j) = dist_t3(t,2,squeeze(dist_t3(t-1,1,:)==i)) + 1;
                end
                if t >= rect(individual(1,:)==i,2) + 1  && dist_t3(t,2,j) < dist_t3(t,2,squeeze(dist_t3(t-1,1,:)==i)) - 1
                    dist_t3(t,2,j) = dist_t3(t,2,squeeze(dist_t3(t-1,1,:)==i)) - 1;
                end
                % reduce the processing time
                % attention: this operation will make the number of QC less
                % than q_min, but the last time step, it does not matter
                if dist_t3(t,2,j) >= remain_sum(i) && ~remain_sum(i) ==0
                   dist_t3(t,2,j) = remain_sum(i);
                   rect(individual(1,:)==i,4) = t + 1 - rect(individual(1,:)==i,2);
                end
                remain_sum(i) = remain_sum(i) - dist_t3(t,2,j);
                remain_time(i) = remain_time(i) - 1;
                % add the processing time
                if ~remain_sum(i) == 0 && remain_time(i)==0
                    if t+1 <= T
                        dist_t2(t+1) = dist_t2(t+1) + 1;
                        dist_t1(t+1,individual(1,:)==i) = i;
                        remain_time(i) = remain_time(i) + 1;
                        rect(individual(1,:)==i,4) = rect(individual(1,:)==i,4) + 1;
                    end
                end
            end
        end
    end

    %% Calculate t_out
    t_out =zeros(2,size(rect,1));
    obj = 0;
    for i=1:size(rect,1)
       t_out(1,i) = individual(1,i);
       t_out(2,i) = rect(i,2) + rect(i,4);
       [tmp1,tmp2] = InHighWater(t_out(2,i),V,N);
       if ismember(t_out(1,i),tide) && ~tmp2
           t_out(2,i) = tmp1;
       end
       obj = obj + t_out(2,i) + C - a(t_out(1,i));
    end
    
    fitness(k)=1/obj;

    %% Visualization of adjusted solution
    if Visual
        figure;
        set(gcf,'Position',[0 0 12000 6762])
        xlabel('Berth Segment/50m');
        ylabel('T/Hour');
        title(['BACASP-TC/' Method '-Adjusted'])
        axis([0 B+1 0 2*N*V]);
        grid on;

        for i = 1:N
           hold on;
           rectangle('Position',[0, 1+(i-1) * 2 * V, B+1, V], 'LineWidth', 2, 'FaceColor', 'R', 'EdgeColor','R');
           rectangle('Position',[0, 1+(2 * i-1) * V, B+1, V], 'LineWidth', 2, 'FaceColor', 'G', 'EdgeColor','G');
        end

        for i = 1:size(individual,2)
           hold on;
           if ismember(individual(1,i),tide)
               rectangle('Position',[rect(i,1),rect(i,2),rect(i,3),rect(i,4)], 'LineWidth', 2, 'FaceColor', 'B');
           end
           if ismember(individual(1,i),free)
               rectangle('Position',[rect(i,1),rect(i,2),rect(i,3),rect(i,4)], 'LineWidth', 2, 'FaceColor', 'Y');
           end

           text(rect(i,1) + rect(i,3)/2 , rect(i,2) + rect(i,4)/2, {num2str(individual(1,i))},'FontSize',24,'Color', 'Black');
           text(rect(i,1) + rect(i,3) - 0.25 , rect(i,2) + rect(i,4)/2, {num2str(rect(i,2) - a(individual(1,i)) - C)},'FontSize',10,'Color', 'Black');
        end
        
        % QC Visualization
        for t=1:T
            dist = reshape(dist_t3(t,:,:),2,size(dist_t3,3));
            if dist(1,1) == 0
                continue;
            else
                index = find(dist(1,:) > 0);
                tmp = [];
                for i = 1:size(index,2)
                    tmp(1,i) = dist(1,index(i));
                    tmp(2,i) = dist(2,index(i));
                    info = rect(individual(1,:,:)==tmp(1,i),:);
                    tmp(3,i) = info(1);
                    tmp(4,i) = info(3); 
                end
                tmp_sort = sortrows(tmp',3)';
                
                
                qc = 1;   
                for i = 1:size(index,2)
                    for j = 1 : tmp_sort(2,i)
                    hold on; 
                    x_pos = tmp_sort(3,i) + (j-1) * (tmp_sort(4,i)/tmp_sort(2,i));
                    width = tmp_sort(4,i)/tmp_sort(2,i);
                    rectangle('Position',[x_pos, t, width, 1], 'LineWidth', 0.5);
                    
                    x_center_rect = tmp_sort(3,i) + (j-1) * (tmp_sort(4,i) / tmp_sort(2,i)) ...
                                    + 1/2 * tmp_sort(4,i)/tmp_sort(2,i);
                    text(x_center_rect, t + 0.5 , ['QC',num2str(qc)],'FontSize',5);
                    qc = qc + 1;
                    end
                end
            end

        end
        figure_name = ['Experiment/' Method '_' num2str(I)];
        print('-djpeg','-r600',figure_name);
    end
end
end