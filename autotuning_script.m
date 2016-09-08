%%%%%%%%%
%
% Autotuning Algorithm for second order systems
% 
% Dimitrios Tsounis
% dimtsouis@gmail.com
% The University of Manchester
% April 2016
%
%%%%%%%

clear all;
clc;

% Values of damping, natural frequency and SS gain
damping_ratio = (2:8)/10;
natural_freq = (3:10);
b0 = 1;

%variables used to select different frequency/damping values
i = 1;  % natural frequency
j = 1;  % damping ratio

% Calculating the values of the nominator and denominator of the TF
a = b0*(natural_freq(i)^2);
b = 2*natural_freq(i)*damping_ratio(j);
c = natural_freq(i)^2;         

num = [a];
den = [1 b c]; 

% Setting up the Transfer Function
system_tf = tf(num,den);

% flags & counting variables
k = 0;
inc = 1; %delete for final version
sum_inc(1:50)=0;

%%%%%%
% String Manipulation
% 1st Annotation box
s1_freq = num2str(natural_freq(i));
s2_damp = num2str(damping_ratio(j));
s3_b0 = num2str(b0);

s1 = strcat('Natural Frequency = ', s1_freq,' rad/s');
s2 = strcat('Damping ratio = ', s2_damp);
s3 = strcat('Bo = ', s3_b0);

s = [s1 char(10) s2 char(10) s3];
%%%%%%

% Setting initial PID gains
max_prop_gain = 4;
max_int_gain = 4;
% kp = rand*max_prop_gain;
% ki = rand*max_int_gain;

kp = 10;
ki = 10;

kp_init = kp;
ki_init = ki;

% Setting Crossover ratios
COR1 = 0.2; % fitness
COR2 = 0.4; % oscillation
COR3 = 0.2; % overshoot
COR4 = 0.2; % time ratio

%population creation
gains = [ kp ki;
    0.5*kp ki;
    kp 0.5*ki;
    0.5*kp 0.5*ki;
    0.5*kp 3*ki;
    3*kp 0.5*ki;
    3*kp 3*ki;];

disp('Initial gains');
disp(kp);
disp(ki);

%%%%%%
initial_pid = pid(gains(1,1),gains(1,2),0.1);
% Plotting the initial response of the system
input_function1 = feedback(initial_pid*system_tf,1);
[y1,t1]=step(input_function1);
fig1 = figure;
plot(t1,y1)
xlabel('Time (sec)');
ylabel('Amplitude');
title('');
annotation('textbox', 'String',s,'Fontsize',12);
hold on;
%%%%%%

while true
    %%%
    % Following values need updating if the size of the population changes
    sum(1:7) = 0;
    sum_counter = 1;
    flag_var(1:7) = 0;
    overshoot(1:7) = 0;

    for population_size=1:size(gains,1)
        clearvars y;
        clearvars t;
        clearvars pks;
        clearvars locs;
        clearvars mpks;
        clearvars rise_point;
        clearvars time_stamp1;
        clearvars time_stamp2;
        clearvars ts1_value;
        clearvars ts2_value;

        % Setting up PI Controller
        C = pid(gains(population_size,1),gains(population_size,2),0.1);
        
        % generating input function for feedback loop with control
        input_function = feedback(C*system_tf,1);
        
        %step function
        [y,t]=step(input_function);
        %%%
        
        %%%%%%
        % Annotation Box
        skp = num2str(gains(population_size,1));
        ski = num2str(gains(population_size,2));
        % skd = num2str(Kd); % derivative contol not present
    
        s1kp = strcat('Kp = ', skp);
        s2ki = strcat('Ki = ',ski);
        %  s3kd = strcat('Kd = ',skd);  % derivative contol not present
        
        s_gains = ['PI Gains' char(10) s1kp char(10) s2ki]; % textbox element
        %%%%%%
        
        % Default values
        flag_var(population_size) = 0;
        tuning_rqrd(population_size) = 0;
        tuning_level(population_size) = 1;
        
        % Signal Info
        s_info = stepinfo(y,t);
        
        % Plotting the response (commented out to speed up program execution)
        % 
        % figure;
        % plot(t,y);
        % annotation('textbox', 'String',s,'Fontsize',12);
        % annotation('textbox',[.6 0.3 .1 .1], 'String',s_gains,'Fontsize',12);
        % title('');
        
        %%%%%%%%%%%%%
        % Signal Analysis
        
        [pks,locs] = findpeaks(y); % Local maxima estimation
        [mpks,mlocs] = findpeaks(-y); % Local minima estimation
        mpks = (-mpks); % chaning the sign of the values
        
        maxima_num(population_size) = size(pks,1);
        minima_num(population_size) = size(mpks,1);
        
        % Check if system reaches steady state
        ss_value = y(size(y,1));
        if (ss_value>=0.95) && (ss_value<=1.05);
            steady_state_reached(population_size) = true;
            tuning_rqrd(population_size) = 0;
        else
            tuning_rqrd(population_size) = 1;
            steady_state_reached(population_size) = false;
        end
        
        % Checking if system is oscillating
        % By changing the if statements, it is possible to change the
        % number of acceptable oscillation
        if ((maxima_num(population_size))>=4) && (minima_num(population_size)>=3);
            oscillating(population_size) = 1;
            tuning_rqrd(population_size) = 1;
            tuning_level(population_size) = 2; %#ok<*SAGROW>
            if (maxima_num(population_size)>5) || (abs(pks(1)-mpks(1))>0.4)
                tuning_level(population_size) = 3;
            end
            if maxima_num(population_size)>6 || (abs(pks(1)-mpks(1))>0.6)
                tuning_level(population_size) =4;
            end
            if maxima_num(population_size)>7 || (abs(pks(1)-mpks(1))>0.8)
                tuning_level(population_size) =5;
            end
        else
            oscillating(population_size) = 0;
        end  
        
        % Checking system time response
        % further work is required to established a 'good' reference value
        t_ratio(population_size) = s_info.SettlingTime/s_info.RiseTime;
        if t_ratio(population_size)>40  
            tuning_rqrd(population_size) = 1;
        end
        
        % Derivative estimation using central difference
        % Commented out because it is not used in this algorithm
%         for l=1:(size(y,1)-2)
%             der(l) = (y(l+2)-y(l+1))/(t(l+2)-t(l+1));
%         end
        
        % Checking if the system is overshooting or undershooting
        for j=1:round(size(t,1)/2)
            if y(j)> 1
                flag_var(population_size) = 1; % system overshooting
            end
        end       
        if (size(pks,1))>=1
            overshoot(population_size) = pks(1)-1; % overshoot in decimal percentage
        end
        
        if (size(mpks,1))>=1
            undershoot(population_size) = abs(pks(1)-1); % undershoot in decimal percentage
        end
        
        if flag_var(population_size) == 1
            if overshoot(population_size) >0.6
                tuning_rqrd(population_size) =1;
            end
        end
        
        % establishing the nearest discrete time value to the rise time
        for j=1:size(t,1)
            diff_eqn = (s_info.RiseTime)-t(j);
            if diff_eqn >= 0
                time_stamp1 = j;
                ts1_value = diff_eqn;
            end
            if diff_eqn <= 0
                time_stamp2 = j;
                ts2_value = diff_eqn;
                break;
            end
            diff_eqn = 0;
        end
        
        % choosing the discrete t value closest to the rise time
        if abs(ts1_value) > abs(ts2_value)
            rise_point = time_stamp2;
        else
            rise_point = time_stamp1;
        end
        
        % Fitness Function
        for i=1:rise_point
            sum(sum_counter) = sum(sum_counter) + abs(1-y(i));
        end
        sum_counter = sum_counter+1;
        
        % debug code
        % disp('The system is: ');
        % disp(strcat('Oscillating: ',num2str(oscillating(population_size))));
        % disp(strcat('Overshooting: ',num2str(flag_var(population_size))));
        % disp(strcat('Reaches Steady State: ',num2str(steady_state_reached(population_size))));
        % disp(strcat('The settling time to rise time ratio: ',num2str(t_ratio(population_size))));

    end
   
    %%%%%%%%
    
    % sorting Fitness Function  
    [current_sum, index1] = sort(sum);
    
    sum_inc(inc) = sum_inc(inc) + current_sum(1);
    
    for cntr=1:size(sum,2)
        if sum(cntr) > 15
            tuning_rqrd(cntr) = 1;
            % fitness issues
            
        end
    end

    %s sorting tuning_rqrd
    [tun_rqrd_sorted,tun_req_index] = sort(tuning_rqrd);
 
    if tun_rqrd_sorted(1) == 0
        disp('Tuning Achieved');
        disp(gains(tun_req_index(1),1));
        disp(gains(tun_req_index(1),2));
        
        % saving gains for graph
        gainp(k+1) = gains(tun_req_index(1),1);
        gaini(k+1) = gains(tun_req_index(1),2);
        C = pid(gains(tun_req_index(1),1),gains(tun_req_index(1),2),0.1);
        
        % generating input function for feedback loop with control
        input_function = feedback(C*system_tf,1);
        
        %step function
        [y,t]=step(input_function);
        plot(t,y)
        legend('Initial','Tuned');
        
        %stop iteration
        break;
    else
        disp('Tuning required');
    end
    
    
    %%%%%%%%%%%
    % Tuning
    
    % Fitness function optimisation
    kp_fitness = gains(index1(1),1);
    ki_fitness = gains(index1(1),2);
    
    % Steady state optimisation
    % steady state gain not used in this version of the algorithm
    [ss_sorted,index_ss]= sort(steady_state_reached,'descend');
    kp_ss = gains(index_ss(1),1);
    ki_ss = gains(index_ss(1),2);
    
    % Oscillation optimisation
    [tun_level_intensity,index2] = sort(tuning_level);
    
    % adjust gains based on the oscillation intensity level
    if (tun_level_intensity(1) == 5)
        kp_osc = gains(index2(1),1)/3;
        ki_osc = gains(index2(1),2)/3;
    end
    if (tun_level_intensity(1) == 4)
        kp_osc = gains(index2(1),1)/2;
        ki_osc = gains(index2(1),2)/2;
    end
    if (tun_level_intensity(1) == 3)
        kp_osc = gains(index2(1),1)/2;
        ki_osc = gains(index2(1),2)/1.5;
    end
    if (tun_level_intensity(1)== 2)
        kp_osc = gains(index2(1),1)/1.5;
        ki_osc = gains(index2(1),2);
    end
    if (tun_level_intensity(1)== 1)
        % if tuning level = 1 keep the same gains
        kp_osc = gains(index2(1),1);
        ki_osc = gains(index2(1),2);
    end
    
    % Overshoot optimisation
    [flag_sorted,index3] = sort(flag_var);
    
    [overshoot_sorted,index4] =sort(overshoot);

    for i=1:size(overshoot_sorted,1)
        if overshoot_sorted(i)<0
            ovrt_indic = 0;
        end
        if overshoot_sorted(i) >= 0 && overshoot_sorted(i)<0.5
            ovrt_indic = 1;
            kp_overshoot = gains(index4(i),1);
            ki_overshoot = gains(index4(i),2);
            break;
        else
            ovrt_indic = 0;
            kp_overshoot = gains(index4(i),1)/1.5;
            ki_overshoot = gains(index4(i),2)/1.25;
            break;
        end
    end
    
    % t_ratio optimisation
    [t_ratio_sorted,trindex] = sort(t_ratio);
    kp_tr = gains(trindex(1),1);
    ki_tr = gains(trindex(1),2);
    
    if ovrt_indic == 1
        kp = COR1*kp_fitness + COR2*kp_osc + COR3*kp_overshoot + COR4*kp_tr;
        ki = COR1*ki_fitness + COR2*ki_osc + COR3*ki_overshoot + COR4*ki_tr;
    else
        kp = COR1*kp_fitness + COR2*kp_osc + COR4*kp_tr;
        ki = COR1*ki_fitness + COR2*ki_osc + COR4*ki_tr;
    end
    
    if k>1
        if last_kp == kp
            kp = (rand+0.2)*kp;
        end
        if last_ki == ki
            ki = (rand+0.2)*ki;
        end
    end
   
   % pre defined population generation
   gains = [ kp ki;
        0.5*kp ki;
        kp 0.5*ki;
        0.5*kp 0.5*ki;
        0.5*kp 3*ki;
        3*kp   0.5*ki;
        3*kp   3*ki;];
    
            
    % random population generation
    gains = [ kp ki;
        rand*kp ki;
        kp rand*ki;
        rand*kp rand*ki
        rand*kp rand*ki;
        2*kp   2*ki;
        10*rand*kp   10*rand*ki;];
     
    last_kp = kp;
    last_ki = ki;
   
    %saving gains for graph
    gainp(k+1) = kp;
    gaini(k+1) = ki;
    %%%%%%%%%%%

    disp('==================');
    % loop termination
    if k == 50
        disp(gains);
        % Final System
        C = pid(gains(index1(1),1),gains(index1(1),2),0.1);
        
        % generating input function for feedback loop with control
        input_function = feedback(C*system_tf,1);
        
        %step function
        [y,t]=step(input_function);
        plot(t,y)
        legend('Initial','Final');
        break;
    else
        k = k+1;
        inc = inc +1;
    end
end

if k == 0
    k = k+1;
end

% plotting the gain variation

% Kp Graph

% Annotation Box
kp_p1 = num2str(gainp(1));
kp_p2 = num2str(gainp(end));
s_kp_p1 = strcat('Initial Gain: ', kp_p1);
s_kp_p2  = strcat('Final Gain: ',kp_p2);
p_gains_variation = [s_kp_p1 char(10) s_kp_p2]; % textbox element

xaxis = (1:(k+1));
fig2 = figure;
plot(xaxis,gainp)
xlabel('Iterations');
ylabel('Kp');
title('');
xbounds = xlim();
annotation('textbox',[0.15 0.13 0.25 0.08], 'String',p_gains_variation,'Fontsize',13);
set(gca, 'xtick', xbounds(1):1:xbounds(2));

% Ki Graph
% Annotation Box
ki_p1 = num2str(gaini(1));
ki_p2 = num2str(gaini(end));
s_ki_p1 = strcat('Initial Gain: ', ki_p1);
s_ki_p2  = strcat('Final Gain: ',ki_p2);
i_gains_variation = [s_ki_p1 char(10) s_ki_p2]; % textbox element

fig3 = figure;
plot(xaxis,gaini)
xlabel('Iterations');
ylabel('Ki');
title('');
xbounds = xlim();
annotation('textbox',[0.15 0.13 0.25 0.08], 'String',i_gains_variation,'Fontsize',13);
set(gca, 'xtick', xbounds(1):1:xbounds(2));

% fig4 = figure;
% sum_inc = sum_inc(sum_inc~=0); 
% incaxis = (1:inc);
% plot(incaxis,sum_inc);
% 
% 
% saveas(fig1,'response.bmp');
% saveas(fig2,'kp.bmp');
% saveas(fig3,'ki.bmp');
% 
%     