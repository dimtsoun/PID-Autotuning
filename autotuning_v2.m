
%%%%%%%%%
%
% Autotuning Algorithm for reference tracking using second order systems
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

%variables used to selected different frequency/damping values
i = 1;  % natural frequency
j = 1;  % damping ratio

% Calculating the values of the nominator and denominator of the TF
a = b0*(natural_freq(i)^2);
b = 2*natural_freq(i)*damping_ratio(j);
c = natural_freq(i)^2;

k =1;

%Simulink model required
model = 'reference_tracking';
load_system(model)

max_prop_gain = 1;
max_int_gain = 1;
% kp_initial = rand*max_prop_gain;
% ki_initial = rand*max_int_gain;
 
kp_initial = 3;
ki_initial = 3;

%population creation
gains = [ kp_initial ki_initial;
    0.5*kp_initial ki_initial;
    kp_initial 0.5*ki_initial;
    0.5*kp_initial 0.5*ki_initial
    2*kp_initial 2*ki_initial];

disp(gains);
kp = kp_initial;
ki = ki_initial;

kp_p1 = num2str(kp);
ki_p2 = num2str(ki);
s_kp_p1 = strcat('Initial P Gain: ', kp_p1);
s_ki_p2  = strcat('Initial I Gain: ',ki_p2);
initial_gains = [s_kp_p1 char(10) s_ki_p2]; % textbox element


sim(model);
fig1 = figure;
plot(tout,simout.signals.values(:,1),tout,simout.signals.values(:,2));
xlabel('Time (s)');
ylabel('Amplitude');
title('');
legend('Reference Signal', 'Initial Response');
annotation('textbox',[0 0 1 1],'String',initial_gains,'Fontsize',13);

fig2 = figure;
hold on;

while true
    iter_counter = 1;
    sum(1:5) = 0;
    input(1:5) = 0;
  %  response(1:5) = 0;
    
    for i=1:size(gains,1)
        kp = gains(i,1);
        ki = gains(i,2);
        sim(model);
        input = simout.signals.values(:,1);
        response = simout.signals.values(:,2);
        plot(tout,input,tout,response);
        xlabel('Time (s)');
        ylabel('Amplitude');
        title('');

        % fitness function definition
        for j=1:size(response,1)
            sum(iter_counter) = sum(iter_counter) + abs(input(j)-response(j));
        end
        iter_counter = iter_counter + 1;
    end
    
    [sum_sorted,index] = sort(sum);
    sgra(k) = sum_sorted(1);
    kpgr(k) = gains(index(1),1);
    kigr(k) = gains(index(1),2);

    if sum_sorted(1)<= 5
        disp('System Tuned');
        kp = gains(index(1),1);
        ki = gains(index(1),2);
        %%%%
        kp_p1 = num2str(kp);
        ki_p2 = num2str(ki);
        s_kp_p1 = strcat('Final P Gain: ', kp_p1);
        s_ki_p2  = strcat('Final I Gain: ',ki_p2);
        final_gains = [s_kp_p1 char(10) s_ki_p2]; % textbox element
        %%%
        sim(model);
        fig3 = figure;
        plot(tout,simout.signals.values(:,1),tout,simout.signals.values(:,2));
        xlabel('Time (s)');
        ylabel('Amplitude');
        title('');
        legend('Reference Signal', 'Final Response');
        annotation('textbox',[0 0 1 1], 'String',final_gains,'Fontsize',13);
        

        disp(gains(index(1),1));
        disp(gains(index(1),2));
        break;
    end
    
    kp_new = gains(index(1),1);
    ki_new = gains(index(1),2);
    
    if k>2
        if last_kp == kp_new
            kp_new = (rand)*kp_new;
        end
        if last_ki == ki_new
            ki_new = (rand)*ki_new;
        end
    end
    
%     gains = [ kp_new ki_new;
%         0.5*kp_new ki_new;
%         kp_new 0.5*ki_new;
%         0.5*kp_new 0.5*ki_new
%         2*kp_new 2*ki_new];
%     
     gains = [ kp_new ki_new;
        rand*kp_new ki_new;
        kp_new rand*ki_new;
        0.5*kp_new 0.5*ki_new
        10*rand*kp_new 10*rand*ki_new];
    
    last_kp = kp_new;
    last_ki = ki_new;
    
    disp('New Iteration');
   
    if k == 100
        disp('No Tuning');
        break;
    else
        k = k + 1;
    end
end

xaxis = (1:k);

fig4 = figure;
plot(xaxis,sgra);
xlabel('Iterations');
ylabel('Fitness Function');
title('');
xbounds = xlim();
set(gca, 'xtick', xbounds(1):1:xbounds(2));

fig5 = figure;
plot(xaxis,kpgr,xaxis,kigr);
xlabel('Iterations');
ylabel('Gains');
legend('Kp','Ki');
xbounds = xlim();
set(gca, 'xtick', xbounds(1):1:xbounds(2));


saveas(fig1,'initial.bmp');
saveas(fig2,'intermediate.bmp');
saveas(fig3,'final_response.bmp');
saveas(fig4,'fitness_function.bmp');
saveas(fig5,'gains_variation.bmp');





xbounds = xlim();
set(gca, 'xtick', xbounds(1):1:xbounds(2));


