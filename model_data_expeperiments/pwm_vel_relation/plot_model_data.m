%% Get the data
% cw - clockwise
% ccw - countercloclwise
close all;
clear;

data_CW = importfile('pwm_vel_relation_CW.csv');
data_CCW = importfile('pwm_vel_relation_CCW.csv');

%% CCW only

% we set the maximum velocity to the 90% of the maximum measured to be sure
% to be sure of not working in the motor saturation area
max_vel_CCW = max(data_CCW(:,3))*0.9; 

% Linear regression model
PWM_CCW = data_CCW(:,1);
vel_CCW = data_CCW(:,3);
format long
vel_PWM_relation_CCW = max_vel_CCW/data_CCW(end,1);
vel_model_CCW = vel_PWM_relation_CCW*PWM_CCW;

figure
plot(PWM_CCW,vel_CCW,'-','LineWidth',0.5)
hold on
plot(PWM_CCW,vel_model_CCW)
legend('data', 'CCW model')
ylabel('velocity [rad/s]');
xlabel('PWM [bits]')
xlim([0 max(PWM_CCW)]);
grid on; grid minor;

%% CW only

% we set the maximum velocity to the 90% of the maximum measured to be sure
% to be sure of not working in the motor saturation area
max_vel_CW = max(abs(data_CW(:,3)))*0.9; 

% Linear regression model
PWM_CW = -data_CW(:,1);
vel_CW = data_CW(:,3);
format long
vel_PWM_relation_CW = max_vel_CW/data_CW(end,1);
vel_model_CW = vel_PWM_relation_CW*PWM_CW;

figure
scatter(PWM_CW,vel_CW)
hold on
plot(PWM_CW,vel_model_CW)
legend('data', 'CW model')
ylabel('velocity [rad/s]');
xlabel('PWM [bits]')
xlim([min(PWM_CW) 0]);
grid on; grid minor;



%% Both directions

% we set the maximum velocity to the 90% of the maximum measured to be sure
% to be sure of not working in the motor saturation area
max_vel_both = (max(data_CCW(:,3))+max(abs(data_CW(:,3))))/2*0.9; 

% Linear regression model
PWM_both = [-data_CW(:,1);data_CCW(:,1)];
vel_both = [data_CW(:,3);data_CCW(:,3)];
vel_PWM_relation_both = max_vel_both/PWM_both(end,1);
vel_model_both = vel_PWM_relation_both*PWM_both;
vel_model_CCW = vel_PWM_relation_CCW*PWM_both;
vel_model_CW = vel_PWM_relation_CW*PWM_both;

figure
plot(PWM_both,vel_both,'.','LineWidth',0.5)
hold on
plot(PWM_both,vel_model_both)
plot(PWM_both,vel_model_CCW)
plot(PWM_both,vel_model_CW)
legend('data','whole model','CCW model','CW model')
ylabel('velocity [rad/s]');
xlabel('PWM [bits]')
xlim([min(PWM_both) max(PWM_both)]);
grid on; grid minor;