%% TEST ANFIS CLASS
clc;
clear all;
close all;
anf = ANFIS(7,150,0);

l1 = 0.9144; % length of first arm
l2 = 0.9144; % length of second arm

theta1 = 0:0.1:pi; % all possible theta1 values
theta2 = 0:0.1:pi; % all possible theta2 values

[T1,T2] = meshgrid(theta1,theta2); % generate grid of angle values

X1 = l1 * cos(T1) + l2 * cos(T1 + T2); % compute x coordinates
Y1 = l1 * sin(T1) + l2 * sin(T1 + T2); % compute y coordinates

data1 = [X1(:) Y1(:) T1(:)]; % create x-y-theta1 dataset
data2 = [X1(:) Y1(:) T2(:)]; % create x-y-theta2 dataset
fprintf("Model 1 starting training");
model1 = anf.train(data1);
fprintf("Model 1 finished training");
%% 
fprintf("Model 2 starting training");
model2 = anf.train(data2);
fprintf("Model 2 training finished");

%% 
writeFIS(model1,'theta1_model');
writeFIS(model2,'theta2_model');
%% 
%% 

% x1 = 0:0.1:2; % x coordinates for validation
% y1 = 8:0.1:10; % y coordinates for validation
% [X2,Y2] = meshgrid(x1,y1);
% XY1 = [X2(:) Y2(:)];
% out_1 = evaluate(anf,model1,XY1);
% out_2 = evaluate(anf,model2,XY1);
% 
% c2 = (X2.^2 + Y2.^2 - l1^2 - l2^2)/(2*l1*l2);
% s2 = sqrt(1 - c2.^2);
% pred_theta_2 = atan2(s2,c2); % theta2 is deduced
% 
% k1 = l1 + l2.*c2;
% k2 = l2*s2;
% pred_theta_1 = atan2(Y2,X2) - atan2(k2,k1); % theta1 is deduced
% %% 
% 
% [error_theta1, error_theta2] = error(anf,out_1,pred_theta_1,out_2,pred_theta_2);
% fprintf("Evaluation done");
% %% 
% figure
% plot(error_theta1);
% hold on
% plot(error_theta2);
% legend('theta1 error','theta2 error')