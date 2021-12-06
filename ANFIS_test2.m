clc;
clear all;
close all;
x1 = 0:0.1:2; % x coordinates for validation
y1 = 8:0.1:10; % y coordinates for validation
l1 = 7; % length of first arm
l2 = 10; % length of second arm


[X2,Y2] = meshgrid(x1,y1);
XY1 = [X2(:) Y2(:)];
anf = ANFIS(7,150,0);
model1 = readfis('theta1_model');
model2 = readfis('theta2_model');

out_1 = evaluate(anf,model1,XY1);
out_2 = evaluate(anf,model2,XY1);

c2 = (X2.^2 + Y2.^2 - l1^2 - l2^2)/(2*l1*l2);
s2 = sqrt(1 - c2.^2);
pred_theta_2 = atan2(s2,c2); % theta2 is deduced

k1 = l1 + l2.*c2;
k2 = l2*s2;
pred_theta_1 = atan2(Y2,X2) - atan2(k2,k1); % theta1 is deduced
%% 

[error_theta1, error_theta2] = error(anf,out_1,pred_theta_1,out_2,pred_theta_2);
fprintf("Evaluation done");
%% 
figure
plot(error_theta1);
hold on
plot(error_theta2);
legend('theta1 error','theta2 error')

%% 
XY1 = [0.5 0.5];
