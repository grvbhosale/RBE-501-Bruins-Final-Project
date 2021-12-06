close all
figure(1)
bar(flighttime,mean_ik,'k')
hold on
bar(flighttime,mean_anfis,'r','FaceAlpha',0.8)
xlim([0.1 1])
xlabel("Time of Flight (s)")
ylabel("Distance from ball (m)")
set(gca, 'XDir','reverse')
legend({'Inverse Kinematics',"ANFIS"},'location','best')
title("Mean distance from ball to end effector for each algorithm")
hold off

figure(2)
bar(flighttime,difference*1000)
xlim([0.1 1])
xlabel("Time of Flight (s)")
ylabel("Difference(mm)")
set(gca, 'XDir','reverse')
title("Difference of distance from ball between algorithms")
% hold on 
% plot(flighttime,dev,'LineWidth',1)
% hold off

figure(3)
scatter(ft_all,deviation_all,10,distance_ik_all,'filled')
xlim([0.1 1])
xlabel("Time of Flight (s)")
ylabel("Deviation(m)")
set(gca, 'XDir','reverse')
title("Distance from ball based on flight time and deviation (ik)")
colorbar

figure(4)
scatter(ft_all,deviation_all,10,distance_anfis_all,'filled')
xlim([0.1 1])
xlabel("Time of Flight (s)")
ylabel("Deviation(m)")
set(gca, 'XDir','reverse')
title("Distance from ball based on flight time and deviation (anfis)")
colorbar

figure(5)
bar(flighttime,int_ik_mean*100,'c')
hold on
bar(flighttime,int_anfis_mean*100,'g')
xlim([0.1 1])
xlabel("Time of Flight (s)")
ylabel("Mean Iteration Interval (ms)")
set(gca, 'XDir','reverse')
legend({'Inverse Kinematics',"ANFIS"},'location','best')
title("Mean iteration interval of each algorithm")
hold off