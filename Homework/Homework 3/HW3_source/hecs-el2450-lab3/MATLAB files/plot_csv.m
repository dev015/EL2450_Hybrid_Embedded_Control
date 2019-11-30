pose = importdata('pos_log_17.csv')
% [n,~] = size(pose.data(:,2));
%  ref = [];
%  ref(1:231) = 1;
%  ref(232:480) = 2;
%  ref(481:531) = 1;
%  ref(532:580) = 2;
%  ref(581:731) = 1;
%  ref(732:778) = 2;
%  ref(778:n) = 3;
plot(pose.data(:,1),pose.data(:,4))
%plot(pose.data(:,1),pose.data(:,2),pose.data(:,1),pose.data(:,3));
%plot(pose.data(:,1),pose.data(:,2),pose.data(:,1),pose.data(:,3))
%plot(pose.data(:,1),ref)
%legend('x','y')
%legend('theta')
legend('Discrete States')
hold on
grid on;
title('Plot of States v/s time')
xlabel('Timestamp')
ylabel('State Number')
hold off