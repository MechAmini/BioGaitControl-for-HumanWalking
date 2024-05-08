%% Figures
% load results data for six gait simulation by GaitSimulation Script 
load('Results_SixCycleGait.mat')

% load pelvis tilt and pelvis tilt velocity from desired 
time = [0; t];
[tjdt,tjdt_dt,tjdt_ddt,tu_fit,wu_fit] = desired(time);
%% Kinematic Results 

figure(1)

subplot(421);
plot(time, RCoordinates(:,3)*180/pi,'LineWidth',1.5,'Color','[0 0.4470 0.7410]','LineStyle','-');
hold on;plot(time, RJointRef(:,3)*180/pi,'LineWidth',1.5,'Color','[0.8 0.1470 0.7410]','LineStyle','--');
xticks([]); box off;ylabel('right hip(deg)');

subplot(423);
plot(time, RCoordinates(:,2)*180/pi,'LineWidth',1.5,'Color','[0 0.4470 0.7410]','LineStyle','-');
hold on;plot(time, RJointRef(:,2)*180/pi,'LineWidth',1.5,'Color','[0.8 0.1470 0.7410]','LineStyle','--');
xticks([]); box off;ylabel('right knee(deg)');

subplot(425);
plot(time, RCoordinates(:,1)*180/pi-0.052*180/pi,'LineWidth',1.5,'Color','[0 0.4470 0.7410]','LineStyle','-');
hold on;plot(time, RJointRef(:,1)*180/pi,'LineWidth',1.5,'Color','[0.8 0.1470 0.7410]','LineStyle','--');
xticks([]); box off;ylabel('right ankle(deg)');

subplot(422);
plot(time, RCoordinates(:,6)*180/pi,'LineWidth',1.5,'Color','[0 0.4470 0.7410]','LineStyle','-');
hold on;plot(time, RJointRef(:,6)*180/pi,'LineWidth',1.5,'Color','[0.8 0.1470 0.7410]','LineStyle','--');
xticks([]); box off;ylabel('left hip(deg)');

subplot(424);
plot(time, RCoordinates(:,5)*180/pi,'LineWidth',1.5,'Color','[0 0.4470 0.7410]','LineStyle','-');
hold on;plot(time, RJointRef(:,5)*180/pi,'LineWidth',1.5,'Color','[0.8 0.1470 0.7410]','LineStyle','--');
xticks([]); box off;ylabel('left knee(deg)');

subplot(426);
plot(time, RCoordinates(:,4)*180/pi+0.03*180/pi,'LineWidth',1.5,'Color','[0 0.4470 0.7410]','LineStyle','-');
hold on;plot(time, RJointRef(:,4)*180/pi,'LineWidth',1.5,'Color','[0.8 0.1470 0.7410]','LineStyle','--');
xticks([]); box off;ylabel('left ankle(deg)');
legend ('Bio-inspired gait control', 'Normal');

subplot(427);
plot(time, sqrt(RCoordinates(:,16).^2+RCoordinates(:,18).^2),'LineWidth',1.5,'Color','[0 0.4470 0.7410]','LineStyle','-')
hold on;plot(time, (wu_fit+3.15)/2.5,'LineWidth',1.5,'Color','[0.8 0.1470 0.7410]','LineStyle','--');
axis([0 8 0.5 2]);box off;ylabel('gait speed(m/s)'); xlabel('Time(s)');

subplot(428);
plot(time, RCoordinates(:,13)*180/pi,'LineWidth',1.5,'Color','[0 0.4470 0.7410]','LineStyle','-');
hold on;plot(time, (tu_fit-10*pi/180)*180/(2*pi),'LineWidth',1.5,'Color','[0.8 0.1470 0.7410]','LineStyle','--');
axis([0 8 -10 -5]);box off; ylabel('pelvis tilt(deg)'); xlabel('Time(s)');

%% Moments and Ground reaction force 

% Subject parameters
M = 66.7; % Mass (kg)
G = 9.81; % Gravitational acceleration (m/s^2)

% Filter data 
RTorques_f = RTorques;
for k = 1:num_torque
    RTorques_f(:,k) = conv(RTorques(:,k), g, 'same');
end

figure(2) 
subplot(321);
plot(time,-RTorques_f(:,3)/M,'LineWidth',1.5,'Color','[0 0.4470 0.7410]','LineStyle','-');hold on
% plot(time,Data_Normal_Th,'LineWidth',1.5,'Color','[0.8 0.1470 0.7410]','LineStyle','--');hold on 
xticks([]);box off;ylabel('Hip (N.m/kg)')
title ('Moments')
subplot(323);
plot(time,RTorques_f(:,2)/M,'LineWidth',1.5,'Color','[0 0.4470 0.7410]','LineStyle','-');hold on
% plot(xx,Data_Normal_Tk,'LineWidth',1.5,'Color','[0.8 0.1470 0.7410]','LineStyle','--');
xticks([]);box off;ylabel('knee (N.m/kg)')

subplot(325);
plot(time,-RTorques_f(:,1)/M,'LineWidth',1.5,'Color','[0 0.4470 0.7410]','LineStyle','-');hold on
% plot(xx,Data_Normal_Tk,'LineWidth',1.5,'Color','[0.8 0.1470 0.7410]','LineStyle','--');
box off;ylabel('Ankle (N.m/kg)');xlabel('Time(s)');

% Filter data 
RForces_f = RForces;
for k = 1:num_forces
    RForces_f(:,k) = conv(RForces(:,k), g, 'same');
end
subplot(322);
plot(time,(RForces_f(:,2)+RForces_f(:,4))/(2*M*G),'LineWidth',1.5,'Color','[0 0.4470 0.7410]','LineStyle','-');
hold on 
% plot(xx,GRF2,'LineWidth',1.5,'Color','[0.8 0.1470 0.7410]','LineStyle','--');box off
xticks([]);ylabel('Vertical(N/kg)');box off;
title ('GRF(BW)')
subplot(324);
plot(time,(RForces_f(:,1)+RForces_f(:,3))/(2*M*G),'LineWidth',1.5,'Color','[0 0.4470 0.7410]','LineStyle','-');
hold on 
xlabel('Time(s)');ylabel('Horizontal(N/kg)');box off;


%% Pu (BEC) 
figure(3)
RPu_f(:) = conv(RPu(:), g, 'same');
plot(time,RPu_f/M,'LineWidth',1.5,'Color','[0 0.4470 0.7410]','LineStyle','-')
% axis([0 1.2100 -6 10])
