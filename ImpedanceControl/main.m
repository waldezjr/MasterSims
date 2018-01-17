clear all;
close all;
clc;
clf;

%% Simulation Parameters
Ts = 0.01; % Sample time durat ion
tSim = 10; %Simulation Time
t = 0:Ts:tSim;

%% Robot Parameters
robot = planar3dof();
qr = [0 pi/2 pi/2]; % Ready Pose
robot.teach(qr); % Show Pose
omega = 2*pi/tSim; % Trajectory angular speed

%% Human Impedance Parameters
%Constant Parameters
Kh = [1 0;0 2];
%Variable Parameters
Kh0 = [1 0;0 2];

%% Robot Desired Impedance Parameters
%Constant Parameters
Md = [2 0;0 2];
D = [25 0;0 25];
K = [625 0;0 625];

%Variable Parameters
D0 = [1 0;0 2];
D1 = [1 0;0 2];
K0 = [1 0;0 2];

%% ICC parameters
iccMin = 0.05;
iccMax = 0.40;

%% Human Force

Fh = [0;0];

%% Simulation
q = qr'; %initialization
T = fkine(robot,qr);
xE = T(1:2,4);
xE_dot_old = zeros(2,1);

Xe=[]; XeDot=[]; Q=[]; QDot=[];Xr=[];

teste = [0;0];
for i=1:length(t)
    teste(:,i) = humanTraj(t(i),omega,tSim);
    
    J  = jacob0(robot, q);
    Jp = J([1:2],1:3);
    
    xR = robotTraj(t(i),omega);
    xH = humanTraj(t(i),omega,tSim);
    
    if t(i)> tSim/8 && t(i) < 2 * tSim/8
        Fh = [-5;0];
    else
        Fh = [0;0];
    end
    
    xE_dot = inv(Md/Ts+D)*( Fh + Md*xE_dot_old/Ts - K*(xE -xR) );
    
    q_dot = pinv(Jp)*xE_dot;
    
    q=Ts*q_dot+q;
    
    T = fkine(robot,q);
    xE = T(1:2,4);
    
    xE_dot_old = xE_dot;
    
    Xr(:,i) = xR;
    Xe(:,i) = xE;
    XeDot(:,i) = xE_dot;
    Q(:,i) = q;
    QDot(:,i) = q_dot;
    
end

%% Plot Results

%robot.animate(Q');

figure;
plot(Xe(1,:),Xe(2,:));
hold on;
plot(Xr(1,:),Xr(2,:));
