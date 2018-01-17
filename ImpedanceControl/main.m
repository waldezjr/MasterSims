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

Xe=[]; XeDot=[]; Q=[]; QDot=[];Xr=[]; Eh=[]; Er=[];

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
    
    %admittance controller
    xE_dot = inv(Md/Ts+D)*( Fh + Md*xE_dot_old/Ts - K*(xE -xR) );
    
    %kinematic controller
    q_dot = pinv(Jp)*xE_dot;
    
    %integrate joint position
    q=Ts*q_dot+q;
    
    %get end-effector position
    T = fkine(robot,q);
    xE = T(1:2,4);
    
    xE_dot_old = xE_dot;
    
    Xr(:,i) = xR;
    Xe(:,i) = xE;
    Xh(:,i) = xH;
    XeDot(:,i) = xE_dot;
    Q(:,i) = q;
    QDot(:,i) = q_dot;
    Er(:,i) = norm(xE - xR);
    Eh(:,i) = norm(xE - xH);
    
end

%% Plot Results

%robot.animate(Q');

figure;
plot(Xe(1,:),Xe(2,:));
hold on;
plot(Xr(1,:),Xr(2,:));
hold on;
plot(Xh(1,:),Xh(2,:));
legend('x_E','x_R','x_H');
ylabel('Y_b_a_s_e');
xlabel('X_b_a_s_e');

figure;
plot(t,Eh);
hold on
plot(t,Er);
legend('Eh','Er');
xlabel('time')
ylabel('m');

figure;
plot(t,Q)
xlabel('time');
ylabel('rad');
legend('q_1','q_2','q_3')

figure;
plot(t,QDot)
xlabel('time');
ylabel('rad/s');
legend('qDot_1','qDot_2','qDot_3')
