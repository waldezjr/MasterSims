clear all;
close all;
clc;
clf;

%% Simulation Parameters
Ts = 0.01; % Sample time duration
tSim = 10; %Simulation Time
t = 0:Ts:tSim+1;

%% Robot Parameters
robot = planar3dof();
qr = [0 pi/2 pi/2]; % Ready Pose
%robot.teach(qr); % Show Pose
omega = 2*pi/tSim; % Trajectory angular speed

%% Kinematic Controller
Kkin = [50 0;0 50];

%% Human Impedance Parameters
%Constant Parameters
Kh = [500 0;0 500];
%Variable Parameters
Kh0 = Kh;
%VERIFY INITIAL VALUE

%% Robot Desired Impedance Parameters
%Constant Parameters
Md = [2 0;0 2];
D = [25 0;0 25];
Kd = [625 0;0 625];

%Variable Parameters
D0 = [1 0;0 2];
D1 = [1 0;0 2];
Kd0 = Kd;
%VERIFY INITIAL VALUE

%% ICC parameters
iccMin = 0.05;
iccMax = 0.40;

%% Human Force

Fh = [0;0];

alpha = 0.5;
%START VARYING IT WITH TIME
%TRY TO MAKE UNSTABLE VIBRATIONS APPEAR

%% Simulation
q = qr'; %initialization
T = fkine(robot,qr);
xE = T(1:2,4);

xRef=xE;
xRef_dot_old = zeros(2,1);

Xe=[]; XeDot=[]; Q=[]; QDot=[];Xr=[]; Eh=[]; Er=[]; Ekin=[]; Xref=[];

teste = [0;0];
for i=1:length(t)
    teste(:,i) = humanTraj(t(i),omega,tSim);
    
    J  = jacob0(robot, q);
    Jp = J([1:2],1:3);
    
    xR = robotTraj(t(i),omega,tSim);
    xH = humanTraj(t(i),omega,tSim);
    
    %if t(i)> tSim/8 && t(i) < 3 * tSim/8
        %Fh = [-50;0];
     %   Fh = -Kh*(xE - xH);
    %elseif t(i)> 3*tSim/8 && t(i) < 5 * tSim/8
        %Fh = [0;5];
    %else
     %   Fh = [0;0];
    %end
    
    Kd = Kd0 * (1-alpha);
    Kh = Kh0 * alpha;
    
    Fh = -Kh*(xE - xH);
    
    %admittance controller
    xRef_dot = inv(Md/Ts+D)*( Fh + Md*xRef_dot_old/Ts - Kd*(xE -xR) ); 
    xRef = Ts*xRef_dot+ xRef;
    
    %kinematic controller
    q_dot = pinv(Jp)*(xRef_dot+Kkin*(xRef-xE));
    
    %integrate joint position
    q=Ts*q_dot+q;
    
    %get end-effector position
    T = fkine(robot,q);
    xE = T(1:2,4);
    
    xRef_dot_old = xRef_dot;
    
    Xr(:,i) = xR;
    Xe(:,i) = xE;
    Xh(:,i) = xH;
    Xref(:,i) = xRef;
    Q(:,i) = q;
    QDot(:,i) = q_dot;
    Er(:,i) = norm(xE - xR);
    Eh(:,i) = norm(xE - xH);
    Ekin(:,i) = norm(xRef-xE);
    
end

%% Plot Results

%robot.animate(Q');

%figure;
plot(Xe(1,:),Xe(2,:));
hold on;
plot(Xr(1,:),Xr(2,:));
hold on;
plot(Xh(1,:),Xh(2,:));
hold on;
plot(Xref(1,:),Xref(2,:));
legend('x_E','x_R','x_H','X_r_e_f');
ylabel('Y_b_a_s_e');
xlabel('X_b_a_s_e');

figure;
plot(t,Eh);
hold on
plot(t,Er);
hold on;
plot(t,Ekin);
legend('Eh','Er','Ekin');
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
