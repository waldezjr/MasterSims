clear all;
close all;
clc;
clf;

%% Simulation Parameters
Ts = 0.001; % Sample time duration
tSim = 10; %Simulation Time
t = 0:Ts:tSim+1;

%% Robot Parameters
robot = planar3dof();
qr = [0 pi/2 pi/2]; % Ready Pose
%robot.teach(qr); % Show Pose
omega = 2*pi/tSim; % Trajectory angular speed

%% Kinematic Controller
Kkin = 50 * eye(2);

%% Human Impedance Parameters
%Constant Parameters
Kh = 2000 * eye(2);
%Variable Parameters
Kh0 = Kh;
%VERIFY INITIAL VALUE

%% Robot Desired Impedance Parameters
%Constant Parameters
Md = 2*eye(2);
D  = 32*eye(2);
Kd = 1000*eye(2);

%Variable Parameters
D0 = 1*eye(2);
D1 = 1*eye(2);
Kd0 = Kd;
%VERIFY INITIAL VALUE

%% Human Force
Fh = [0;0];

%% ICC Simulation Parameters

iccMin = 0.05;
iccMax = 0.50;
Kicc = 25;

alpha = 0;

%% Simulation
q = qr'; %initialization
T = fkine(robot,qr);
xE = T(1:2,4);

xRef=xE;
xRef_dot_old = zeros(2,1);
xRef_dot = zeros(2,1);
xEDot = zeros(2,1);

xR_old = xE;

Xe=[]; XeDot=[]; Q=[]; QDot=[];Xr=[]; Eh=[]; Er=[]; Ekin=[]; Xref=[]; Alpha=[];
ICC=[];

for i=1:length(t)
    
    J  = jacob0(robot, q);
    Jp = J([1:2],1:3);
       
    [xR,xRDot,xRDotDot] = robotTraj(t(i),omega,tSim);
    xH = humanTraj(t(i),omega,tSim);
    
    %alpha variation for simulation
    
    icc = Kicc * norm(xR-xH) + iccMin;
    
    if icc > iccMax
        icc = iccMax;
    end
    
    alpha = (icc - iccMin) / (iccMax - iccMin);
    
    %External disturbances    
    if t(i)> 2 * tSim/8 && t(i) < 3 * tSim/8
       % Fh = [-1;0];
    elseif t(i)> 3*tSim/8 && t(i) < 5 * tSim/8
       % Fh = [0;0];
    else
       % Fh = [0;0];
    end
    
    %Variation of Stiffnesses
    Kd = Kd0 * (1-alpha);
    Kh = Kh0 * alpha;
    
    %Human Spring
    Fh = -Kh*(xE - xH);
    
    %Fh = Fh*0.1*exp(-0.1*Ts);
    
    %admittance controller
    %xRef_dot = inv(Md/Ts+D)*( Fh + Md*xRef_dot_old/Ts - Kd*(xE -xR) ); 
    %xRef = Ts*xRef_dot+ xRef;
    
    %even newer admittance controller block implementation
    xRef_dot_dot = (1-alpha) * xRDotDot + inv(Md)*( Fh -D*(xEDot - (1-alpha)*xRDot ) - Kd*(xE - xR) );
        %integrate xRef_dot_dot, and xRef_dot
    xRef_dot = Ts * xRef_dot_dot + xRef_dot;
    xRef = Ts*xRef_dot+ xRef;
    
    %kinematic controller
    q_dot = pinv(Jp)*(xRef_dot+Kkin*(xRef-xE));
    
    %integrate joint position
    q=Ts*q_dot+q;
    
    %get end-effector position
    T = fkine(robot,q);
    xE = T(1:2,4);
    
    %get end-effector velocity
    xEDot = Jp * q_dot;
    
    xRef_dot_old = xRef_dot;
    
    xR_old = xR;
    
    Xr(:,i) = xR;
    Xe(:,i) = xE;
    Xh(:,i) = xH;
    Xref(:,i) = xRef;
    Q(:,i) = q;
    QDot(:,i) = q_dot;
    Er(:,i) = norm(xE - xR);
    Eh(:,i) = norm(xE - xH);
    Ekin(:,i) = norm(xRef-xE);
    Alpha(i) = alpha;
    ICC(i) = icc;    
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
%plot(PMinJerk(1,:),PMinJerk(2,:));
legend('x_E','x_R','x_H','X_r_e_f');
ylabel('Y_b_a_s_e');
xlabel('X_b_a_s_e');

% figure;
% plot(t,PMinJerk(1,:),t,PMinJerk(2,:));
% hold on;
% plot(t,AMinJerk(1,:),t,AMinJerk(2,:));
% hold on;
% plot(t,VMinJerk(1,:),t,VMinJerk(2,:));
% legend('p_M_J_X','p_M_J_Y','a_M_J_X','a_M_J_Y','v_M_J_X','v_M_J_Y');
% xlabel('time (s)');
% %ylabel('m/s');

figure;
plot(t,Eh);
hold on
plot(t,Er);
hold on;
plot(t,Ekin);
legend('Eh','Er','Ekin');
xlabel('time(s)')
ylabel('m');

figure;
plot(t,Q)
xlabel('time(s)');
ylabel('rad');
legend('q_1','q_2','q_3')

figure;
plot(t,QDot)
xlabel('time(s)');
ylabel('rad/s');
legend('qDot_1','qDot_2','qDot_3')

figure;
plot(t,Alpha,t,ICC);
xlabel('time(s)');
ylabel('%');
legend('Alpha', 'icc');

