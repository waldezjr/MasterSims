clear all;
close all;
clc;
clf;
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

%% Simulation Parameters
Ts = 0.001; % Sample time duration
tSim = 10; %Simulation Time
t = 0:Ts:tSim+1;

%% Robot Parameters
robot = planar3dof();
qr = [0 pi/2 pi/2]; % Ready Pose
robot.teach(qr); % Show Pose
omega = 2*pi/tSim; % Trajectory angular speed
set(gca,'FontSize',25)
set(findall(gcf,'type','text'),'FontSize',25)


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
% Kicc = 20;

alpha = 0.5;
%% Lyapunov Fn Parameter
gamma = min(eig(D)) / max(eig(Md)) - 1;

KdOld = Kd;



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
    
%     alpha variation for simulation
    
%     icc = Kicc * norm(xR-xH) + iccMin;
    
    sigmoid =(iccMax-iccMin) * 1/(1+exp(-(600*norm(xR-xH)-6))) + iccMin;
    
    icc = sigmoid;
    
    if icc > iccMax
        icc = iccMax;
    end
    
%     alpha = (icc - iccMin) / (iccMax - iccMin);
    
    %External disturbances    
    if t(i)> 2 * tSim/8 && t(i) < 3 * tSim/8
       % Fh = [-1;0];
    elseif t(i)> 3*tSim/8 && t(i) < 5 * tSim/8
       % Fh = [0;0];
    else
       % Fh = [0;0];
    end
    
    %Variation of Stiffnesses
    Kd = Kd0 * (1-alpha) + 10*eye(2); 
    Kh = Kh0 * alpha;
    
    %Human Spring
    Fh = -Kh*(xE - xH);

    
    %admittance controller
    %xRef_dot = inv(Md/Ts+D)*( Fh + Md*xRef_dot_old/Ts - Kd*(xE -xR) ); 
    %xRef = Ts*xRef_dot+ xRef;
    
    %even newer admittance controller block implementation
%     xRef_dot_dot = (1-alpha) * xRDotDot + inv(Md)*( Fh -D*(xEDot - (1-alpha)*xRDot ) - Kd*(xE - xR) );
    xRef_dot_dot = xRDotDot + inv(Md)*( Fh -D*(xEDot - xRDot ) - Kd*(xE - xR) );
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
    
    %Calculating Lyapunov function
    beta = Kd + gamma*D -gamma^2*Md;
    v = ((xEDot - xRDot) + gamma*(xE - xR))' * Md * ((xEDot - xRDot) + gamma*(xE - xR))* 0.5 + ((xE - xR)'*beta*(xE - xR))*0.5; 
    
    %Calculating stability conditions
    
    f1 = (Kd(4) - KdOld(4)) / Ts ;
    f2 = 2*gamma *Kd(4);    
    KdOld = Kd;
    
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
    SIG(i) = sigmoid;
    V(i) = v;
    F1(i) = f1;
    F2(i) = f2;
end

%% Plot Results

%robot.animate(Q');

figure('pos',[10 10 800 600]);


plot(Xr(1,:),Xr(2,:),'LineWidth',2);
hold on;
plot(Xh(1,:),Xh(2,:),'LineWidth',2);
hold on;
plot(Xe(1,:),Xe(2,:),'LineWidth',2);
hold on;
% plot(Xref(1,:),Xref(2,:));
lgd = legend('$x_r^d(t)$','$x_h^d(t)$','$x_e$');
% legend('$x_r^d(t)$','$x_h^d(t)$');
title('Task Trajectories')
ylabel('y_{b}(m)');
xlabel('x_{b}(m)');
axis([-0.20 0.15 0.25 0.55])
set(gca,'FontSize',25)
set(findall(gcf,'type','text'),'FontSize',25)
% lgd.FontSize = 50;
% 
figure('pos',[10 10 800 600]);
plot(t,Eh,'LineWidth',2);
hold on
plot(t,Er,'LineWidth',2);
% hold on;
% plot(t,Ekin,'LineWidth',2);
lgd = legend('$\Vert e_h^d(t)\Vert$','$\Vert e_r^d(t)\Vert$');
xlabel('time(s)')
ylabel('Error norm (m)');
title('Error Norms - \alpha(t) = 0.5')
set(gca,'FontSize',25)
set(findall(gcf,'type','text'),'FontSize',25)
% lgd.FontSize = 50;
% 
figure('pos',[10 10 800 600]);
plot(t,Q,'LineWidth',2)
xlabel('time(s)');
ylabel('rad');
title('Joint Angles - \alpha(t) = 0.5')
lgd = legend('$q_1$','$q_2$','$q_3$');
set(gca,'FontSize',25)
set(findall(gcf,'type','text'),'FontSize',25)
% lgd.FontSize = 50;
% 
figure('pos',[10 10 800 600]);
plot(t,QDot,'LineWidth',2)
xlabel('time(s)');
ylabel('rad/s');
lgd = legend('$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$');
set(gca,'FontSize',25)
set(findall(gcf,'type','text'),'FontSize',25)
title('Joint Velocities - \alpha(t) = 0.5 ')
% lgd.FontSize = 50;
% 
figure('pos',[10 10 800 600]);
plot(t,Alpha,t,ICC,'LineWidth',2);
xlabel('time(s)');
ylabel('');
lgd = legend('$\alpha(t)$', 'icc(t)');
title('\alpha(t) and icc(t)')
set(gca,'FontSize',25)
set(findall(gcf,'type','text'),'FontSize',25)
% lgd.FontSize = 50;
% 
figure('pos',[10 10 800 600]);
plot(t,V,'LineWidth',2);
xlabel('time(s)');
ylabel('J');
title('Lyapunov Function V_2(t)')
lgd = legend('$V_2(t)$');
set(gca,'FontSize',25)
set(findall(gcf,'type','text'),'FontSize',25)
% lgd.FontSize = 50;
% 
figure('pos',[10 10 800 600]);
plot(t,F1,t,F2,'LineWidth',2);
xlabel('time(s)');
ylabel('N/m.s');
lgd = legend('$\dot{k}_y$', '$2 \gamma k_y$');
title('Stability Condition')
set(gca,'FontSize',25)
set(findall(gcf,'type','text'),'FontSize',25)
axis([0 12 -10000 40000])
% lgd.FontSize = 50;
% 
