clear all
close all
clc
global L d k1 k2 k3 ar
tic


%%
% Example 1&2
% 8 < v < 13, w_max = 2
% c1 < v_max - max(v_r)
% 2 * v_r * c2 + c3 < w_max - min(w_r)
L=[10 1 2 0.3 0.3 -0.3];
%k1=2;
%k2=0.1;
%k3=1;

% Parameter Analysis
% Example 1
% c1 < 13 - 10
% 20 * c2 + c3 < 2 - (-1.2180)
k1 = 3;
k2 = 0.450;
k3 = 1.218; 

% 0.5      1.5        3.0   (c2 = 0.075, c3 = 1.218)
% 0.005    0.040      0.075 (c1=3, c3 = 1.218)     0.418      0.818       1.218 (c1 = 3, c2 = 0.040)

d0=0;
d=[0;d0]';
p0=[-16 26 1*pi]';
p00=[1 1 -2*pi]';

%% initial state
z0=[p0;p00];

%% ode with fixed step
k=1e-2; tfinal=30;
options=odeset('reltol',1e-4,'abstol',1e-4);      % ('reltol',1e-12,'abstol',1e-12)
[t,z]=ode45('f_track_1_robust',0:k:tfinal,z0,options); 

% ode ref
[t_ref,z_ref]=ode45('f_track_1_robust_ref',0:k:tfinal,z0,options); 

%% obtain data after ode
n=length(t);

% Execution
for i=1:n

% robust trajectory
    x=z(i,1);
    y=z(i,2);
    a=z(i,3);
    x0=z(i,4);
    y0=z(i,5);
    a0=z(i,6);

    % Example 1 
    v0=L(1);
    w0=1.5*3.38321412225*0.24*cos(0.24*t(i))/(1+(3.38321412225*sin(0.24*t(i)))^2);


    ww0(i)=w0; 
    vv0(i)=v0;
    %1
    xe=cos(a)*(x0-x+d(1,1))+sin(a)*(y0-y+d(1,2));
    ye=-sin(a)*(x0-x+d(1,1))+cos(a)*(y0-y+d(1,2));
    ae=a0-a;

    phi_t = 0.25 * t(i);
    psi_t = 1.25 * t(i);    
    v = v0 * cos(ae) + k1 * xe + phi_t * sign(xe);
    w = w0 + v0 * (k2 * ye + k3 * sin(ae)) + psi_t * sign(sin(ae));


    vv(i,1)=v;
    ww(i,1)=w;
    ex(i,1)=x0-x+d(1,1);
    ey(i,1)=y0-y+d(1,2);
    ea(i,1)=a0-a;
    X(i,1)=x;
    Y(i,1)=y;
    A(i,1)=a;
    X0(i,1)=x0;
    Y0(i,1)=y0;
    YE(i,1)=ye;
    XE(i,1)=xe;

% ref trajectory
    x_ref  = z_ref(i,1);
    y_ref  = z_ref(i,2);
    a_ref  = z_ref(i,3);
    %x0 = z_ref(i,4);
    %y0 = z_ref(i,5);
    %a0 = z_ref(i,6);

    %1
    xe_ref =  cos(a)*(x0-x+d(1,1)) + sin(a)*(y0-y+d(1,2));
    ye_ref = -sin(a)*(x0-x+d(1,1)) + cos(a)*(y0-y+d(1,2));
    ae_ref = a0 - a;

    v_ref = v0 * cos(ae_ref) + k1 * xe_ref;
    w_ref = w0 + v0 * (k2 * ye_ref + k3 * sin(ae_ref));

    vv_ref(i,1) = v_ref;
    ww_ref(i,1) = w_ref;
    ex_ref(i,1) = x0-v_ref+d(1,1);
    ey_ref(i,1) = y0-y_ref+d(1,2);
    ea_ref(i,1) = a0-a_ref;
    X_ref(i,1)  = x_ref;
    Y_ref(i,1)  = y_ref;
    A_ref(i,1)  = a_ref;
    YE_ref(i,1) = ye_ref;
    XE_ref(i,1) = xe_ref;
    
end


%% plot figures
figure(1)
plot(X0,Y0, 'Color', [254 67 101] / 255,'LineWidth',4)
hold on
plot(X,Y, 'Color', [214 200 75] / 255,'LineWidth', 3)
hold on
plot(X_ref,Y_ref, 'Color', [69 137 148] / 255,'LineWidth', 1.25)
hold off
xlabel('X(m)')
ylabel('Y(m)')
legend('reference','vehicle', 'norminal','Location','Northeast')
grid on
% axis([-350 0 -100 150])
% axis equal

figure(2)
subplot(3,1,1)
plot(t,ex,'Color', [214 200 75] / 255,'LineWidth',1)
hold on
plot(t_ref,ex_ref,'Color', [69 137 148] / 255,'LineWidth',1)
hold off
xlabel('Time (sec)')
ylabel('x_r(t)-x(t) (m)')
grid on
legend('vehicle', 'norminal','Location','Northeast')
subplot(3,1,2)
plot(t,ey,'Color', [214 200 75] / 255,'LineWidth',1)
hold on
plot(t_ref,ey_ref,'Color', [214 200 75] / 255,'LineWidth',1)
hold off
xlabel('Time (sec)')
ylabel('y_r(t)-y(t) (m)')
grid on
subplot(3,1,3)
plot(t,ea,'Color', [214 200 75] / 255,'LineWidth',1)
hold on
plot(t,ea_ref,'Color', [69 137 148] / 255,'LineWidth',1)
hold off
xlabel('Time (sec)')
ylabel('\theta_r(t)-\theta(t) (rad)')
%axis([0 30 -1 1])
grid on

figure(3)
subplot(2,1,1)
plot(t,vv,'Color', [214 200 75] / 255,'LineWidth',1)
hold on
plot(t,vv_ref,'Color', [69 137 148] / 255,'LineWidth',1)
hold on
plot(t,vv0,'Color', [254 67 101] / 255,'LineWidth',2)
hold off
xlabel('Time (sec)')
ylabel('Linear velocity (m/s)')
axis([0 30 1 105])
legend('vehicle', 'norminal','reference','Location','Northeast')
grid on
subplot(2,1,2)
plot(t,ww,'Color', [214 200 75] / 255,'LineWidth',1)
hold on
plot(t,ww_ref,'Color', [69 137 148] / 255,'LineWidth',1)
hold on
plot(t,ww0,'Color', [254 67 101] / 255,'LineWidth',2)
hold off
xlabel('Time (sec)')
ylabel('Angular velocity (rad/s)')
% legend('vehicle','reference','Location','Southeast')
grid on



% 
