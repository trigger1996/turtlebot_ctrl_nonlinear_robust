function y=f_track_1_adaptive(t,z)

global L d k1 k2 k3  v_dist w_dist
%%
x  = z(1);      % x
y  = z(2);      % y
a  = z(3);      % w
x0 = z(4);      % x_r
y0 = z(5);      % y_r
a0 = z(6);      % w_r

v_hat = z(7);
w_hat = z(8);



%% to generate reference trajectory

% Example 2 
%v0=L(1)+L(2)*sin(L(3)*t);
%w0=L(4)*cos(L(5)*t);

% Example 1 
vr = L(1);                                                                          % vr
wr = 1.5 * 3.38321412225 * 0.24 * cos(0.24*t) / (1+(3.38321412225*sin(0.24*t))^2);  % wr

% Other Example
% v0=L(1);
% w0=1.5*atan(3.38321412225*sin(0.24*t));

dx0 = vr * cos(a0);     % dx_r
dy0 = vr * sin(a0);     % dy_r
da0 = wr;               % dw_r



%% robot

% Plant
xe1 =  cos(a(1))*(x0-x(1) + d(1,1)) + sin(a(1))*(y0-y(1) + d(1,2)); % xe
ye1 = -sin(a(1))*(x0-x(1) + d(1,1)) + cos(a(1))*(y0-y(1) + d(1,2)); % ye
ae1 = a0 - a(1);                                                    % we

% Control Law
% Yu2015
%v1 = v0 *1 + k1 * xe1 * sqrt(1+xe1^2+ye1^2)^(-1);
%w1 = w0    + k2 * v0  * sqrt(1+xe1^2+ye1^2)^(-1 )* (ye1*cos(0.5*ae1)-xe1*sin(0.5*ae1)) + k3*sin(0.5*ae1);

% 2021_norimal
%uv = vr * cos(ae1) + k1 * xe1;
%uw = wr + vr * (k2 * ye1 + k3 * sin(ae1));

% 2021_adaptive
d_v_hat = -xe1;
d_w_hat = -sin(ae1) / k2;

uv = vr * cos(ae1) + k1 * xe1             - v_hat;
uw = wr + vr * (k2 * ye1 + k3 * sin(ae1)) - w_hat;

% Plant 2
dx1 = (uv + v_dist) * cos(a(1));
dy1 = (uv + v_dist) * sin(a(1));
da1 = uw + w_dist;


%% ode update
d0=[dx0 dy0 da0]';
d_est = [d_v_hat d_w_hat]';
y=[dx1;dy1;da1;d0;d_est];

end

