function y=f_track_1_robust(t,z)

global L d k1 k2 k3
%%
x  = z(1);
y  = z(2);
a  = z(3);
x0 = z(4);
y0 = z(5);
a0 = z(6);



%% to generate reference trajectory

% Example 2 
%v0=L(1)+L(2)*sin(L(3)*t);
%w0=L(4)*cos(L(5)*t);

% Example 1 
v0 = L(1);                                                                          % vr
w0 = 1.5 * 3.38321412225 * 0.24 * cos(0.24*t) / (1+(3.38321412225*sin(0.24*t))^2);  % wr

% Other Example
% v0=L(1);
% w0=1.5*atan(3.38321412225*sin(0.24*t));

dx0 = v0 * cos(a0);
dy0 = v0 * sin(a0);
da0 = w0;



%% robot

% Plant
% d : disturbances
xe1 =  cos(a(1))*(x0-x(1) + d(1,1)) + sin(a(1))*(y0-y(1) + d(1,2)); % xe
ye1 = -sin(a(1))*(x0-x(1) + d(1,1)) + cos(a(1))*(y0-y(1) + d(1,2)); % ye
ae1 = a0 - a(1);                                                    % we

% Control Law
%v1 = v0 *1 + k1 * xe1 * sqrt(1+xe1^2+ye1^2)^(-1);
%w1 = w0    + k2 * v0  * sqrt(1+xe1^2+ye1^2)^(-1 )* (ye1*cos(0.5*ae1)-xe1*sin(0.5*ae1)) + k3*sin(0.5*ae1);

% 2021 norminal
%v1 = v0 * cos(ae1) + k1 * xe1;
%w1 = w0 + v0 * (k2 * ye1 + k3 * sin(ae1));

% 2021 robust
phi_t = 0.25 * t;
psi_t = 1.25 * t;
v_dist = ((rand() - 0.5) * 2) * phi_t;  % a random number in [-1, 1]
w_dist = ((rand() - 0.5) * 2) * psi_t;

v1 = v0 * cos(ae1) + k1 * xe1 + phi_t * sign(xe1);
w1 = w0 + v0 * (k2 * ye1 + k3 * sin(ae1)) + psi_t * sign(sin(ae1));

% Plant 2
dx1 = (v1 + v_dist) * cos(a(1));
dy1 = (v1 + v_dist) * sin(a(1));
da1 = w1 + w_dist;


%% ode update
d0=[dx0 dy0 da0]';
y=[dx1;dy1;da1;d0];

end

