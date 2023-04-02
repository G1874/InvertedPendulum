function dy = pendulum(y,m,M,I,l,g,b,u)
% y = [x,v,phi,omega]
% x - cart position
% v - cart speed
% phi - pendulum angle
% omega - pendulum angular speed
% 
% u - input force
%
% m - pendulum mass
% M - cart mass
% I - pendulum moment of inertia
% l - pendulum length
% g - gravitational acceleration
% b - friction coefficient

Sy = sin(y(3));
Cy = cos(y(3));
Ky = m^2*l^2*Cy^2-(M+m)*(I+m*l^2);

dy(1,1) = y(2);
dy(2,1) = ((I+m*l^2)*(b*y(2)-m*l*y(4)^2*Sy-u)+m^2*l^2*g*Sy*Cy)/Ky;
dy(3,1) = y(4);
dy(4,1) = (u*m*l*Cy-b*m*l*y(2)*Cy-(M+m)*m*l*g*Sy+m^2*l^2*y(4)^2*Sy*Cy)/Ky;

end

