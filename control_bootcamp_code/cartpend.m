% function dy = cartpend(y,m,M,L,g,d,u)
% 
% Sy = sin(y(3));
% Cy = cos(y(3));
% D = m*L*L*(M+m*(1-Cy^2));
% 
% dy(1,1) = y(2);
% dy(2,1) = (1/D)*(-m^2*L^2*g*Cy*Sy + m*L^2*(m*L*y(4)^2*Sy - d*y(2))) + m*L*L*(1/D)*u;
% dy(3,1) = y(4);
% dy(4,1) = (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*y(4)^2*Sy - d*y(2))) - m*L*Cy*(1/D)*u +.01*randn;

function dy = cartpend(y,m,M,L,g,b,u,I)

% Sy = sin(y(3));
% Cy = cos(y(3));
% q = [(M+m)*(I+m*L^2)-(m*L)^2];

dy(1,1) = y(2);
dy(2,1) = (u + (m*L)^2*g*y(3)/(I+m*L^2) - b*y(2))/(M+m -m*L/(I+m*L^2));
dy(3,1) = y(4);
dy(4,1) = (u+(g*(M+m)*y(3))-(b*y(2)))/((((M+m)*I)/m*L)+((M+m)*L)-(m*L));