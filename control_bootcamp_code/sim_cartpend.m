clear all, close all, clc

m = 90;
M = 5;
L = .75;
g = -10;
d = 1;

tspan = 0:.1:100;
y0 = [0; 0; 2.7909; 0];
[t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,0),tspan,y0);

for k=1:length(t)
    drawcartpend_bw(y(k,:),m,M,L);
end

% function dy = pendcart(y,m,M,L,g,d,u)