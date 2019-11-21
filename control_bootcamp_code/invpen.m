%% Inverted Pendulum 

phi = 3.14;            % 0 at bottom, Pi at Top
init = [0 0 phi 0];    % X, X_dot, Phi, Phi_dot
tspan = [0 100];        % Time Span 

m = 90;                 % Mass of Person [kg]
M = 5;                  % Mass of Wheel [kg]
L = .75;                % Length of Pendulum[m]
g = -9.8;               % Gravity [m/s^2]
b = 15;                  % Damping Constant
M_beam = 2;             % Mass of Pendulum [kg]
I = (1/3)*M_beam*(L^2); % Mass Moment of Inertia of Pendulum [kg/m^2]


[t,y] = ode45(@derivatives,tspan,init);
    
for k=1:length(t)
   drawcartpend_bw(y(k,:),m,M,L);
end

    function out = derivatives(t,y)
    
    % System Parameters
    m = 90;                 % Mass of Person [kg]
    M = 5;                  % Mass of Wheel [kg]
    L = .75;                % Length of Pendulum[m]
    g = -9.8;               % Gravity [m/s^2]
    b = 15;                  % Damping Constant
    M_beam = 2;             % Mass of Pendulum [kg]
    I = (1/3)*M_beam*(L^2); % Mass Moment of Inertia of Pendulum [kg/m^2]
    if t < .001 
        u = 20;
    else
        u = 0;
    end
    
    
    % Update functions for state variables (X, X_dot, Phi, Phi_dot)
    dxdt = y(2);
    dx2dt2 = (u + (m*L)^2*g*y(3)/(I+m*L^2) - b*y(2))/(M+m -m*L/(I+m*L^2));
    dphidt = y(4);
    dphi2dt2 =(u+(g*(M+m)*y(3))-(b*y(2)))/((((M+m)*I)/m*L)+((M+m)*L)-(m*L));
    
    out = [dxdt; dx2dt2; dphidt; dphi2dt2];
    end
