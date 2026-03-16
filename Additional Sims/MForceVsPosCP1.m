% Magnetic Levitator Force vs Position Plot

clear; clc;

% Physical constants
mu0 = 4*pi*1e-7;     % permeability of free space (H/m)

% System parameters (choose reasonable example values)
N = 500;             % number of coil turns
I = 1.5;             % coil current (A)
R = 0.01;            % coil radius (m)
md = 0.5;           % magnetic dipole moment (A*m^2)
m = 0.01;            % magnet mass (kg)
g = 9.81;            % gravity (m/s^2)

% Position range (distance from coil center)
x = linspace(0.002,0.05,1000);  % meters

% Magnetic force equation
F_coil = (3*mu0*N*I*R^2*md.*x) ./ (2*(x.^2 + R^2).^(5/2));

% Gravity force
F_gravity = m*g*ones(size(x));

% Plot
figure
plot(x, F_coil,'LineWidth',2)
hold on
plot(x, F_gravity,'--','LineWidth',2)

xlabel('Magnet Position x (m)')
ylabel('Force (N)')
title('Magnetic Force vs Position for Open-Loop Magnetic Levitation')
legend('Magnetic Force from Coil','Gravitational Force')
grid on

% Optional: mark approximate equilibrium point
[~,idx] = min(abs(F_coil - F_gravity));
xe = x(idx);
Fe = F_coil(idx);

plot(xe,Fe,'ro','MarkerSize',8,'MarkerFaceColor','r')
text(xe,Fe,['  x_e = ' num2str(xe,'%0.4f') ' m'])