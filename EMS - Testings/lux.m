clc; clear; close all;

% PARAMETERS
I  = 954.92;      % luminous intensity (candela)
h  = 6;         % lamp height (m)
d  = 8;        % spacing from center (square half length)

% GRID (ground plane)
x = -30:0.2:30;
y = -30:0.2:30;
[X,Y] = meshgrid(x,y);

% LAMP POSITIONS (square)
lamps = [ -d -d;
           d -d;
           d  d;
          -d  d];

E_total = zeros(size(X));

% ILLUMINANCE CALCULATION
for k = 1:size(lamps,1)

    xk = lamps(k,1);
    yk = lamps(k,2);

    r = sqrt((X-xk).^2 + (Y-yk).^2 + h^2);

    % inverse square + cosine law
    E = (I*h) ./ (r.^3);

    E_total = E_total + E;

end

% 3D SURFACE PLOT
figure
surf(X,Y,E_total)

shading interp
xlabel('x (m)')
ylabel('y (m)')
zlabel('Illuminance (lux)')
title('3D Illuminance Distribution - 4 Point Lamps')

colorbar
view(45,30)
grid on

% Iso-lux CONTOUR PLOT
figure
contourf(X,Y,E_total,15)
colorbar
xlabel('x (m)')
ylabel('y (m)')
title('Iso-lux Contour Map')
axis equal
