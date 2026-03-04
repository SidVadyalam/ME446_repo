
for t = [0:0.001:6]

if t < 1
    y = -(40*pi/3) + 12*pi*t - 3.5*pi*t.^2 + (pi/3)*t.^3;

    yd = 12*pi - 7*pi*t + pi*t.^2;

    ydd = -7*pi + 2*pi*t;
elseif t < 3
    y = pi/6;

    yd = 0;

    ydd = 0;
elseif t < 4
   y  = (pi/2)*t.^2 - (pi/3)*t.^3;

    yd = pi*t - pi*t.^2;

    ydd = pi - 2*pi*t;
else 
    y  = 0;

    yd = 0;

    ydd = 0;


end

end

plot(t,y,'LineWidth',2)
grid on
xlabel('x')
ylabel('y')
