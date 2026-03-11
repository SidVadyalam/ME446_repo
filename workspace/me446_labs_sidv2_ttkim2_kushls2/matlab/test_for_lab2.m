pi = 3.1415;

y = [];
yd = [];
ydd = [];

for t = 0:0.001:6
    if t < 1
    y(end + 1) = (pi/2)*t*t - (pi/3)*t*t*t;

    yd(end + 1) = (pi)*t - (pi)*t*t;

    ydd(end + 1) = pi - 2*pi*t;
elseif t < 3
    y(end + 1) = pi/6;

    yd(end + 1) = 0;

    ydd(end + 1) = 0;
elseif t < 4
    y(end + 1)  = ((-40*pi)/3) + (12*pi*t) + (-3.5*pi*t*t) + (pi/3)*t*t*t;

    yd(end + 1) = 12*pi -7*pi*t + pi*t*t;

    ydd(end + 1) = -7*pi + 2*pi*t;
else 
    y(end + 1)  = 0;

    yd(end + 1) = 0;

    ydd(end + 1) = 0;

    end




end

t = [0:0.001:6]
hold on

plot(t,y,'LineWidth',2)
plot(t,yd,'LineWidth',2)
plot(t,ydd,'LineWidth',2)
grid on
xlabel('x')
ylabel('y')