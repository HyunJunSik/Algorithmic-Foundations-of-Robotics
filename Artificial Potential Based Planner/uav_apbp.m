clear;

uav = [-1, -1, 2;
        2, -2, 0;
        0,  0, 0];
uav_global = uav;
uav_center = uav(:, 1);
target = [11, 11, 13;
             13,    10,    10;
             10,   10,   10];
theta1 = 0;
pi1 = 0;
psi1 = 0;
epsilon = 1;
att_scaling = 0.01;

RotZ = @(theta)[
    cosd(theta) -sind(theta) 0 ;
    sind(theta) cosd(theta) 0 ;
    0 0 1 ;
    ];
RotX = @(theta)[
    1 0 0 ;
    0 cosd(theta) -sind(theta) ;
    0 sind(theta) cosd(theta) ;
    ];

RotY = @(theta)[
    cosd(theta) 0 sind(theta) ;
    0 1 0;
    -sind(theta) 0 cosd(theta);
    ];

u = zeros(6, 1);
figure;
for i = 1 : 1000
    
    cla;

    plot3(target(1, :), target(2, :), target(3, :), 'bo', 'MarkerSize', 5, 'LineWidth', 1);
    u = potential_field(uav, target, epsilon, att_scaling, theta1, pi1, psi1);
    uav_center = uav_center + u(1:3);

    theta1 = theta1 + u(4);
    pi1 = pi1 + u(5);
    psi1 = psi1 + u(6);

    R = RotZ(theta1) * RotY(pi1) * RotX(psi1);
    T = uav_center;
    H = [R, T; 0, 0, 0, 1];
    uav_global = H * vertcat(uav, [1, 1, 1]);
    uav_global = uav_global(1:3, :);
    
    plot3(uav_global(1, :), uav_global(2, :), uav_global(3, :), 'ro', 'MarkerSize', 5, 'LineWidth', 1);
    view(3);
    xlim([-5, 30]);
    ylim([-5, 30]);
    zlim([-5, 30]);
    grid on;
    drawnow;
    

    if norm(uav - target, 2) < 0.3
        break
    end
    
end

function [u] = potential_field(uav, target, epsilon, att_scaling, theta1, pi1, psi1)
    
    J1 = @(ax, theta1, pi1, psi1)[1, 0, 0, -ax*((pi*cos((pi*psi1)/180)*sin((pi*theta1)/180))/180 + (pi*cos((pi*theta1)/180)*sin((pi*pi1)/180)*sin((pi*psi1)/180))/180), -(ax*pi*cos((pi*pi1)/180)*sin((pi*psi1)/180)*sin((pi*theta1)/180))/180, -ax*((pi*cos((pi*theta1)/180)*sin((pi*psi1)/180))/180 + (pi*cos((pi*psi1)/180)*sin((pi*pi1)/180)*sin((pi*theta1)/180))/180)];
    J2 = @(ay, theta1, pi1, psi1)[0, 1, 0,  ay*((pi*cos((pi*psi1)/180)*cos((pi*theta1)/180))/180 - (pi*sin((pi*pi1)/180)*sin((pi*psi1)/180)*sin((pi*theta1)/180))/180),  (ay*pi*cos((pi*pi1)/180)*cos((pi*theta1)/180)*sin((pi*psi1)/180))/180, -ay*((pi*sin((pi*psi1)/180)*sin((pi*theta1)/180))/180 - (pi*cos((pi*psi1)/180)*cos((pi*theta1)/180)*sin((pi*pi1)/180))/180)];
    J3 = @(az, theta1, pi1, psi1)[0, 0, 1,                                                                                                                           0,                       (az*pi*sin((pi*pi1)/180)*sin((pi*psi1)/180))/180,                                                                           -(az*pi*cos((pi*pi1)/180)*cos((pi*psi1)/180))/180];
    u = zeros(6, 1);
    for i = 1 : 3
        x = uav(1, i);
        y = uav(2, i);
        z = uav(3, i);

        vec = uav(:, i) - target(:, i);
        dist = norm(vec, 2);

        if dist > epsilon
            F_att = -epsilon * att_scaling * vec / dist;
        else
            F_att = - att_scaling * vec;
        end
       
        j1 = J1(x, theta1, pi1, psi1);
        j2 = J2(y, theta1, pi1, psi1);
        j3 = J3(z, theta1, pi1, psi1);
        j = vertcat(j1, j2, j3);
        u = u + transpose(j) * F_att;
    end

end
