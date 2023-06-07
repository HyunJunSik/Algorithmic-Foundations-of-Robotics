clear all;

theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = 0;

% DH table
%         theta       d      a          alpha 
% ----------------------------------------------
%      1 | theta1     7.7      0           90
%      2 | theta2     0      12.8           0
%      3 | theta3     0      12.4           0
%      4 | theta4     0      10.0           0
% ----------------------------------------------

d1 = 7.7;
a2 = 12.8;
a3 = 12.4;
a4 = 10.0;

figure;

h = gcf;
set(h, 'KeyPressFcn', @keyPress);

function keyPress(~, event)
    persistent theta1 theta2 theta3 theta4 d1 a2 a3 a4
    if isempty(theta1)
        theta1 = 0;
        theta2 = 0;
        theta3 = 0;
        theta4 = 0;
        d1 = 7.7;
        a2 = 12.8;
        a3 = 12.4;
        a4 = 10.0;
    end
    
    %% Rotation X, Y, Z matrix

    DH = @(theta, d, a, alpha)[
        cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) a*cosd(theta);
        sind(theta) cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) a*sind(theta);
        0           sind(alpha)              cosd(alpha)              d;
        0           0                        0                        1;
    ];

    step = 5;
    key = event.Key;

    if strcmp(key, 'q')
        theta1 = theta1 + step;
    elseif strcmp(key, 'a')
        theta1 = theta1 - step;
    elseif strcmp(key, 'w')
        theta2 = theta2 + step;
    elseif strcmp(key, 's')
        theta2 = theta2 - step;
    elseif strcmp(key, 'e')
        theta3 = theta3 + step;
    elseif strcmp(key, 'd')
        theta3 = theta3 - step;
    elseif strcmp(key, 'r')
        theta4 = theta4 + step;
    elseif strcmp(key, 'f')
        theta4 = theta4 - step;
    else
        return;
    end
    
    T_0_1 = DH(theta1, d1, 0, 90);
    T_1_2 = DH(theta2, 0, a2, 0);
    T_2_3 = DH(theta3, 0, a3, 0);
    T_3_4 = DH(theta4, 0, a4, 0);
    T_total = T_0_1 * T_1_2 * T_2_3 * T_3_4;
    
    y0 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    y0_1 = T_0_1 * y0;
    y0_2 = T_0_1 * T_1_2 * y0;
    y0_3 = T_0_1 * T_1_2 * T_2_3 * y0;
    y0_4 = T_total * y0;

    X = [y0(1, 4), y0_1(1, 4), y0_2(1, 4), y0_3(1, 4), y0_4(1, 4)];
    Y = [y0(2, 4), y0_1(2, 4), y0_2(2, 4), y0_3(2, 4), y0_4(2, 4)];
    Z = [y0(3, 4), y0_1(3, 4), y0_2(3, 4), y0_3(3, 4), y0_4(3, 4)];
    clf;
    plot3(X, Y, Z, '*-', 'linewidth', 2);
    
    xlim([-60, 60]);
    ylim([-60, 60]);
    zlim([-60, 60]);
    grid on;

    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(3); % 3D view angle
    drawnow;
end