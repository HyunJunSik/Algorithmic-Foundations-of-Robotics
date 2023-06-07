theta1 = 0;
theta2 = 0;
theta3 = 0;

d1 = 8;
d2 = 6;
d3 = 6;

figure;

drill_theta = 0;

h = gcf;
set(h, 'KeyPressFcn', @keyPress);

function keyPress(~, event)
    persistent theta1 theta2 theta3 d1 d2 d3 drill_theta
    if isempty(theta1)
        theta1 = 0;
        theta2 = 0;
        theta3 = 0;

        d1 = 8;
        d2 = 6;
        d3 = 6;
        drill_theta = 0;
    end

    DH = @(theta, d, a, alpha)[
        cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) a*cosd(theta);
        sind(theta) cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) a*sind(theta);
        0           sind(alpha)              cosd(alpha)              d;
        0           0                        0                        1;
    ];  

    RotZ = @(theta)[
    cosd(theta) -sind(theta) 0 ;
    sind(theta) cosd(theta) 0 ;
    0 0 1 ;
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
        drill_theta = drill_theta + 20;
    else
        return;
    end

    T_0_1 = DH(theta1, d1, 0, 90);
    T_1_2 = DH(theta2, 0, d2, 0);
    T_2_3 = DH(theta3, 0, d3, 0);
    T_total = T_0_1 * T_1_2 * T_2_3;

    end_position = T_total(1:3, 4);

    drill_base = end_position;

    triangle_points = [-1 1 0; -1 -1 0; 0 0 -5];
    rotate_traingle_points = RotZ(drill_theta) * triangle_points;
    rotate_traingle_points2 = RotZ(drill_theta + 90) * triangle_points;
    rotate_traingle_points3 = RotZ(drill_theta + 180) * triangle_points;
    rotate_traingle_points4 = RotZ(drill_theta + 270) * triangle_points;


    translate = rotate_traingle_points + drill_base;
    translate2 = rotate_traingle_points2 + drill_base;
    translate3 = rotate_traingle_points3 + drill_base;
    translate4 = rotate_traingle_points4 + drill_base;

    link_0_1 = T_0_1(1:3, 4);
    link_1_2 = T_0_1 * T_1_2;
    link_1_2 = link_1_2(1:3, 4);
    link_2_3 = T_total(1:3, 4);

    clf;

    hold on;
    plot3([0, link_0_1(1), link_1_2(1), link_2_3(1)], ...
      [0, link_0_1(2), link_1_2(2), link_2_3(2)], ...
      [0, link_0_1(3), link_1_2(3), link_2_3(3)], 'o-', 'linewidth', 2);
    hold on;
    fill3(translate(1, :), translate(2, :), translate(3, :), 'r');
    fill3(translate2(1, :), translate2(2, :), translate2(3, :), 'r');
    fill3(translate3(1, :), translate3(2, :), translate3(3, :), 'r');
    fill3(translate4(1, :), translate4(2, :), translate4(3, :), 'r');

    xlim([-15, 15]);
    ylim([-15, 15]);
    zlim([-15, 15]);
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(3);
    hold off;
    drawnow;
end