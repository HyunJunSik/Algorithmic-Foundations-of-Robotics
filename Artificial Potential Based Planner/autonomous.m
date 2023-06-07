clear;
local_car = [0, 5, 5;
             0, 0, 3];

car_global = local_car;
car_centor = car_global(:, 1);

target = [20, 20, 17;
          15, 20, 20;
          ];

obstacle = [10; 12];

theta = 0;
u = zeros(3, 1);

figure;
hold on;

att_scaling = 0.2;
epsilon = 0.5;

figure;
hold on;

for i = 1 : 5000
    
    cla;
    xlim([0, 25]);
    ylim([0, 25]);
    plot(target(1, :), target(2, :), 'bo', 'MarkerSize', 5, 'LineWidth', 1);

    RotZ = @(theta)[
    cosd(theta) -sind(theta) 0 ;
    sind(theta) cosd(theta) 0 ;
    0 0 1 ;
    ];

    [u] = potential_field(att_scaling, epsilon, car_global, target, theta);
  
    car_centor = car_centor + u(1:2);
    theta = theta + u(3);

    R_0_1 = RotZ(theta);
    T_0_1 = vertcat(car_centor, 0);
    H_0_1 = [R_0_1, T_0_1;
             0, 0, 0, 1];

    car_global = H_0_1 * vertcat(local_car, [0, 0, 0; 1, 1, 1]);
    car_global = car_global(1:2, :);

    plot(car_global(1, :), car_global(2, :), 'ro', 'MarkerSize', 5, 'LineWidth', 1);
    drawnow;
    
    if norm(car_global -  target, 2) < 0.3
        break
    end

end

function [u] = potential_field(att_scaling, epsilon, car, target, theta)
    
    u = zeros(3, 1);

    for i = 1:3
        vec = car(:, i) - target(:, i);
        dist = norm(vec, 2);
        
        if dist > epsilon
            F_att = -epsilon * att_scaling * vec / dist;
        else
            F_att = - att_scaling * vec;
        end
           
        J = [                            1,                              0 ;
                                         0,                              1 ;
          -car(1, i)*sind(theta)-car(2, i)*cosd(theta), car(1, i)*cosd(theta)-car(2, i)*sind(theta)];
        
        u = u + J * F_att;

    end
end