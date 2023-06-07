clear all;

% local minimum 확인하셨다면, 중지 버튼 눌러서 프로그램 강제 종료해주세요.

robot = [5; 50];
goal = [95; 50];

obstacle = [50, 40, 40;
            50, 60, 40];


epsilon = 8;
att_scaling = 0.05;
rep_scaling = 100;

obstacle_dist = 15;

figure;
hold on;

plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
xlim([0, 100]);
ylim([0, 100]);
plot(obstacle(:, 1), obstacle(:, 2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);

for i = 2:1000
    cla;
    plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    xlim([0, 100]);
    ylim([0, 100]);
    plot(obstacle(1, :), obstacle(2, :), 'bo', 'MarkerSize', 10, 'LineWidth', 2);

    [F_att, F_rep] = potential_field(rep_scaling, att_scaling, obstacle_dist, epsilon, robot, goal, obstacle);

    F = F_att + F_rep;

    robot = robot + F;

    plot(robot(1), robot(2), 'ro');
    drawnow;

    if norm(robot - goal, 2) < 0.1
        break;
    end
    
end

function [F_att, F_rep] = potential_field(rep_scaling, att_scaling, obstacle_dist, epsilon, robot, goal, obstacle)
    
    % Definition attrative function U
    tmp_vec = robot - goal;
    dist = norm(tmp_vec, 2);

    % define attractive potential function U & force field
    if dist > epsilon
        % c1 case
        F_att = - epsilon * att_scaling * tmp_vec / dist;
    else
        % c2 case
        F_att = - att_scaling * tmp_vec;
    end

    F_rep = [0; 0];

    % define repulsive potential function U_rep
    for obs = 1:3
        obs_vec = robot - obstacle(:, obs);

        obs_dist = norm(obs_vec, 2);

        if obs_dist < obstacle_dist
            % c3 case
            F_rep = F_rep + rep_scaling * (1/obs_dist - 1/obstacle_dist) * (1 / (obs_dist^2)) * obs_vec / obs_dist;
        end
    end
end