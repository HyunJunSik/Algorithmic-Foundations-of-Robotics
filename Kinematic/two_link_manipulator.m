theta1 = 0;
theta2 = 0;

a1 = 5;
a2 = 6;

% DH table
%         theta       d      a          alpha 
% ----------------------------------------------
%      1 | theta1     0      a1           0
%      2 | theta2     0      a2           0
% ----------------------------------------------

figure;
% handling plot 
h = gcf;
set(h, 'KeyPressFcn', @keyPress);

function keyPress(~, event)
    persistent theta1 theta2 a1 a2
    if isempty(theta1)
        theta1 = 0;
        theta2 = 0;
        a1 = 5;
        a2 = 6;
    end

    RotZ = @(theta)[
    cosd(theta) -sind(theta) 0 0;
    sind(theta) cosd(theta) 0 0;
    0 0 1 0;
    0 0 0 1;
    ];

    Trans = @(x, y, z) [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];
    step = 5;
    key = event.Key;
    if strcmp(key, 'uparrow')
        theta1 = theta1 + step;
    elseif strcmp(key, 'downarrow')
        theta1 = theta1 - step;
    elseif strcmp(key, 'rightarrow')
        theta2 = theta2 + step;
    elseif strcmp(key, 'leftarrow')
        theta2 = theta2 - step;
    else
        return;
    end

    H_0_1 = RotZ(theta1) * Trans(a1, 0, 0);
    H_1_2 = RotZ(theta2) * Trans(a2, 0, 0);
    H_total = H_0_1 * H_1_2;

    y0 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    y0_1 = H_0_1 * y0;
    y0_2 = H_total * y0;

    X = [y0(1, 4), y0_1(1, 4), y0_2(1, 4)];
    Y = [y0(2, 4), y0_1(2, 4), y0_2(2, 4)];

    clf;
    plot(X, Y, '*-', 'linewidth', 2);
    
    xlim([-15, 15]);
    ylim([-15, 15]);
    zlim([-15, 15]);
    grid on;

    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(3); % 3D view angle
    drawnow;
end


