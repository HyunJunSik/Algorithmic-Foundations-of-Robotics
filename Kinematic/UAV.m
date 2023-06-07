%% 삼각형 그리기
d = [0; 0; 0];

p1 = [-1, 2, 0];
p2 = [-1, -2, 0];
p3 = [2, 0, 0];
%%
% 필요한 변수 -> theta, pi, psi, dx, dy, dz, p1, p2, p3

theta = 0;
pi = 0;
psi = 0;
dx = 0;
dy = 0;
dz = 0;

figure;
h = gcf;
set(h, 'KeyPressFcn', @keyPress);

function keyPress(~, event)
    persistent theta pi psi dx dy dz p1 p2 p3
    if isempty(theta)
        theta = 0;
        psi = 0;
        pi = 0;
        dx = 0;
        dy = 0;
        dz = 0;
        p1 = [-1, 2, 0];
        p2 = [-1, -2, 0];
        p3 = [2, 0, 0];
    end

    step = 10;
    go = 0.5;
    key = event.Key;
    % keyboard로 입력받아 각도 변환 및 위치 변환 수행
    if strcmp(key, 'q')
        theta = theta + step;
    elseif strcmp(key, 'a')
        theta = theta - step;
    elseif strcmp(key, 'w')
        pi = pi + step;
    elseif strcmp(key, 's')
        pi = pi - step;
    elseif strcmp(key, 'e')
        psi = psi + step;
    elseif strcmp(key, 'd')
        psi = psi - step;
    elseif strcmp(key, 'r')
        dx = dx + go;
    elseif strcmp(key, 'f')
        dx = dx - go;
    elseif strcmp(key, 't')
        dy = dy + go;
    elseif strcmp(key, 'g')
        dy = dy - go;
    elseif strcmp(key, 'y')
        dz = dz + go;
    elseif strcmp(key, 'h')
        dz = dz - go;
    else
        return;
    end
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

    d = [dx; dy; dz; 0];
    P = [p1(1) p2(1) p3(1) ; p1(2) p2(2) p3(2) ; p1(3) p2(3) p3(3); 1 1 1];
    R_0_1 = RotZ(theta) * RotY(pi) * RotX(psi);
    H_0_1 = [R_0_1, d(1:3); 0, 0, 0, 1];
    
    P_0 = H_0_1 * P;
    
    X = [P_0(1, 1), P_0(1, 2), P_0(1, 3), P_0(1, 1)];
    Y = [P_0(2, 1), P_0(2, 2), P_0(2, 3), P_0(2, 1)];
    Z = [P_0(3, 1), P_0(3, 2), P_0(3, 3), P_0(3, 1)];
    disp(P_0(:, 1));
    disp(P_0(:, 2));
    disp(P_0(:, 3));
    clf;

    % 3D View
    subplot(2, 2, 1);
    plot3(X, Y, Z, 'o-', 'linewidth', 2);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(3); % 3D view angle
    xlim([-20,20]);
    ylim([-20,20]);
    zlim([-20,20]);
    grid on;
    title('3D');

    % XY Plane View
    subplot(2, 2, 2);
    plot(X, Y, 'o-', 'linewidth', 2);
    xlabel('X');
    ylabel('Y');
    xlim([-10,10]);
    ylim([-10,10]);
    grid on;
    title('XY Plane');

    % YZ Plane View
    subplot(2, 2, 3);
    plot(Y, Z, 'o-', 'linewidth', 2);
    xlabel('Y');
    ylabel('Z');
    xlim([-10,10]);
    ylim([-10,10]);
    grid on;
    title('YZ Plane');

    subplot(2, 2, 4);
    plot(X, Z, 'o-', 'linewidth', 2);
    xlabel('X');
    ylabel('Z');
    xlim([-10,10]);
    ylim([-10,10]);
    grid on;
    title('XZ Plane');
    drawnow;
   
end