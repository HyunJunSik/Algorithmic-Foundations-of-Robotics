clear;

V = zeros(1000, 6);
G = graph();
cnt = 1;
rng(0, 'twister');
a = -40;
b = 40;
c = 0;
d = 2 * pi;

%% set init, goal, ostacle collisionBox

% init, goal
init_box = collisionBox(6, 12, 10);
goal_box = collisionBox(6, 12, 10);
init_box.Pose = trvec2tform([-20 -20 -20]);
goal_box.Pose = trvec2tform([30 30 30]) * eul2tform([pi/4 pi/3 pi/3]);

% obstacle
obs1 = collisionBox(70, 1, 25);
obs2 = collisionBox(70, 1, 25);
obs3 = collisionBox(25, 1, 70);
obs4 = collisionBox(25, 1, 70);
obs1.Pose = trvec2tform([0 0 -24]);
obs2.Pose = trvec2tform([0, 0, 24]);
obs3.Pose = trvec2tform([22.5, 0, 0]);
obs4.Pose = trvec2tform([-22.5, 0, 0]);

%% random sampling

disp("random sampling..");
while cnt < 1001
    x = (b - a).*rand(1, 1) + a;
    y = (b - a).*rand(1, 1) + a;
    z = (b - a).*rand(1, 1) + a;
    th1 = (d - c).*rand(1, 1) + c;
    th2 = (d - c).*rand(1, 1) + c;
    th3 = (d - c).*rand(1, 1) + c;
    f = check_colli(x, y, z, th1, th2, th3, obs1, obs2, obs3, obs4);
    
    % f == 0 Collision Free point
    if f == 0
        G = addnode(G, table(string(cnt), [x y z th1 th2 th3], 'VariableNames', {'Name', 'Point'}));
        cnt = cnt + 1;
    end
    
end

%% k neariest neighbors

% 가장 가까운 점 k개를 간선으로 연결해야함.
% 하지만, k개의 점은 
disp("connecting 100 points edges...");
for i = 1 : 1000
    if i == 100
        disp("connecting edges 10% complete....");
    elseif i == 400
        disp("connecting edges 40% complete....");
    elseif i == 700
        disp("connecting edges 70% complete....");
    end
    shortest = [];
    point1 = G.Nodes.Point(i, :);
    for j = 1 : 1000
        if j ~= i
            point2 = G.Nodes.Point(j, :);
            dist = norm(point2 - point1, 2);
            shortest = [shortest; [dist j]];
        end
    end
    
    shortest = sortrows(shortest, 1);
    % shortest 배열에서 순차적으로 서로를 잇는 선이 Collision_free인지 체크
    % 이을 수 있다면, findedge(G, s, t)로 이을 수 있는 간선인지 체크
    % addedge and cnt + 1 repeat
    cnt = 0;
    for j = 1 : 999
        if cnt >= 100
            break
        end

        ch_point = G.Nodes.Point(shortest(j, 2), :);
        c = check_delta(point1, ch_point, obs1, obs2, obs3, obs4);

        % c == 1이면 Collision free
        idx = findedge(G, i, shortest(j, 2));
        if c == 1 && idx == 0
            G = addedge(G, i, shortest(j, 2));
            cnt = cnt + 1;
        end
    end
end

%% Solve Query Algorithm
disp("Query Algorithm executing...");
% Set Graph node init and goal
G = addnode(G, table("init", [-20 -20 -20 0 0 0], 'VariableNames', {'Name', 'Point'}));
G = addnode(G, table("goal", [30 30 30 pi/4 pi/3 pi/3], 'VariableNames', {'Name', 'Point'}));

% connecting init point to nearest neighbor
init_idx = findnode(G, "init");
init_point = G.Nodes.Point(init_idx, :);
shortest = [];
for i = 1 : 1000
    point1 = G.Nodes.Point(i, :);
    dist = norm(init_point - point1, 2);
    shortest = [shortest; [dist i]];
end

shortest = sortrows(shortest, 1);
cnt = 0;
for i = 1 : 1000
    if cnt >= 1
        break
    end

    ch_point = G.Nodes.Point(shortest(i, 2), :);
    c = check_delta(init_point, ch_point, obs1, obs2, obs3, obs4);
    if c == 1
        G = addedge(G, init_idx, shortest(i, 2));
        cnt = cnt + 1;
    end
end

% connecting goal point to nearest neighbor
goal_idx = findnode(G, "goal");
goal_point = G.Nodes.Point(goal_idx, :);
shortest = [];
for i = 1 : 1000
    point1 = G.Nodes.Point(i, :);
    dist = norm(goal_point - point1, 2);
    shortest = [shortest; [dist i]];
end

shortest = sortrows(shortest, 1);
cnt = 0;
for i = 1 : 1000
    if cnt >= 1
        break
    end

    ch_point = G.Nodes.Point(shortest(i, 2), :);
    c = check_delta(goal_point, ch_point, obs1, obs2, obs3, obs4);
    if c == 1
        G = addedge(G, goal_idx, shortest(i, 2));
        cnt = cnt + 1;
    end
end

P = shortestpath(G, init_idx, goal_idx);

[~, patchObj] = show(init_box);
patchObj.FaceColor = [1 1 1];
hold on;
show(goal_box);
[~, patchObj] = show(obs1);
patchObj.FaceColor = [0 1 1];
[~, patchObj] = show(obs2);
patchObj.FaceColor = [0 1 1];
[~, patchObj] = show(obs3);
patchObj.FaceColor = [0 1 1];
[~, patchObj] = show(obs4);
patchObj.FaceColor = [0 1 1];
xlim([-40 40]);
ylim([-40 40]);
zlim([-40 40]);

b = collisionBox(6, 12, 10);

for t = 1 : 5
    x = [G.Nodes.Point(P(t), 1) G.Nodes.Point(P(t + 1), 1)];
    y = [G.Nodes.Point(P(t), 2) G.Nodes.Point(P(t + 1), 2)];
    z = [G.Nodes.Point(P(t), 3) G.Nodes.Point(P(t + 1), 3)];
    th1 = [G.Nodes.Point(P(t), 4) G.Nodes.Point(P(t + 1), 4)];
    th2 = [G.Nodes.Point(P(t), 5) G.Nodes.Point(P(t + 1), 5)];
    th3 = [G.Nodes.Point(P(t), 6) G.Nodes.Point(P(t + 1), 6)];
    
    x = linspace(x(1), x(2), 5);
    y = linspace(y(1), y(2), 5);
    z = linspace(z(1), z(2), 5);
    th1 = linspace(th1(1), th1(2), 5);
    th2 = linspace(th2(1), th2(2), 5);
    th3 = linspace(th3(1), th3(2), 5);
    for i = 1 : 5
        b.Pose = trvec2tform([x(i) y(i) z(i)]) * eul2tform([th1(i) th2(i) th3(i)]);
        [~, patchObj] = show(b);
        patchObj.FaceColor = [1 1 1];
        patchObj.FaceAlpha = 0.5;
        pause(0.1);
    end
end
hold off;

function f = check_colli(x, y, z, theta1, theta2, theta3, obs1, obs2, obs3, obs4)
    test = collisionBox(6, 12, 10);
    test.Pose = trvec2tform([x y z]) * eul2tform([theta1 theta2 theta3]);
    f = 0;
    for obs = [obs1, obs2, obs3, obs4]
        [check, ~, ~] = checkCollision(test, obs);
        if check == 1
            f = 1;
            break
        end
    end
end

function c = check_delta(p1, p2, obs1, obs2, obs3, obs4)

    x = linspace(p1(1), p2(1), 5);
    y = linspace(p1(2), p2(2), 5);
    z = linspace(p1(3), p2(3), 5);
    th1 = linspace(p1(4), p2(4), 5);
    th2 = linspace(p1(5), p2(5), 5);
    th3 = linspace(p1(6), p2(6), 5);
    
    colli = [];
    for i = 1 : 5
        f = check_colli(x(i), y(i), z(i), th1(i), th2(i), th3(i), obs1, obs2, obs3, obs4);
        colli = [colli; f];
        % 하나라도 1이 있으면 Collision free인 간선이 아님
    end

    % colli = [ 0 0 0 0 0] 이면 Collision free
    % colli = [ 0 0 0 1 0] 이면 Collision not free
    % 이때 any로 colli를 받았을때, 0 혹은 1로 출력됨
    c = ~any(colli);
end
