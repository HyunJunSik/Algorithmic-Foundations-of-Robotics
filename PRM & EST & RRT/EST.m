clear;

%% set init, goal, obstacle collisionBox

init_box = collisionBox(6, 12, 10);
goal_box = collisionBox(6, 12, 10);
init_box.Pose = trvec2tform([-20 -20 -20]);
goal_box.Pose = trvec2tform([30 30 30]) * eul2tform([pi/4 pi/3 pi/3]);

obs1 = collisionBox(90, 1, 25);
obs2 = collisionBox(90, 1, 25);
obs3 = collisionBox(35, 1, 70);
obs4 = collisionBox(35, 1, 70);
obs1.Pose = trvec2tform([0 0 -24]);
obs2.Pose = trvec2tform([0, 0, 24]);
obs3.Pose = trvec2tform([25, 0, 0]);
obs4.Pose = trvec2tform([-25, 0, 0]);

%% random sampling
V = zeros(1001, 6);
G = graph();
cnt = 1;
rng(0, 'twister');
a = -40;
b = 40;
c = 0;
d = 2 * pi;

% Tree의 루트노드로 init point 넣기
G = addnode(G, table("init", [-20 -20 -20 0 0 0], 'VariableNames', {'Name', 'Point'}));

% 1000개 샘플링
while cnt < 1001
    x = (b - a).*rand(1, 1) + a;
    y = (b - a).*rand(1, 1) + a;
    z = (b - a).*rand(1, 1) + a;
    th1 = (d - c).*rand(1, 1) + c;
    th2 = (d - c).*rand(1, 1) + c;
    th3 = (d - c).*rand(1, 1) + c;
    f = check_colli(x, y, z, th1, th2, th3, obs1, obs2, obs3, obs4);
    
    if f == 0
        V(cnt, :) = [x y z th1 th2 th3];
        cnt = cnt + 1;
    end
end
% lastnode is goal_point
V(1001, :) = [30 30 30 pi/4 pi/3 pi/3];

%% Expansion Tree
flag = 1;
cnt = 1;
shortest = [];

while flag
    if isempty(G.Edges)
        % rootnode와의 연결
        init_idx = findnode(G, "init");
        init_point = G.Nodes.Point(init_idx, :);
        shortest = nearest_neighbor(V, init_point);
        num = shortest(1, 2);
        point = V(num, :);
        G = addnode(G, table(string(num), point, 'VariableNames', {'Name', 'Point'}));
        G = addedge(G, init_idx, findnode(G, string(num)));
        cnt = cnt + 1;
    else
        deg = zeros(cnt, 1);
        for j = 1 : cnt
            deg(j, 1) = 1 / (degree(G, j) + 1);
        end
        S = sum(deg);
        for j = 1 : cnt
            deg(j, 1) = deg(j, 1) / S;
        end
        sample_idx = datasample(1:cnt, 1, 'Weights', deg);
        sample_point = G.Nodes.Point(sample_idx, :);
        shortest = nearest_neighbor(V, sample_point);
     
        s = 1;
        num = shortest(s, 2);
        while findnode(G, string(num))
            s = s + 1;
            num = shortest(s, 2);
        end
        point = V(num, :);
        c = check_delta(sample_point, point, obs1, obs2, obs3, obs4);

        if c == 1
            G = addnode(G, table(string(num), point, 'VariableNames', {'Name', 'Point'}));
            G = addedge(G, sample_idx, findnode(G, string(num)));
            cnt = cnt + 1;
            if num == 1001
                flag = 0;
            end
        end
    end
end

P = shortestpath(G, init_idx, findnode(G, string(1001)));

[~, patchObj] = show(init_box);
patchObj.FaceColor = [0.5 1 1];
hold on;
[~, patchObj] = show(goal_box);
patchObj.FaceColor = [1 0.5 1];

[~, patchObj] = show(obs1);
patchObj.FaceColor = [0 1 1];
[~, patchObj] = show(obs2);
patchObj.FaceColor = [0 1 1];
[~, patchObj] = show(obs3);
patchObj.FaceColor = [0 1 1];
[~, patchObj] = show(obs4);
patchObj.FaceColor = [0 1 1];
xlim([-50 50]);
ylim([-50 50]);
zlim([-50 50]);

b = collisionBox(6, 12, 10);

for t = 1 : size(P, 2) - 1
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


function shortest = nearest_neighbor(V, search_point)
    % search_point가 들어오면 가장 가까운 점들을 순서가 있는 list 출력
    shortest = [];
    for i = 1 : 1001
        p = V(i, :);
        dist = norm(search_point - p, 2);
        shortest = [shortest; [dist i]];
    end
    shortest = sortrows(shortest, 1);
end



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

    x = linspace(p1(1), p2(1), 10);
    y = linspace(p1(2), p2(2), 10);
    z = linspace(p1(3), p2(3), 10);
    th1 = linspace(p1(4), p2(4), 10);
    th2 = linspace(p1(5), p2(5), 10);
    th3 = linspace(p1(6), p2(6), 10);
    
    colli = [];
    for i = 1 : 10
        f = check_colli(x(i), y(i), z(i), th1(i), th2(i), th3(i), obs1, obs2, obs3, obs4);
        colli = [colli; f];
        % 하나라도 1이 있으면 Collision free인 간선이 아님
    end

    % colli = [ 0 0 0 0 0] 이면 Collision free
    % colli = [ 0 0 0 1 0] 이면 Collision not free
    % 이때 any로 colli를 받았을때, 0 혹은 1로 출력됨
    c = ~any(colli);
end
