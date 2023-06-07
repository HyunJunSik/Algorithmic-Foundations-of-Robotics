clc
clear

bel = ones(200, 1);
bel = bel / sum(bel);

M_plus = zeros(200, 200);
for i = 1 : 200
    if i + 1 < 201
        M_plus(i + 1, i) = 1/3;
    end
    if i + 2 < 201
        M_plus(i + 2, i) = 1/3;
    end
    if i + 3 < 201
        M_plus(i + 3, i) = 1/3;
    end
end

sensor_pos = [31 61 121];
sensor_model_at_sensor_position = measure_1D(sensor_pos);
sample = linspace(1, 200, 200);
sensor_model = zeros(200, 200);

for i = 1 : 200
    if any(i == sensor_pos)
        sensor_model(i, :) = sensor_model_at_sensor_position;
    else
        sensor_model(i, :) = ones(1, 200);
        sensor_model(i, :) = sensor_model(i, :) / sum(sensor_model(i, :));
    end
end


x = 1;
x_list = zeros(200, 1);
bel_list = zeros(200, 200);
for i = 1 : 100
    x_list(i) = x;
    bel_list(i, :) = bel;
    bel = bayes_filter(bel, x, sensor_model, M_plus);
    x = x + 2;
    if x == 200
        break
    end
end
 
f = figure;
f.Position = [0 0 1200 1000];
 
for i = 1 : 100
    subplot(2, 1, 1);
    plot(sample, bel_list(i, :), "b", "LineWidth", 3);
    axis([0, 200, 0, 1]);
    
    subplot(2, 1, 2);
    plot(sensor_pos, [1, 1, 1], 'go', "MarkerSize", 10, "MarkerEdgeColor", "g", "MarkerFaceColor", [.5 .5 .5]);
    axis([0, 200, 0, 2]);
    hold on;
    plot([x_list(i)], 1, 'rd', "MarkerSize", 10, "MarkerEdgeColor", "r", "MarkerFaceColor", "auto");
    hold off;
    drawnow

end


function bel = bayes_filter(bel, x, sensor_model, M)
    bel = M * bel;
    bel = sensor_model(x, :)' .* bel;
    bel = bel / sum(bel);
end


function gf_temp = Gaussian_1D(sigma, size)
    gf_temp1 = zeros(1, fix((size+1)/2));

    for x = 0 : (fix((size+1)/2)-1)
        gf_temp1(x + 1) = 1 / (sqrt(2 * pi) * sigma^2) * exp((-1 * x * x) / (2 * sigma * sigma));
    end
    gf_temp2 = gf_temp1;
    gf_temp2(1) = [];
    gf_temp2 = fliplr(gf_temp2);
    gf_temp = [gf_temp2 gf_temp1];
end

function sensor_model = measure_1D(sensor_pos)
    sensor_model_single = Gaussian_1D(1.5, 12);
    sensor_model = zeros(1, 200);
    for j = 1 : length(sensor_pos)
        sp = sensor_pos(j);
        lb = sp - floor(length(sensor_model_single) / 2);
        for i = 1 : length(sensor_model_single)
            m_idx = lb + i - 1;
            if (m_idx >= 1) && (m_idx <= length(sensor_model))
                if sensor_model(m_idx) < sensor_model_single(i)
                    sensor_model(m_idx) = sensor_model_single(i);
                end
            end
        end
    end
end