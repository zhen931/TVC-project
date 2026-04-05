function [] = animate(data, opts)
%%%%% Animates simulation history

%% Config
time_vec     = data.t;
pos_history  = data.state(:, 1:3);
q_history    = data.state(:, 7:10);
num_steps    = size(data.state, 1);
dt           = mean(diff(time_vec));

%% Figure Setup
fig = figure('Color', 'w', 'Name', 'Rocket Flight Animation');
ax  = axes('Position', [0.05 0.05 0.65 0.90], 'NextPlot', 'add');
view(3); grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Rocket Flight');

% Ground plane
[gx, gy] = meshgrid(linspace(-20, 20, 5));
surf(gx, gy, zeros(size(gx)), ...
    'FaceColor', [0.6 0.8 0.6], ...
    'FaceAlpha', 0.3, ...
    'EdgeColor', [0.5 0.7 0.5]);

%% Graphics Objects
rocket_path   = animatedline('Color', [0.8 0.2 0.2], 'LineWidth', 1.5);
rocket_marker = scatter3(0, 0, 0, 100, 'k', 'filled');
orient_x = quiver3(0,0,0,0,0,0, 'r', 'AutoScale', 'off', 'LineWidth', 2);
orient_y = quiver3(0,0,0,0,0,0, 'g', 'AutoScale', 'off', 'LineWidth', 2);
orient_z = quiver3(0,0,0,0,0,0, 'b', 'AutoScale', 'off', 'LineWidth', 2);
time_text = text(0.02, 0.95, '', ...
    'Units', 'normalized', 'FontSize', 11, 'FontName', 'Monospaced');

%% HUD Panel
labels  = opts.labels;          % pull label config from opts
n_labels = size(labels, 1);
hud_vals = gobjects(n_labels, 1);

hud_y_top = 0.88;
hud_dy    = min(0.08, 0.85 / n_labels);   % auto-space if many labels

% Header
annotation('textbox', [0.72 0.91 0.26 0.06], ...
    'String', 'Flight Data', ...
    'FontSize', 11, 'FontWeight', 'bold', ...
    'EdgeColor', 'none', 'Color', [0.2 0.2 0.2], ...
    'HorizontalAlignment', 'center');
annotation('line', [0.72 0.98], [0.91 0.91], 'Color', [0.6 0.6 0.6]);

for i = 1:n_labels
    y_pos = hud_y_top - (i-1) * hud_dy;

    % Name
    annotation('textbox', [0.73, y_pos-0.03, 0.12, 0.05], ...
        'String', labels{i,1}, ...
        'FontSize', 9, 'Color', [0.4 0.4 0.4], ...
        'EdgeColor', 'none', 'FontWeight', 'bold');

    % Value — handle updated each frame
    hud_vals(i) = annotation('textbox', ...
        [0.83, y_pos-0.03, 0.14, 0.05], ...
        'String', '---', ...
        'FontSize', 9, 'Color', [0.1 0.1 0.1], ...
        'EdgeColor', 'none', 'HorizontalAlignment', 'right');
end

time_ann = annotation('textbox', [0.72 0.03 0.26 0.05], ...
    'String', 't = 0.00 s', ...
    'FontSize', 9, 'EdgeColor', [0.7 0.7 0.7], ...
    'BackgroundColor', [0.97 0.97 0.97], ...
    'HorizontalAlignment', 'center');

%% Animation Loop
for k = 1:num_steps
    tic;

    pos = pos_history(k, :);
    q   = q_history(k, :)';

    % Trail and marker
    addpoints(rocket_path, pos(1), pos(2), pos(3));
    set(rocket_marker, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));

    % Orientation
    x_dir = Body2World(q, [1;0;0]) * opts.orient_scale;
    y_dir = Body2World(q, [0;1;0]) * opts.orient_scale;
    z_dir = Body2World(q, [0;0;1]) * opts.orient_scale;
    set(orient_x, 'XData',pos(1),'YData',pos(2),'ZData',pos(3), ...
        'UData',x_dir(1),'VData',x_dir(2),'WData',x_dir(3));
    set(orient_y, 'XData',pos(1),'YData',pos(2),'ZData',pos(3), ...
        'UData',y_dir(1),'VData',y_dir(2),'WData',y_dir(3));
    set(orient_z, 'XData',pos(1),'YData',pos(2),'ZData',pos(3), ...
        'UData',z_dir(1),'VData',z_dir(2),'WData',z_dir(3));

    % HUD update
    for i = 1:n_labels
        extractor = labels{i, 2};    % function handle @(data,k) ...
        fmt       = labels{i, 3};
        unit      = labels{i, 4};
        val       = extractor(k);
        set(hud_vals(i), 'String', sprintf([fmt ' %s'], val, unit));
    end

    % Follow-cam
    xlim([pos(1)-opts.cam_pad, pos(1)+opts.cam_pad]);
    ylim([pos(2)-opts.cam_pad, pos(2)+opts.cam_pad]);
    zlim([max(0, pos(3)-opts.cam_pad), pos(3)+opts.cam_pad]);

    set(time_text, 'String', sprintf('t = %.2f s  (x%.0f)', time_vec(k), opts.playback_speed));
    set(time_ann,  'String', sprintf('t = %.2f s  (%.0fx)', time_vec(k), opts.playback_speed));

    drawnow limitrate;

    elapsed = toc;
    pause_time = (dt / opts.playback_speed) - elapsed;
    if pause_time > 0, pause(pause_time); end
end
end