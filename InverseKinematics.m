% Koordinat target
x_target = 20; % cm
y_target = 15; % cm

% Panjang lengan
L1 = 10; % Panjang link pertama (cm)
L2 = 10; % Panjang link kedua (cm)

% Posisi joint
joint1_x = -10;
joint1_y = y_target;
joint2_x = 0;
joint2_y = y_target;
joint3_x = 10;
joint3_y = y_target;

% Inisialisasi figure
figure;
hold on;
grid on;
axis equal;
xlabel('X (cm)');
ylabel('Y (cm)');
xlim([-15, 25]); % Diperluas agar posisi joint-1 dan joint-3 terlihat
ylim([0, 20]);
title(['Point-', num2str(0), ', Loops = ', num2str(0)]);

% Plot target points
plot(target_points_x, target_points_y, 'rx', 'MarkerSize', 10, 'LineWidth', 2);

% Inisialisasi plot lengan
h1 = plot([0, L1], [0, 0], 'b-', 'LineWidth', 2); % Link 1
h2 = plot([L1, L1 + L2], [0, 0], 'b-', 'LineWidth', 2); % Link 2

% Plot untuk joint
h3 = plot(joint1_x, joint1_y, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k'); % Joint 1
h4 = plot(joint2_x, joint2_y, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k'); % Joint 2
h5 = plot(joint3_x, joint3_y, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k'); % Joint 3
h6 = plot(L1+L2, 0, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k'); % End-effector

% Loop untuk animasi
loops = 318; % Total jumlah loops
for i = 1:loops
    % Update posisi end-effector sesuai dengan target point
    if mod(i, num_points) == 0
        idx = num_points;
    else
        idx = mod(i, num_points);
    end
    x_target = target_points_x(idx);
    y_target = target_points_y(idx);
    
    % Kinematika balik untuk menghitung sudut joint
    d = sqrt(x_target^2 + y_target^2);
    theta2 = acos((d^2 - L1^2 - L2^2) / (2 * L1 * L2));
    theta1 = atan2(y_target, x_target) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
    
    % Hitung posisi joint berdasarkan sudut
    x1 = L1 * cos(theta1);
    y1 = L1 * sin(theta1);
    x2 = x1 + L2 * cos(theta1 + theta2);
    y2 = y1 + L2 * sin(theta1 + theta2);
    
    % Update plot lengan
    set(h1, 'XData', [0, x1]);
    set(h1, 'YData', [0, y1]);
    set(h2, 'XData', [x1, x2]);
    set(h2, 'YData', [y1, y2]);
    
    % Update posisi joint
    set(h3, 'XData', joint1_x, 'YData', joint1_y); % Joint 1
    set(h4, 'XData', joint2_x, 'YData', joint2_y); % Joint 2
    set(h5, 'XData', joint3_x, 'YData', joint3_y); % Joint 3
    set(h6, 'XData', x2, 'YData', y2); % End-effector
    
    % Update title
    title(['Point-', num2str(idx), ', Loops = ', num2str(i)]);
    
    % Pause untuk animasi (dibuat lebih lambat)
    pause(0.2);
end

hold off;
