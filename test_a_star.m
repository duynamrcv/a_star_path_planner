clc;
clear;
close all;
%% Environment
x_max = 1.2; x_min = -0.2;
y_max = 1.2; y_min = -0.2;

%% Obstacle description
x_obs = [0.15 0.6];
y_obs = [0.25 0.5];
r_obs = [0.1 0.15];

%% Robot parameters
r = 0.05;   % robot radius
start = [0,0];
goal = [1,1];

%% Build the grid map
step = 0.05;
X = x_min:step:x_max;
Y = y_min:step:y_max;
[XX,YY] = meshgrid(X, Y);
V = true(numel(X), numel(Y));

for i = 1:length(x_obs)
    mask = sqrt((XX - x_obs(i)).^2 + (YY - y_obs(i)).^2) < r_obs(i)+r;
    V(mask) = false;
end
% Convert start and goal to grid map
grid_start = round((start(1)-x_min)/step)*numel(X) + round((start(2)-y_min)/step)+1;
grid_goal = round((goal(1)-x_min)/step)*numel(X) + round((goal(2)-y_min)/step)+1;

%% A star running
tic
final = a_star((V), ones(size(V)), grid_start, grid_goal);
toc
a_star_plot((V), ones(size(V)), final)
grid_final = [];
for i =1:length(final)
    [x, y] = ind2sub(size(V), final(i));
    grid_final = [grid_final; [y-1,x-1]];
end

%% Convert final path from grid to euler
path = grid_final*step + [x_min,y_min];
% path

%% Plot the environment
figure();
xlim([x_min x_max]);
ylim([y_min y_max]);
axis equal
grid on
hold on

% Plot obstacle
for i = 1:length(x_obs)
    lin = 0:0.1*pi:2*pi;
    x = x_obs(i) + r_obs(i)*cos(lin);
    y = y_obs(i) + r_obs(i)*sin(lin);
    plot(x, y, '-r', 'LineWidth', 2);
    plot(x_obs(i), y_obs(i), 'ro','MarkerSize',10,'MarkerFaceColor','r');
end
% Plot planned path
plot(path(:,1), path(:,2), '-k', 'LineWidth', 2);
% Plot goal
plot(path(1,1), path(1,2), 'ko','MarkerSize',10,'MarkerFaceColor','k');
% Plot start
plot(path(end,1), path(end,2), 'ks','MarkerSize',10,'MarkerFaceColor','k');
xlabel('X (m)');
ylabel('Y (m)');