clc
clear 
close all;
%% Main code
% Parameters
M = 3;    % Mass of the cart (kg)
m = 1;    % Mass of the pendulum (kg)
l = 1;    % Length of the pendulum (m)
g = 9.81; % Acceleration due to gravity (m/s^2)

% Time span for simulation
t_range = [0 2];

% Target point
T = [1; 0.5];

% Optimization to find the control input coefficients
initial_guess =  [-0.1 , -0.1, -0.1, 0.1 , 0.1, 0.1];  % Initial guess for the coefficients
%initial_guess = zeros(6, 1);
lb = [];  % No lower bounds
ub = [];  % No upper bounds
options = optimset('fmincon');  % Set options for fmincon
%options.Display = 'iter';  % Display optimization iterations
options.Algorithm = 'sqp';  % Use the Sequential Quadratic Programming algorithm

% Solve the optimization problem
coeffs = fmincon(@(coeffs) objective_function(coeffs, M, m, l, g, T), initial_guess, [], [], [], [], lb, ub, @(coeffs) constraint(coeffs, M, m, l, g), options);

% Simulate the system with the optimized force input
[t, state] = ode45(@(t, state) dynamics(t, state, M, m, l, g, @(t) force_input(t, coeffs)), t_range, [0; 0; 0; 0]);

% Extract the solution
x_solution = state(:, 1);
phi_solution = state(:, 2);
x_dot_solution = state(:, 3);
phi_dot_solution = state(:, 4);
u_solution = arrayfun(@(t) force_input(t, coeffs), t);

%% Results illustration
% Animation
figure(1);
set(gcf, 'Position', [1000, 200, 400, 400]);

for i = 1:length(t)
    draw_cart_pendulum(x_solution(i), phi_solution(i), l);
    title(['Simulation - Time: ' num2str(t(i)) ' seconds']);
    pause(0.05);
end

% Plot the pendulum trajectory - changing colors
% figure(2);
% for i = 1:length(t)
%     draw_pendulum_trajectory(x_solution(i), phi_solution(i), l, i / length(t));
%     title(['Pendulum Trajectory at Time: ' num2str(t(i)) ' seconds']);
%     pause(0.05);
% end

% Plots
figure(3);
set(gcf, 'Position', [100, 100, 600, 600]);

% Subplot for control input
subplot(2, 2, 1);
plot(t, u_solution, 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Force Input (u)');
title('Optimized Control Input over Time');
grid on 
grid minor

% Subplot for state variables
subplot(2, 2, 3);
plot(t, x_solution, 'LineWidth', 1, 'DisplayName', '$x$');
hold on;
plot(t, phi_solution, 'LineWidth', 1, 'DisplayName', '$\phi$');
plot(t, x_dot_solution, 'LineWidth', 1, 'DisplayName', '$\dot{x}$');
plot(t, phi_dot_solution, 'LineWidth', 1, 'DisplayName', '$\dot{\phi}$');
xlabel('Time (s)');
ylabel('System State Variables');
title('State Variables over Time');
legend('Interpreter', 'latex','Location', 'best');
grid on 
grid minor
hold off;

subplot(2, 2, 2);
for i = 1:length(t)
    draw_pendulum_trajectory(x_solution(i), phi_solution(i), l, i / length(t));
    title('Pendulum Trajectory');
end
initial_state_plot = plot(x_solution(1) + l * sin(phi_solution(1)),-l * cos(phi_solution(1)), ...
    'Color', [0, 0 , 0], 'LineWidth', 1.5);
final_state_plot = plot(x_solution(end) + l * sin(phi_solution(end)),-l * cos(phi_solution(end)), ...
    'Color', [0, 0+0.5, 0+1], 'LineWidth', 1.5);
target_point = plot(T(1),T(2),'xk',MarkerSize=12);
legend([initial_state_plot, final_state_plot, target_point],'Initial State', ...
    'Final State', 'Target point','Location','best')


%% Functions
function dxdt = dynamics(t, state, M, m, l, g, u)
    % ODE of the dynamics of the cart-and-pendulum system

    % Unpack state vector
    phi = state(2);
    x_dot = state(3);
    phi_dot = state(4);

    % ODEs
    dxdt = zeros(4, 1);

    dxdt(1) = x_dot;
    dxdt(2) = phi_dot;
    dxdt(3) = (m*l*sin(phi)*phi_dot^2 + m*g*sin(phi)*cos(phi) + u(t)) / (M + m - m*cos(phi)^2);
    dxdt(4) = ((M + m)*g*sin(phi) - m*l*sin(phi)*cos(phi)*phi_dot^2 - u(t)*cos(phi)) / (l*(M + m - m*cos(phi)^2));

end

function force = force_input(t, coeffs)
    % Fifth-order polynomial force input

    % Coefficients
    a0 = coeffs(1);
    a1 = coeffs(2);
    a2 = coeffs(3);
    a3 = coeffs(4);
    a4 = coeffs(5);
    a5 = coeffs(6);

    % Fifth-order polynomial
    force = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;

end

function cost = objective_function(coeffs, M, m, l, g, T)
    % Objective function to minimize the distance to the target

    % Simulation time
    t_range = [0 2];

    % Simulate the system with the current coefficients
    [t, state] = ode45(@(t, state) dynamics(t, state, M, m, l, g, @(t) force_input(t, coeffs)), t_range, [0; 0; 0; 0]);

    % Extract the position of the tip of the pendulum at the final time
    P_final = [state(end, 1) + l*sin(state(end, 2)); -l*cos(state(end, 2))];

    % Target position
    %T = [1; 0.5];

    % Euclidean norm (distance) between P_final and T
    cost = norm(P_final - T);

end

function [c, ceq] = constraint(coeffs, M, m, l, g)

    c = [];
    ceq = [];

end

function draw_cart_pendulum(x, phi, l)
    % Function to draw the cart-and-pendulum system

    % Parameters for visualization (modify as needed)
    cart_width = 0.4;
    cart_height = 0.2;

    % Compute pendulum position
    pendulum_x = x + l * sin(phi);
    pendulum_y = -l * cos(phi);

    % Plot cart
    plot([x - cart_width/2, x + cart_width/2, x + cart_width/2, x - cart_width/2, x - cart_width/2], ...
         [0, 0, cart_height, cart_height, 0], 'b', 'LineWidth', 2);

    hold on;

    % Plot pendulum
    plot([x, pendulum_x], [0, pendulum_y], 'r', 'LineWidth', 2);
    plot(pendulum_x, pendulum_y,"o", 'MarkerSize', 6, ...
        'MarkerFaceColor','r', 'Color','r');
    
    % Display position and speed annotations
    text(pendulum_x, pendulum_y, ['(' num2str(pendulum_x, '%.2f') ', ' num2str(pendulum_y, '%.2f') ')'], ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'FontSize', 8, 'Color', 'k');

    % Set axis limits
    xlim([-2 2]);
    ylim([-2 2]);
    % Set aspect ratio to be equal
    daspect([1 1 1]);
    % Remove axis ticks
    %axis off;
    grid on 
    grid minor
    hold off;
end

function draw_pendulum_trajectory(x, phi, l, color_progress)
    % Function to draw the pendulum trajectory with changing colors

    % Parameters for visualization (modify as needed)
    pendulum_x = x + l * sin(phi);
    pendulum_y = -l * cos(phi);

    % Plot pendulum trajectory
    plot([x, pendulum_x], [0, pendulum_y], 'Color', [0, 0 + color_progress/2, 0 + color_progress], 'LineWidth', 1.5);
    hold on;
    %plot(x, 0, 'ok','MarkerSize', 4);
    plot(pendulum_x, pendulum_y,"o", 'MarkerSize', 6, ...
        'MarkerFaceColor',[0, 0 + color_progress/2, 0 + color_progress], 'Color',[0, 0 + color_progress/2, 0 + color_progress]);

    

    % Set axis limits
    xlim([-2 2]);
    ylim([-2 2]);

    % Set aspect ratio to be equal
    daspect([1 1 1]);

    % Remove axis ticks
    axis off;

end
