clear
clc

syms M m l g k1 k2 u
syms x [4 1]

P = [M+m m*l*cos(x2); cos(x2) l];
f = [u + m*l*x4^2*sin(x2) - k1*x3; g*sin(x2) - k2/m*x4];

syms dx3 dx4
sol = solve(P*[dx3; dx4]-f, [dx3; dx4]);
dx3 = sol.dx3;
dx4 = sol.dx4;
simplify([dx3; dx4] == P\f)

%% Points of equilibrium

assume(M, {'real', 'positive'})
assume(m, {'real', 'positive'})
assume(l, {'real', 'positive'})
assume(g, {'real', 'positive'})
assume(k1, {'real', 'positive'})
assume(k2, {'real', 'positive'})
assume(x1, 'real')
assume(x2, 'real')
assume(x3, 'real')
assume(x4, 'real')
assume(u, 'real')

solve([x3; x4; P\f; u] == zeros(5, 1), [x; u], 'ReturnConditions', true)

%% Linearize around a point

x0 = zeros(4, 1);
u0 = 0;
dx = [x3; x4; P\f];

A = jacobian(dx, x);
B = jacobian(dx, u);

A = subs(A, [x; u], [x0; u0]);
B = subs(B, [x; u], [x0; u0]);

A = simplify(A);
B = simplify(B);

disp("The A matrix around the origin is");
pretty(A);
disp("The B matrix around the origin is");
pretty(B);

%% Numerical linearization

A = double(subs(A, [M m l g k1 k2], [2 0.1 0.5 9.81 0.1 0.1]));
B = double(subs(B, [M m l g k1 k2], [2 0.1 0.5 9.81 0.1 0.1]));

%% Fast numerical non-linear computation

F = [x3; x4; P\f];
F = simplify(subs(F, [M m l g], [2 0.1 0.5 9.81]));
% matlabFunctionBlock('tp5_s2_simulink/nonlinear', F)

%% Control Prep

F = subs(F, [k1 k2], [0.1 0.1]);
F = matlabFunction(F);
F = @(x, u) F(u, x(2), x(3), x(4));

%% Control

p = [-2 -2.5 -9 -10];
K = place(A, B, p);
G = ss(A, B, eye(4), zeros(4, 1));

T = 0:0.01:10;
[~,~,X] = lsim(ss(A-B*K, zeros(4,1), eye(4), zeros(4, 1)), 0*T, T, [0 pi/12 0 0]);

figure;
plot(T, X);

p_min = 0;
p_max = pi;
dt = 1d-4;
while abs(p_min - p_max) > 1e-6
    p = (p_min + p_max)/2;
    x = [0; p; 0; 0];
    for t=0:dt:10
        x = x + dt * F(x, -K*x);
    end
    if all(abs(x) < 1e-6)
        p_min = p;
    else
        p_max = p;
    end
end

T = 0:dt:10;
x = zeros(4, length(T));
x(:,1) = [0; p; 0; 0];
for k=1:length(T)-1
    x(:,k+1) = x(:,k) + dt * F(x(:,k), -K*x(:,k));
end

figure;
plot(T, x);

%% Observer

ps = 10 * [-2 -2.5 -9 -10];
C = [1 0 0 0; 0 1 0 0];
L = place(A', C', ps)';

p_min = 0;
p_max = pi;
dt = 1d-5;
while abs(p_min - p_max) > 1e-5
    p = (p_min + p_max)/2;
    x = [0; p; 0; 0];
    xh = [0; 0; 0; 0];
    for t=0:dt:10
        x = x + dt * F(x, -K*xh);
        xh = xh + dt * (A*xh - B*K*xh + L*(C*x - C*xh));
    end
    if all(abs(x) < 1e-6)
        p_min = p;
    else
        p_max = p;
    end
end

T = 0:dt:10;
x = zeros(4, length(T));
x(:,1) = [0; p; 0; 0];
xh = zeros(4, length(T));
xh(:,1) = [0; 0; 0; 0];
for k=1:length(T)-1
    x(:,k+1) = x(:,k) + dt * F(x(:,k), -K*xh(:,k));
    xh(:, k+1) = xh(:,k) + dt*(A*xh(:,k) - B*K*xh(:,k) + L*(C*x(:, k) - C*xh(:, k)));
end

figure;
plot(T, x);

%% Disturbance

T = 0:dt:15;
x = zeros(4, length(T));
x(:,1) = [0; p; 0; 0];
xh = zeros(4, length(T));
xh(:,1) = [0; 0; 0; 0];
d_start = 5/dt;
d_end = 8/dt;
for k=1:length(T)-1
    x(:,k+1) = x(:,k) + dt * F(x(:,k), -K*xh(:,k));
    y = C*x(:, k);
    if k > d_start && k < d_end
        y(2) = y(2) + 0.1;
    end
    xh(:, k+1) = xh(:,k) + dt*(A*xh(:,k) - B*K*xh(:,k) + L*(y - C*xh(:, k)));
end

figure;
plot(T, x);
legend(["x1", "x2", "x3", "x4"])
