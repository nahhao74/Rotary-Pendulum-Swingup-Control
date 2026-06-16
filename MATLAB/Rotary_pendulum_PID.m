%% ===== GA TUNE PID VELOCITY (STABLE + FAST) =====
clear; clc; close all;

%% --- Load data ---
data = readtable('C:\Users\ADMIN\Documents\NCKH_RAI_Lab\Data_Final\motor_data_012.csv');
t = data.time;
y = data.vel;      % velocity (rpm)
u = data.output;   % PWM

Ts = mean(diff(t));
data_id = iddata(y, u, Ts);

%% --- Identify G(s)_vel ---
bestFit = -inf;
for np = 1:3
    for nz = 0:np-1
        try
            sys = tfest(data_id, np, nz);
            fit = sys.Report.Fit.FitPercent;
            if fit > bestFit
                bestFit = fit;
                G_vel = sys;
            end
        catch
        end
    end
end
fprintf('Best G_vel fit = %.2f%%\n', bestFit);
G_vel

%% --- Discretize ---
Gd = c2d(G_vel, Ts, 'tustin');
[bp, ap] = tfdata(Gd, 'v');

%% ===== GA PARAMETERS =====
vel_ref = 60;          % rpm
PWM_max = 255;
PWM_min = -255;

nVars = 3;               % [Kp Ki Kd]
lb = [0 0 0];
ub = [1 12 0.2];         % <<< hạn chế rung

opts = optimoptions('ga',...
    'PopulationSize',120,...
    'MaxGenerations',120,...
    'Display','iter',...
    'UseParallel',false);

fitness = @(x) vel_objective( ...
    x, ap, bp, Ts, vel_ref, PWM_max, PWM_min);

[x_best, J_best] = ga( ...
    fitness, nVars, [],[],[],[], lb, ub, [], opts);

Kp = x_best(1); Ki = x_best(2); Kd = x_best(3);

fprintf('\n===== BEST PID VELOCITY =====\n');
fprintf('Kp = %.4f | Ki = %.4f | Kd = %.4f\n', Kp, Ki, Kd);
fprintf('static const float VEL_KP = %.5f;\n', Kp);
fprintf('static const float VEL_KI = %.5f;\n', Ki);
fprintf('static const float VEL_KD = %.5f;\n', Kd);
fprintf('Best cost J = %.4f\n', J_best);

%% --- Simulate final ---
[vel, pwm] = simulate_vel_pid(x_best, ap, bp, Ts, vel_ref, PWM_max, PWM_min);

figure;
subplot(2,1,1)
plot(vel,'LineWidth',1.6); grid on;
yline(vel_ref,'--');
ylabel('Velocity (rpm)');
title('Velocity Response');

subplot(2,1,2)
plot(pwm,'LineWidth',1.6); grid on;
ylabel('PWM'); xlabel('Samples');

%% =========================================================
%% ================== OBJECTIVE FUNCTION ===================
function J = vel_objective(x, ap, bp, Ts, v_ref, PWM_max, PWM_min)

    Kp = x(1); Ki = x(2); Kd = x(3);

    na = length(ap); nb = length(bp);
    u_hist = zeros(nb,1);
    y_hist = zeros(na,1);

    int_e = 0; prev_e = 0;
    vel = 0;

    N = 450;
    vel_log = zeros(N,1);
    u_log = zeros(N,1);

    % derivative filter
    d_prev = 0;
    alpha = 0.15;

    for k = 1:N
        e = v_ref - vel;

        int_e = int_e + e*Ts;
        d_e = (e - prev_e)/Ts;
        d_e = alpha*d_e + (1-alpha)*d_prev;
        d_prev = d_e;
        prev_e = e;

        u = Kp*e + Ki*int_e + Kd*d_e;
        u = max(min(u,PWM_max),PWM_min);

        u_hist = [u; u_hist(1:end-1)];
        vel = (bp*u_hist - ap(2:end)*y_hist(1:end-1))/ap(1);
        y_hist = [vel; y_hist(1:end-1)];

        vel_log(k) = vel;
        u_log(k) = u;

        % ===== KILL UNSTABLE =====
        if abs(vel) > 2*v_ref || isnan(vel)
            J = 1e8;
            return
        end
    end

    %% ===== METRICS =====
    idx10 = find(vel_log >= 0.1*v_ref, 1);
    idx90 = find(vel_log >= 0.9*v_ref, 1);
    if isempty(idx10) || isempty(idx90)
        rise_time = inf;
    else
        rise_time = (idx90 - idx10)*Ts;
    end

    tol = 0.02*v_ref;
    settling_time = inf;
    for i = 1:N
        if all(abs(vel_log(i:end) - v_ref) < tol)
            settling_time = (i-1)*Ts;
            break
        end
    end

    overshoot = max(vel_log) - v_ref;
    overshoot_frac = max(overshoot/v_ref,0);
    steady_err = abs(vel_log(end) - v_ref) / v_ref;

    % oscillation
    osc = sum(abs(diff(vel_log))) / v_ref;

    % saturation
    sat_ratio = mean(abs(u_log) >= 0.95*PWM_max);

    %% ===== OBJECTIVE =====
    J = ...
        6*settling_time + ...
        2.5*rise_time + ...
        150*max(overshoot_frac-0.05,0) + ...
        80*steady_err + ...
        15*osc + ...
        50*sat_ratio + ...
        1e-4*mean(u_log.^2);

    if ~isfinite(J)
        J = 1e8;
    end
end

%% =========================================================
%% ===================== SIMULATION ========================
function [vel_log, u_log] = simulate_vel_pid(x, ap, bp, Ts, v_ref, PWM_max, PWM_min)

    Kp = x(1); Ki = x(2); Kd = x(3);

    na = length(ap); nb = length(bp);
    u_hist = zeros(nb,1);
    y_hist = zeros(na,1);

    int_e = 0; prev_e = 0;
    vel = 0;

    d_prev = 0;
    alpha = 0.15;

    N = 450;
    vel_log = zeros(N,1);
    u_log = zeros(N,1);

    for k = 1:N
        e = v_ref - vel;

        int_e = int_e + e*Ts;
        d_e = (e - prev_e)/Ts;
        d_e = alpha*d_e + (1-alpha)*d_prev;
        d_prev = d_e;
        prev_e = e;

        u = Kp*e + Ki*int_e + Kd*d_e;
        u = max(min(u,PWM_max),PWM_min);

        u_hist = [u; u_hist(1:end-1)];
        vel = (bp*u_hist - ap(2:end)*y_hist(1:end-1))/ap(1);
        y_hist = [vel; y_hist(1:end-1)];

        vel_log(k) = vel;
        u_log(k) = u;
    end
end
