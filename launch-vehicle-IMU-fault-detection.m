clc;
clear;
close all;

%% STEP 1: System Parameters
% Define physical parameters and simulation settings

J   = 5000;      % Rotor inertia
D   = 800;       % Aerodynamic damping
tau = 0.1;       % Actuator time constant

Kp  = 35000;     % Proportional controller gain

dt = 0.001;      % Simulation step
t  = 0:dt:10;    % Simulation time

bias_value = 0.02; % Gyro bias fault magnitude

s = tf('s');     % Laplace variable

%% STEP 2: Plant Model
% Rigid body dynamics + actuator lag

G_rigid = 1/(J*s + D);     % Rotor dynamics
G_act   = 1/(tau*s + 1);   % Actuator model
G_plant = G_act * G_rigid; % Complete plant

disp('Plant Transfer Function')
G_plant

figure;
bode(G_plant)
grid on
title('Bode Plot of Open Loop Plant')

%% STEP 2B: Effect of Actuator Lag on Phase Margin
% Study how actuator time constant affects stability

tau_values = [0.01 0.05 0.1 0.2];

figure;
hold on

for i = 1:length(tau_values)

    tau = tau_values(i);

    G_rigid = 1/(J*s + D);
    G_act   = 1/(tau*s + 1);
    G_plant_tau = G_act * G_rigid;

    L_tau = Kp * G_plant_tau;   % Open-loop system

    margin(L_tau)

    [GM, PM, ~, ~] = margin(L_tau);

    fprintf('Tau = %.3f sec\n', tau);
    fprintf('Phase Margin = %.2f deg\n\n', PM);

end

grid on
title('Effect of Actuator Lag on Stability Margins')

%% Restore nominal plant after tau sweep

tau = 0.1;

G_rigid = 1/(J*s + D);
G_act   = 1/(tau*s + 1);
G_plant = G_act * G_rigid;

%% STEP 3: Closed Loop System
% Design P controller and evaluate stability

C = pid(Kp,0,0);   % Proportional controller

L = C*G_plant;     % Open-loop transfer function

figure;
margin(L)
grid on
title('Open Loop with P Controller')

[GM, PM, ~, ~] = margin(L);

fprintf('Phase Margin = %.2f deg\n\n', PM);

T = feedback(L,1); % Closed-loop system

figure;
step(T)
title('Closed Loop Step Response')

stepinfo(T)        % Performance metrics

%% STEP 4: Convert to State Space
% State-space model used for simulation

sys = ss(G_plant);

A = sys.A;
B = sys.B;
Cmat = sys.C;

%% STEP 5: Single Fault Injection Example
% Inject gyro bias fault and simulate closed-loop behavior

x = zeros(size(A,1),1);  % Initial states

omega = zeros(size(t));       % True angular rate
omega_meas = zeros(size(t));  % Measured rate

bias = 0;

for k = 1:length(t)

    if t(k) >= 5
        bias = bias_value;    % Fault introduced at 5 seconds
    end

    omega(k) = Cmat*x;        % True output

    noise = 0.02*randn;       % Sensor noise

    omega_meas(k) = omega(k) + bias + noise;  % Faulty measurement

    error = -omega_meas(k);   % Feedback error

    u = Kp*error;             % Control input

    xdot = A*x + B*u;         % State dynamics
    x = x + xdot*dt;

end

figure;
plot(t,omega,'LineWidth',1.5)
hold on
plot(t,omega_meas,'--')
legend('True Rate','Measured Rate')
xlabel('Time (s)')
ylabel('Angular Rate')
title('Gyro Bias Fault Injection')
grid on

%% STEP 6: Residual Detection
% Residual = difference between measured and true signal

residual_raw = omega_meas - omega;

residual = movmean(residual_raw,50); % Noise smoothing

threshold = 0.015;  % Fault detection threshold
window_time = 0.2;  % Persistence window

window_samples = round(window_time/dt);

detection_time = NaN;

for k = 1:length(t)-window_samples
    
    if all(abs(residual(k:k+window_samples)) > threshold)
        detection_time = t(k); % Fault detected
        break
    end
    
end

figure;
plot(t,residual)
hold on
yline(threshold,'r--')
yline(-threshold,'r--')
title('Residual and Detection Threshold')
grid on

fprintf('Detection Time = %.3f sec\n', detection_time)

if ~isnan(detection_time)
    idx = find(t >= detection_time,1);
    omega_at_detection = omega(idx);
    fprintf('True rate at detection = %.5f rad/s\n', omega_at_detection);
end

%% STEP 7: Detection Delay vs Noise (Averaged Study)
% Evaluate how measurement noise affects detection speed

noise_levels = [0.005 0.01 0.02 0.05];

num_runs = 5;

detection_delay = zeros(size(noise_levels));

for i = 1:length(noise_levels)

    sigma = noise_levels(i);
    drift_runs = zeros(1,num_runs);

    delay_runs = zeros(1,num_runs);

    for r = 1:num_runs

        x = zeros(size(A,1),1);
        residual_raw = zeros(size(t));

        bias = 0;

        for k = 1:length(t)

            if t(k) >= 5
                bias = bias_value;
            end

            omega = Cmat*x;

            noise = sigma*randn;

            omega_meas = omega + bias + noise;

            error = -omega_meas;

            u = Kp*error;

            xdot = A*x + B*u;
            x = x + xdot*dt;

            residual_raw(k) = omega_meas - omega;

        end

        residual = movmean(residual_raw,50);

        detection_time = NaN;

        for k = 1:length(t)-window_samples

            if all(abs(residual(k:k+window_samples)) > threshold)
                detection_time = t(k);
                break
            end

        end

if ~isnan(detection_time)
    delay_runs(r) = detection_time - 5;
    idx = find(t >= detection_time,1);
    drift_runs(r) = omega;
else
    delay_runs(r) = NaN;
    drift_runs(r) = NaN;
end

    end

    detection_delay(i) = mean(delay_runs,'omitnan');
    drift_at_detection(i) = mean(drift_runs,'omitnan');

fprintf('Sigma = %.3f , Delay = %.4f s , Drift = %.5f rad/s\n', sigma, detection_delay(i), drift_at_detection(i));
end

%% Plot Result

figure
plot(noise_levels,detection_delay,'-o','LineWidth',1.5)
xlabel('Noise Standard Deviation')
ylabel('Average Detection Delay (s)')
title('Detection Delay vs Measurement Noise')
grid on
