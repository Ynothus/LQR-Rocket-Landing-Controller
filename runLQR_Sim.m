%% Input Dialog
prompts = {'Dry Mass (kg):', ...
           'Fuel Mass (kg):', ...
           'Gravity (m/s²):', ...
           'Initial Altitude (m):', ...
           'Initial Velocity (m/s):', ...
           'Max Time (s):', ...
           'Max Thrust (N):', ...
           'Max Altitude Error (m):', ...
           'Max Velocity Error (m/s):', ...
           'Time Step (s):', ...
           'Drag Coefficient (Cd):', ...
           'Air Density (kg/m³):', ...
           'Altimeter Noise σ (m):', ...
           'Accelerometer Noise σ (m/s²):', ...
           'Model Uncertainty:'};

dlgTitle = 'LQR Kalman 1D Landing Simulation';
dims = [1 50];

defaults = {'0.156', '0.1', '9.81', '500', '0', ...
            '30', '100', '40', '30', '0.01', ...
            '0.45', '1.225', ...
            '0.4', '0.15', '2.0'};

answer = inputdlg(prompts, dlgTitle, dims, defaults);

% Handle cancel
if isempty(answer)
    disp('Simulation cancelled.');
    return;
end

% Parse inputs
vals = str2double(answer);

% Check for invalid entries
if any(isnan(vals))
    errordlg('All inputs must be valid numbers.', 'Input Error');
    return;
end

dmass = vals(1);
fm = vals(2);
gravity = vals(3);
int_alt = vals(4);
int_velo = vals(5);
max_time = vals(6);
maxthrust = vals(7);
max_alt_error = vals(8);
max_velo_error = vals(9);
dt = vals(10);
Cd = vals(11);
Pden = vals(12);
sigma_alt = vals(13);
sigma_acc = vals(14);
model_uncertainty = vals(15);

%% Run Simulation
[t, altitude, velocity, acceleration, thrust, mass, fmass, ...
 altitude_est, velocity_est, altitude_meas] = ...
    LQR_Kal_1D_Sim(dmass, fm, gravity, int_alt, int_velo, ...
    max_time, maxthrust, max_alt_error, max_velo_error, dt, ...
    Cd, Pden, sigma_alt, sigma_acc, model_uncertainty);

%% Plot 1: Simulation Results
figure('Name', 'LQR Kalman 1D Landing Simulation', 'NumberTitle', 'off');

subplot(3,2,1);
plot(t, altitude, 'b', 'LineWidth', 5); hold on;
plot(t, altitude_meas, 'r.', 'MarkerSize', 6);
plot(t, altitude_est, 'g', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude'); legend('True', 'Measured', 'Estimated');
grid on;

subplot(3,2,2);
plot(t, velocity, 'b', 'LineWidth', 3); hold on;
plot(t, velocity_est, 'g', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Velocity'); legend('True', 'Estimated');
grid on;

subplot(3,2,3);
plot(t, acceleration);
xlabel('Time (s)'); ylabel('Acceleration (m/s²)');
title('Acceleration'); grid on;

subplot(3,2,4);
plot(t, thrust);
xlabel('Time (s)'); ylabel('Thrust (N)');
title('Thrust'); grid on;

subplot(3,2,5);
plot(t, mass);
xlabel('Time (s)'); ylabel('Mass (kg)');
title('Total Mass'); grid on;

subplot(3,2,6);
plot(t, fmass);
xlabel('Time (s)'); ylabel('Fuel Remaining (%)');
title('Fuel Mass'); grid on;

%% Plot 2: Kalman Filter Performance
figure('Name', 'Kalman Filter Performance', 'NumberTitle', 'off');

% Find landing index (where altitude becomes 0)
land_idx = find(altitude == 0, 1);
if isempty(land_idx)
    land_idx = numSteps;
end

% Only use flight data for error analysis
alt_err_flight   = altitude(1:land_idx) - altitude_est(1:land_idx);
vel_err_flight   = velocity(1:land_idx) - velocity_est(1:land_idx);
alt_sens_flight  = altitude(1:land_idx) - altitude_meas(1:land_idx);

subplot(2,2,1);
plot(t(1:land_idx), alt_err_flight, 'b', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Error (m)');
title('Altitude Estimation Error (True - Estimated)');
grid on;

subplot(2,2,2);
plot(t(1:land_idx), vel_err_flight, 'b', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Error (m/s)');
title('Velocity Estimation Error (True - Estimated)');
grid on;

subplot(2,2,3);
plot(t(1:land_idx), alt_sens_flight, 'r', 'LineWidth', 0.8); hold on;
plot(t(1:land_idx), alt_err_flight, 'g', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Error (m)');
title('Measurement vs Estimation Error');
legend('Sensor Error', 'Kalman Error');
grid on;

subplot(2,2,4);
histogram(alt_err_flight, 30, 'FaceColor', 'b', 'FaceAlpha', 0.5); hold on;
histogram(alt_sens_flight, 30, 'FaceColor', 'r', 'FaceAlpha', 0.5);
xlabel('Error (m)'); ylabel('Count');
title('Error Distribution (Flight Only)');
legend('Kalman', 'Sensor');
grid on;