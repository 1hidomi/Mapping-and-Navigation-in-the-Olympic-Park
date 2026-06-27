%% 0. Load data
load("sensorlog.mat");

%% 1. Extract GPS data
lat = Position.latitude;
lon = Position.longitude;
alt = Position.altitude;
gps_speed_phone = Position.speed;
course = Position.course;
t_pos = Position.Timestamp;

%% 2. Convert GPS coordinates to local Cartesian coordinates
lat0 = lat(1);
lon0 = lon(1);
R = 6371000; 
x = R * cosd(lat0) .* deg2rad(lon - lon0);
y = R * deg2rad(lat - lat0);
z = alt - alt(1);
t_pos_sec = seconds(t_pos - t_pos(1));

%% 3. Compute velocity components from GPS data
dt = diff(t_pos_sec);
vx = diff(x) ./ dt;
vy = diff(y) ./ dt;
vz = diff(z) ./ dt;
t_vel = t_pos_sec(2:end);
v_gps_raw = sqrt(vx.^2 + vy.^2 + vz.^2);
v_gps_phone = gps_speed_phone(2:end);

%% 4. Filter velocity components
win = 3;
vx_f = movmean(vx, win);
vy_f = movmean(vy, win);
vz_f = movmean(vz, win);
v_gps_f = sqrt(vx_f.^2 + vy_f.^2 + vz_f.^2);

%% 5. Plot velocity components
figure;
subplot(3,1,1)
plot(t_vel, vx, 'b-', 'LineWidth', 1.0); hold on;
plot(t_vel, vx_f, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('v_x (m/s)');
title('Velocity in X direction');
legend('Raw', 'Filtered');
grid on;

subplot(3,1,2)
plot(t_vel, vy, 'b-', 'LineWidth', 1.0); hold on;
plot(t_vel, vy_f, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('v_y (m/s)');
title('Velocity in Y direction');
legend('Raw', 'Filtered');
grid on;

subplot(3,1,3)
plot(t_vel, vz, 'b-', 'LineWidth', 1.0); hold on;
plot(t_vel, vz_f, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('v_z (m/s)');
title('Velocity in Z direction');
legend('Raw', 'Filtered');
grid on;

%% 6. Accelerometer-based velocity estimation
ax = Acceleration.X;
ay = Acceleration.Y;
az = Acceleration.Z;
t_acc = Acceleration.Timestamp;
t_acc_sec = seconds(t_acc - t_acc(1));
a_mag = sqrt(ax.^2 + ay.^2 + az.^2);
g_est = movmean(a_mag, 20);
a_dyn = a_mag - g_est;
a_dyn(abs(a_dyn) < 0.1) = 0;
v_acc_corr = cumtrapz(t_acc_sec, a_dyn);

%% 7. Plot speed comparison
figure;
plot(t_vel, v_gps_f, 'b-', 'LineWidth', 1.5); hold on;
plot(t_vel, v_gps_phone, 'g--', 'LineWidth', 1.5);
plot(t_acc_sec, v_acc_corr, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Comparison of speed');
legend('GPS-derived filtered speed', 'Phone GPS speed', 'Accelerometer integrated speed');
grid on;

%% 8. Pattern detection based on filtered GPS speed
pattern = strings(length(v_gps_f),1);
for i = 1:length(v_gps_f)
    if v_gps_f(i) < 0.5
        pattern(i) = "Stop";
    elseif v_gps_f(i) < 1.2
        pattern(i) = "Slow walk";
    else
        pattern(i) = "Fast";
    end
end
course2 = course(2:end);
dcourse = [0; abs(diff(course2))];
for i = 1:length(pattern)
    if dcourse(i) > 20 && v_gps_f(i) > 0.5
        pattern(i) = "Turning";
    end
end

%% 9. Show results table
resultTable = table(t_vel, vx_f, vy_f, vz_f, v_gps_f, dcourse, pattern, ...
    'VariableNames', {'Time_s','vx_filtered','vy_filtered','vz_filtered','Speed_filtered','CourseChange','Pattern'});
disp(resultTable);

%% 10. Plot detected patterns on trajectory
figure;
hold on; grid on; axis equal;
title('Detected motion patterns on trajectory');
xlabel('Local X (m)');
ylabel('Local Y (m)');

% Full trajectory
plot(x, y, 'k--', 'LineWidth', 1.0);
scatter(x(2:end), y(2:end), 25, 'k', 'filled');

idxStop = pattern == "Stop";
idxSlow = pattern == "Slow walk";
idxFast = pattern == "Fast";
idxTurn = pattern == "Turning";
scatter(x(1 + find(idxStop)), y(1 + find(idxStop)), 70, 'r', 'filled');
scatter(x(1 + find(idxSlow)), y(1 + find(idxSlow)), 70, 'b', 'filled');
scatter(x(1 + find(idxFast)), y(1 + find(idxFast)), 70, 'g', 'filled');
scatter(x(1 + find(idxTurn)), y(1 + find(idxTurn)), 90, 'y', 'filled');

legend('Trajectory', 'All points', 'Stop', 'Slow walk', 'Fast', 'Turning');

%% 11.summary counts
nStop = sum(idxStop);
nSlow = sum(idxSlow);
nFast = sum(idxFast);
nTurn = sum(idxTurn);

fprintf('\nPattern counts:\n');
fprintf('Stop      : %d\n', nStop);
fprintf('Slow walk : %d\n', nSlow);
fprintf('Fast      : %d\n', nFast);
fprintf('Turning   : %d\n', nTurn);

rmse_gps = sqrt(mean((v_gps_f - v_gps_phone).^2, 'omitnan'));
fprintf('\nRMSE between GPS-derived filtered speed and phone GPS speed: %.4f m/s\n', rmse_gps);
