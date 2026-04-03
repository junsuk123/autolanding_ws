function [X_train, y_train, X_val, y_val, data_summary] = autlLoadAndPrepareTrainingData(rootDir, collection_dir, num_train, num_val)
% autlLoadAndPrepareTrainingData
% Load raw collected data and prepare feature/label pairs for training
%
% Returns:
%   X_train: [num_train, num_features] - training features
%   y_train: [num_train, 1] - training labels (landing success/fail)
%   X_val: [num_val, num_features] - validation features
%   y_val: [num_val, 1] - validation labels
%   data_summary: struct with data statistics

fprintf('[DataLoader] Searching for collected scenarios...\n');

% Find all raw_data.mat files from parallel collection
scenario_files = dir(fullfile(collection_dir, 'worker_*', 'scenario_*', 'raw_data.mat'));
num_scenarios = numel(scenario_files);

if num_scenarios == 0
    error('[DataLoader] No raw data files found in %s', collection_dir);
end

fprintf('[DataLoader] Found %d scenarios\n', num_scenarios);

% Load all scenarios and extract features
all_X = [];
all_y = [];

for i = 1:min(num_scenarios, num_train + num_val)
    raw_file = fullfile(scenario_files(i).folder, scenario_files(i).name);
    
    try
        raw_data = autlLoadScenarioRawData(raw_file);
        
        % Extract features from raw data
        % Features: [mean_pos, mean_vel, mean_alt, max_alt, position_variance, velocity_variance, battery_trend]
        features = autlExtractFeaturesFromRawData(raw_data);
        
        % Label: 1 = valid flight with landing-like terminal behavior, 0 = otherwise.
        % This explicitly rejects no-takeoff/no-motion runs that used to look like false TP.
        pos = raw_data.position_xyz;
        z = pos(:, 3);
        xy = pos(:, 1:2);
        alt_gain = max(z) - min(z);
        xy_span = max(max(xy, [], 1) - min(xy, [], 1));
        if size(xy, 1) >= 2
            dxy = diff(xy, 1, 1);
            xy_travel = sum(hypot(dxy(:, 1), dxy(:, 2)));
        else
            xy_travel = 0;
        end

        armed_ratio = 0;
        if isfield(raw_data, 'armed_state') && ~isempty(raw_data.armed_state)
            armed_ratio = mean(double(raw_data.armed_state) > 0.5);
        end

        final_dist_xy = inf;
        if isfield(raw_data, 'landing_pad_distance_xy') && ~isempty(raw_data.landing_pad_distance_xy)
            final_dist_xy = double(raw_data.landing_pad_distance_xy(end));
        elseif isfield(raw_data, 'landing_pad_center_xyz') && ~isempty(raw_data.landing_pad_center_xyz)
            lp = raw_data.landing_pad_center_xyz(end, 1:2);
            final_dist_xy = norm(xy(end, :) - lp);
        end

        final_window = max(1, size(pos, 1) - 10):size(pos, 1);
        pos_variance = var(pos(final_window, :), [], 1);
        terminal_stable = mean(pos_variance) < 0.08;
        flight_happened = (alt_gain >= 0.8) && ((xy_span >= 0.5) || (xy_travel >= 3.0)) && (armed_ratio >= 0.3);
        landing_success = flight_happened && (final_dist_xy <= 1.0) && terminal_stable;
        
        all_X = [all_X; features];
        all_y = [all_y; landing_success];
        
        if mod(i, 10) == 0
            fprintf('[DataLoader] Loaded %d/%d scenarios\n', i, min(num_scenarios, num_train + num_val));
        end
        
    catch ME
        fprintf('[DataLoader] Warning: Failed to load scenario %d: %s\n', i, ME.message);
    end
end

num_loaded = size(all_X, 1);
fprintf('[DataLoader] Successfully loaded %d scenarios\n', num_loaded);

if num_loaded < (num_train + num_val)
    fprintf('[DataLoader] Warning: Fewer scenarios than requested. Adjusting sizes.\n');
    if num_loaded >= 2
        num_val = max(1, floor(num_loaded * 0.25));
        num_train = num_loaded - num_val;
    else
        num_train = num_loaded;
        num_val = 0;
    end
end

% Guard against invalid split boundaries on tiny datasets.
num_train = min(num_train, num_loaded);
num_val = min(num_val, max(0, num_loaded - num_train));

% Shuffle data
shuffle_idx = randperm(size(all_X, 1));
all_X = all_X(shuffle_idx, :);
all_y = all_y(shuffle_idx, :);

% Split train/val
X_train = all_X(1:num_train, :);
y_train = all_y(1:num_train, :);
X_val = all_X(num_train+1:num_train+num_val, :);
y_val = all_y(num_train+1:num_train+num_val, :);

% Normalize features (z-score)
X_mean = mean(X_train, 1);
X_std = std(X_train, [], 1) + 1e-6;  % Avoid division by zero
X_train = (X_train - X_mean) ./ X_std;
X_val = (X_val - X_mean) ./ X_std;

% Statistics
data_summary = struct();
data_summary.num_train = size(X_train, 1);
data_summary.num_val = size(X_val, 1);
data_summary.num_features = size(X_train, 2);
data_summary.train_pos_rate = mean(y_train);
if isempty(y_val)
    data_summary.val_pos_rate = NaN;
else
    data_summary.val_pos_rate = mean(y_val);
end
data_summary.X_mean = X_mean;
data_summary.X_std = X_std;

fprintf('[DataLoader] Feature normalization complete\n');
fprintf('[DataLoader]   Training: %d samples (%.1f%% positive)\n', ...
    data_summary.num_train, data_summary.train_pos_rate * 100);
if isnan(data_summary.val_pos_rate)
    fprintf('[DataLoader]   Validation: %d samples (N/A)\n', data_summary.num_val);
else
    fprintf('[DataLoader]   Validation: %d samples (%.1f%% positive)\n', ...
        data_summary.num_val, data_summary.val_pos_rate * 100);
end

end

function raw_data = autlLoadScenarioRawData(raw_file)
% Load either checkpoint-style or final-style raw data.

data = load(raw_file);
if isfield(data, 'raw_data_trimmed')
    raw_data = data.raw_data_trimmed;
elseif isfield(data, 'raw_data')
    raw_data = data.raw_data;
else
    error('Variable ''raw_data'' or ''raw_data_trimmed'' not found in %s', raw_file);
end
end

function features = autlExtractFeaturesFromRawData(raw_data)
% Extract 14-feature vector from raw telemetry data
%
% Features extracted:
% 1-3: Position stats (norm, mean velocity norm, altitude)
% 4-5: Altitude stats (max, std)
% 6-8: Attitude stats (roll, pitch, yaw rate)
% 9-10: Battery stats (mean, min)
% 11-12: Thrust stats (mean, std)
% 13: GPS availability ratio
% 14: Armed time ratio

features = zeros(1, 14);
idx = 1;

% Position features
if isfield(raw_data, 'position_xyz') && size(raw_data.position_xyz, 1) > 0
    pos = raw_data.position_xyz;
    pos_norm = vecnorm(pos, 2, 2);
    features(idx) = mean(pos_norm);  % Mean position norm
    idx = idx + 1;
    
    vel = diff(pos, 1, 1);  % Velocity from position derivative
    vel_norm = vecnorm(vel, 2, 2);
    features(idx) = mean(vel_norm);  % Mean velocity norm
    idx = idx + 1;
    
    features(idx) = mean(pos(:, 3));  % Mean altitude
    idx = idx + 1;
    features(idx) = max(pos(:, 3));   % Max altitude
    idx = idx + 1;
    features(idx) = std(pos(:, 3));   % Altitude std
    idx = idx + 1;
else
    features(idx:idx+4) = 0;
    idx = idx + 5;
end

% Attitude features
if isfield(raw_data, 'attitude_rpy') && size(raw_data.attitude_rpy, 1) > 0
    att = raw_data.attitude_rpy;
    features(idx) = mean(att(:, 1));  % Mean roll
    idx = idx + 1;
    features(idx) = mean(att(:, 2));  % Mean pitch
    idx = idx + 1;
    
    if size(att, 2) >= 3
        features(idx) = mean(att(:, 3));  % Mean yaw rate
    else
        features(idx) = 0;
    end
    idx = idx + 1;
else
    features(idx:idx+2) = 0;
    idx = idx + 3;
end

% Battery features
if isfield(raw_data, 'battery_voltage') && numel(raw_data.battery_voltage) > 0
    batt = raw_data.battery_voltage;
    features(idx) = mean(batt);  % Mean battery voltage
    idx = idx + 1;
    features(idx) = min(batt);   % Min battery voltage
    idx = idx + 1;
else
    features(idx:idx+1) = 0;
    idx = idx + 2;
end

% Thrust features
if isfield(raw_data, 'thrust_percent') && numel(raw_data.thrust_percent) > 0
    thrust = raw_data.thrust_percent;
    features(idx) = mean(thrust);  % Mean thrust
    idx = idx + 1;
    features(idx) = std(thrust);   % Thrust std
    idx = idx + 1;
else
    features(idx:idx+1) = 0;
    idx = idx + 2;
end

% GPS availability
if isfield(raw_data, 'gps_fix') && numel(raw_data.gps_fix) > 0
    gps_available = raw_data.gps_fix > 0;
    features(idx) = mean(gps_available);
else
    features(idx) = 0;
end
idx = idx + 1;

% Armed time ratio
if isfield(raw_data, 'armed_state') && numel(raw_data.armed_state) > 0
    features(idx) = mean(raw_data.armed_state);
else
    features(idx) = 0;
end

end
