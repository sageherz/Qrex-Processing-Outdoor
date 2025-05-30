clear
close all
clc

addpath("library\")

%% ENTER AMBIENT AIR CONDITIONS FROM TIME OF FLIGHT TEST
T = 22; % room temperature (C)
hr = 16; % humidity (%)
P = 99200; % pressure (Pa)

%% Getting ambient room density
rho = air_density(T,hr,P);

%% Rotor geometry 
R = 0.1905; % rotor radius (m)
A = pi*R^2; % rotor disk area (m^2)

%% Save data from ROS (1 or 2) file to .mat file
[rosfile, rospath] = uigetfile({'*.db3';'*.bag'}, 'Select Bag File', 'multiselect', 'on' ); % getting file and path of .bag or .db3 file

rosfile_type = rosfile(end-2:end); % db3 (ROS2) or bag (ROS1) file extension
if strcmp(rosfile_type, "bag") == 1
    ROS_ver = 1; % ROS version (ROS1)
    rawData = saveData_outdoor(rosfile,rospath); % returns name of file with extracted data from bag file (ROS1)
    matname = strcat(rosfile(1:19));
elseif strcmp(rosfile_type, "db3") == 1
    ROS_ver = 0; % ROS version (ROS2)
    rawData = saveData_ROS2(rosfile,rospath); % returns name of file with extracted data from db3 file (ROS2)
    matname = strcat(rosfile(1:29));
    rospath = rospath(1:length(rospath)-28);
end

%% Save data from log file to .mat file
[logfile, logpath] = uigetfile( '*.ulg', 'Select Log File', 'multiselect', 'on' ); % getting file and path of .ulg file
logData = Ulog_Parser_Function(logfile,logpath);

%% Process thrust data (unbias, filter, scale)
Thrust_filt = medfilt1(rawData.data.Thrust,100); % filtering raw thrust
Thrust_filt(1,:) = [];

% plotting filtered thrust to select takeoff and landing points
figure
plot(Thrust_filt)
title("Select takeoff and landing points")
[flight_pts,~] = getpts; flight_pts = round(flight_pts,0);
if length(flight_pts) < 2 % constant unbias (subtracting off start/end point, whichever is provided)
    Thrust_unbias = unbiasThrust_constant(Thrust_filt, flight_pts);
else % linear unbias
    Thrust_unbias = unbiasThrust_linear(Thrust_filt, flight_pts);
end

save(fullfile(rospath, matname),"flight_pts","-append") % saving takeoff/landing points to mat file

kThrust = [0.000943, 0.000968, 0.00111, 0.000891]; % load cell scale factors (s-beam)
Thrust_scale = kThrust.*Thrust_unbias;

%%  Aligning and scaling RPM data (from ESC)
if ROS_ver % checking ROS version...
    tQrex = rawData.times.tData;
else
    tQrex = rawData.times.tQrex;
end

if ~isempty(rawData.data.ESC.RPM)
    RPM_filt = medfilt1(rawData.data.ESC.RPM,15); % filtering raw RPM data
    tESC = rawData.times.tESC;

    for i = 1:length(tQrex)
        [~,corr_idx(i)] = min(abs(tESC-tQrex(i))); %#ok<*SAGROW>
    end
    RPM_align = RPM_filt(corr_idx,:); RPM_align(1,:) = []; % aligning RPM to thrust data
    RPM_scale = RPM_align; % scaling RPM data
else
    RPM_filt = medfilt1(rawData.data.RPM,15); % filtering raw RPM data
    tESC = rawData.times.tESC;

    RPM_scale = [];
end

%% Aligning and scaling RPM data (from ESC) if not recorded in bag file
if isempty(RPM_scale)
    RPM_log = double([logData.esc_status.A.esc_0__esc_rpm,...
        logData.esc_status.A.esc_1__esc_rpm,...
        logData.esc_status.A.esc_2__esc_rpm,...
        logData.esc_status.A.esc_3__esc_rpm]); % ESC RPM from log file
    RPM_log_filt = RPM_log; % filtering raw RPM data
    tLog = seconds(logData.esc_status.A.timestamp);
    tPi = tQrex;

    % finding take off points:
    figure
    plot(RPM_filt)
    title("RPM from pi")
    figure
    plot(RPM_log_filt)
    title("RPM from log")

    takeOffPt_pi = 5335; takeOffPt_log = 15;
    t_delta = tPi(takeOffPt_pi) - tLog(takeOffPt_log); % time offset between pi and pixhawk

    % sanity check plot
    figure
    plot(tLog+t_delta, RPM_log_filt(:,4))
    hold on
    plot(tPi, rawData.data.RPM(:,4))

    tESC = tLog + t_delta; % time vector aligned to pi
    clear("corr_idx");
    for i = 1:length(tQrex)
        [~,corr_idx(i)] = min(abs(tESC-tQrex(i)));
    end
    RPM_align = RPM_log_filt(corr_idx,:);
end

%% Computing CT
Omega = RPM_scale.*((2*pi)/60); % converting RPM to rad/s
CT = Thrust_scale./(rho*((Omega.*R).^2)*A);

%% Calculating power (from ESC or Pi)
power_source = "Pi"; % ENTER SELECTION FOR SOURCE OF POWER DATA (either from ESC ("ESC") or from Pi ("Pi"))

% filtering current and voltage
if strcmp(power_source,"Pi") == 1
    Current_filt = medfilt1(rawData.data.Current,100); Current_filt(1,:) = [];
    Voltage_filt = medfilt1(rawData.data.Voltage(:,end),100); Voltage_filt(1,:) = [];

    kCurrent = [0.02082, 0.02002, 0.02064, 0.02036];
    Current_filt = Current_filt - Current_filt(1,:);
    Current_align = kCurrent.*Current_filt;

    kVoltage = 0.006375425; cVoltage = -0.0882915;
    Voltage_align = kVoltage*Voltage_filt + cVoltage;
elseif strcmp(power_source,"ESC") == 1
    Current_filt = medfilt1(rawData.data.ESC.Current,10); Current_filt(1,:) = [];
    Voltage_filt = medfilt1(rawData.data.ESC.Voltage,10); Voltage_filt(1,:) = [];

    Current_align = Current_filt(corr_idx,:); % aligning Current to thrust data
    Voltage_align = Voltage_filt(corr_idx,:); % aligning Voltage to thrust data
end

Power = Current_align.*Voltage_align; % individual arm power (W)
totPower = sum(Power,2); % total power (W)

%% Aligning local position (x, y) from ROS file to thrust data
pos = rawData.data.pos;
tPos = rawData.times.tPos;
clear("corr_idx");
for i = 1:length(tQrex)
    [~,corr_idx(i)] = min(abs(tPos-tQrex(i)));
end
pos_align = pos(corr_idx,:);

%% Aligning angular rate data from ROS file to thrust data (NO ANGULAR VELOCITY DATA FOR ROS2)
angVel = rawData.data.angVel;
tVel = rawData.times.tVel;
clear("corr_idx");
for i = 1:length(tQrex)
    [~,corr_idx(i)] = min(abs(tVel-tQrex(i)));
end
angVel_align = angVel(corr_idx,:);

%% Aligning linear velocity data from ROS file to thrust data
linVel = rawData.data.linVel;
clear("corr_idx");
for i = 1:length(tQrex)
    [~,corr_idx(i)] = min(abs(tVel-tQrex(i)));
end
linVel_align = linVel(corr_idx,:);

%% Aligning linear velocity data from ROS file to thrust data
tIMU = rawData.times.tIMU;
quat = rawData.data.quat;
clear("corr_idx");
for i = 1:length(tQrex)
    [~,corr_idx(i)] = min(abs(tIMU-tQrex(i)));
end
quat_align = quat(corr_idx,:);

[yaw, pitch, roll] = quat2angle([quat_align(:,4), quat_align(:,1:3)]);
yaw = rad2deg(yaw); pitch = rad2deg(pitch); roll = rad2deg(roll);

%% Saving data to main .mat file
save(fullfile(rospath, matname))
