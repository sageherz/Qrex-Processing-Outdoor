function rawData = saveData_ROS2(file, folder)
%% Reading ROS data file and printing file information
bagpath = fullfile(folder, file); % defining path to bag file
bagReader = ros2bagreader(bagpath);
baginfo = ros2("bag","info",bagpath) %#ok<NOPRT,NASGU>

%% Selecting ROS2 topic
bagSel_Qrex = select(bagReader,"Topic","/Qrex"); % topic containing current, voltage, rpm, and thrust data
bagSel_ESC = select(bagReader,"Topic","/mavros/esc_status/status"); % topic containing ESC data
bagSel_px4flow = select(bagReader,"Topic","/mavros/px4flow/ground_distance"); % topic containing px4 flow ground distance data
bagSel_IMU = select(bagReader,"Topic","/mavros/imu/data"); % topic containing IMU data
bagSel_Vel = select(bagReader,"Topic","/mavros/local_position/velocity_body"); % topic containing body velocity data
bagSel_Pos = select(bagReader,"Topic","/mavros/local_position/pose"); % topic containing local position data
bagSel_debug = select(bagReader,"Topic","/mavros/debug_value/debug_vector"); % topic containing debug vector data (aka multiple distance sensor data)
bagSel_Pressure = select(bagReader,"Topic","/Pressure"); % topic containing Kiel probe pressure data (fast)
bagSel_PressureString = select(bagReader,"Topic","/Pressure_String"); % topic containing Kiel probe pressure data (slow)

%% Reading specific topic messages
msgsFiltered_Qrex = readMessages(bagSel_Qrex);
msgsFiltered_ESC = readMessages(bagSel_ESC);
msgsFiltered_px4flow = readMessages(bagSel_px4flow);
msgsFiltered_IMU = readMessages(bagSel_IMU);
msgsFiltered_Vel = readMessages(bagSel_Vel);
msgsFiltered_Pos = readMessages(bagSel_Pos);
msgsFiltered_debug = readMessages(bagSel_debug);
msgsFiltered_Pressure = readMessages(bagSel_Pressure);
msgsFiltered_PressureString = readMessages(bagSel_PressureString);

%% Extracting time and data, MSG: /Qrex
tQrex = double( cellfun( @(x)x.timestamp.sec, msgsFiltered_Qrex ) )+double( cellfun( @(x)x.timestamp.nanosec, msgsFiltered_Qrex ) )/10^9;

if ~isempty( tQrex )
    rawData.times.tQrex = tQrex - 3600; % adjusting for time zone

    rawData.data.Current = double([cellfun( @(x)x.current(1), msgsFiltered_Qrex),...
        cellfun( @(x)x.current(2), msgsFiltered_Qrex),...
        cellfun( @(x)x.current(3), msgsFiltered_Qrex),...
        cellfun( @(x)x.current(4), msgsFiltered_Qrex)]);

    rawData.data.Thrust = double([typecast(int16(cellfun( @(x)x.thrust(1), msgsFiltered_Qrex)), "uint16"),...
        typecast(int16(cellfun( @(x)x.thrust(2), msgsFiltered_Qrex)), "uint16"),...
        typecast(int16(cellfun( @(x)x.thrust(3), msgsFiltered_Qrex)), "uint16"),...
        typecast(int16(cellfun( @(x)x.thrust(4), msgsFiltered_Qrex)), "uint16")]);

    rawData.data.RPM = double([cellfun( @(x)x.rpm(1), msgsFiltered_Qrex),...
        cellfun( @(x)x.rpm(2), msgsFiltered_Qrex),...
        cellfun( @(x)x.rpm(3), msgsFiltered_Qrex),...
        cellfun( @(x)x.rpm(4), msgsFiltered_Qrex)]);

    rawData.data.Voltage = double(cellfun( @(x)x.voltage(1), msgsFiltered_Qrex));
else
    rawData.times.tQrex = nan;
    rawData.data.Current = nan(1, 4);
    rawData.data.Thrust = nan(1, 4);
    rawData.data.RPM = nan(1, 4);
    rawData.data.Voltage = nan;
end

%% Extracting time and data, MSG: ESC
tESC = double( cellfun( @(x)x.header.stamp.sec, msgsFiltered_ESC ) )+double( cellfun( @(x)x.header.stamp.nanosec, msgsFiltered_ESC ) )/10^9;

if ~isempty( tESC )
    rawData.times.tESC = tESC - 3600; % adjusting for time zone

    rawData.data.ESC.RPM = double([cellfun( @(x)x.esc_status(1).rpm, msgsFiltered_ESC),...
        cellfun( @(x)x.esc_status(2).rpm, msgsFiltered_ESC),...
        cellfun( @(x)x.esc_status(3).rpm, msgsFiltered_ESC),...
        cellfun( @(x)x.esc_status(4).rpm, msgsFiltered_ESC)])./2;

else
    rawData.times.tESC = nan;
    rawData.data.ESC.RPM = nan(1, 4);
end

%% Extracting time and data, MSG: px4flow
tPX4flow = double( cellfun( @(x)x.header.stamp.sec, msgsFiltered_px4flow ) )+double( cellfun( @(x)x.header.stamp.nanosec, msgsFiltered_px4flow ) )/10^9;

if ~isempty( tPX4flow )
    rawData.times.tPX4flow = tPX4flow - 3600; % adjusting for time zone

    rawData.data.px4FlowDist = double(cellfun( @(x)x.range, msgsFiltered_px4flow));
else
    rawData.times.tPX4flow = nan;
    rawData.data.px4FlowDist = nan;
end

%% Extracting time and data, MSG: IMU
tIMU = double( cellfun( @(x)x.header.stamp.sec, msgsFiltered_IMU ) )+double( cellfun( @(x)x.header.stamp.nanosec, msgsFiltered_IMU ) )/10^9;

if ~isempty( tIMU )
    rawData.times.tIMU = tIMU - 3600; % adjusting for time zone

    rawData.data.quat = double([cellfun( @(x)x.orientation.x, msgsFiltered_IMU),...
        cellfun( @(x)x.orientation.y, msgsFiltered_IMU),...
        cellfun( @(x)x.orientation.z, msgsFiltered_IMU),...
        cellfun( @(x)x.orientation.w, msgsFiltered_IMU)]);
else
    rawData.times.tIMU = nan;
    rawData.data.quat = nan(1, 4);
end

%% Extracting time and data, MSG: body velocity
tVel = double( cellfun( @(x)x.header.stamp.sec, msgsFiltered_Vel ) )+double( cellfun( @(x)x.header.stamp.nanosec, msgsFiltered_Vel ) )/10^9;

if ~isempty( tVel )
    rawData.times.tVel = tVel - 3600; % adjusting for time zone

    rawData.data.linVel = double([cellfun( @(x)x.twist.linear.x, msgsFiltered_Vel),...
        cellfun( @(x)x.twist.linear.y, msgsFiltered_Vel),...
        cellfun( @(x)x.twist.linear.z, msgsFiltered_Vel)]);

    rawData.data.angVel = double([cellfun( @(x)x.twist.angular.x, msgsFiltered_Vel),...
        cellfun( @(x)x.twist.angular.y, msgsFiltered_Vel),...
        cellfun( @(x)x.twist.angular.z, msgsFiltered_Vel)]);
else
    rawData.times.tVel = nan;
    rawData.data.linVel = nan(1, 3);
    rawData.data.angVel = nan(1, 3);
end

%% Extracting time and data, MSG: local position
tPos = double( cellfun( @(x)x.header.stamp.sec, msgsFiltered_Pos ) )+double( cellfun( @(x)x.header.stamp.nanosec, msgsFiltered_Pos ) )/10^9;

if ~isempty( tPos )
    rawData.times.tPos = tPos - 3600; % adjusting for time zone

    rawData.data.pos = double([cellfun( @(x)x.pose.position.x, msgsFiltered_Pos),...
        cellfun( @(x)x.pose.position.y, msgsFiltered_Pos),...
        cellfun( @(x)x.pose.position.z, msgsFiltered_Pos)]);
else
    rawData.times.tPos = nan;
    rawData.data.pos = nan(1, 3);
end

%% Extracting time and data, MSG: debug vector
tDebug = double( cellfun( @(x)x.header.stamp.sec, msgsFiltered_debug ) )+double( cellfun( @(x)x.header.stamp.nanosec, msgsFiltered_debug ) )/10^9;

if ~isempty( tDebug )
    rawData.times.tDebug = tDebug - 3600; % adjusting for time zone

    rawData.data.debug = double([cellfun( @(x)x.data(1), msgsFiltered_debug),...
        cellfun( @(x)x.data(2), msgsFiltered_debug),...
        cellfun( @(x)x.data(2), msgsFiltered_debug)]);

else
    rawData.times.tDebug = nan;
    rawData.data.debug = nan(1, 3);
end

%% Extracting time and data, MSG: /Pressure ("fast" pressure data)
tPressure = double( cellfun( @(x)x.timestamp.sec, msgsFiltered_Pressure ) )+double( cellfun( @(x)x.timestamp.nanosec, msgsFiltered_Pressure ) )/10^9;

if ~isempty(tPressure)
    rawData.times.tPressure = tPressure - 3600; % adjusting for time zone

    rawData.data.Pressure = zeros(length(msgsFiltered_Pressure),32); % initializing data vector
    for i = 1:length(msgsFiltered_Pressure) % going through all pressure data...
        rawData.data.Pressure(i,:) = double(msgsFiltered_Pressure{i}.pressure); 
    end

else
    rawData.times.tPressure = nan;
    rawData.data.Pressure = nan(1, 32);
end


%% Extracting time and data, MSG: /PressureString ("slow" pressure data)
% tPressure = []; pressure_data = []; % initializing time and data vectors
% for i = 1:length(msgsFiltered_PressureString)
%     serial_data = msgsFiltered_PressureString{i,1}.data; % getting each line of serial data
%     serial_data_split = strsplit(serial_data, ":"); % splitting time and pressure data
%     % tPressure = cat(1, tPressure, double(string(serial_data_split(1)))); % time stamp
%     if length(split(serial_data_split(end),",")) == 35
%         pressure_data = cat(1, pressure_data, serial_data_split(end)); % pressure data
%     end
% end
% 
% pressure_data_split = split(pressure_data, ","); pressure_data_split(:,1) = [];
% 
% % writing to rawData structure:
% rawData.times.tPressure = tPressure - 3600;
% rawData.data.PBoard.Pressure = double(string(pressure_data_split(:,1:32)));
% rawData.data.PBoard.Temp = double(string(pressure_data_split(:,33:34)));

%% Saving to .mat file
matname = strcat(file(1:29),".mat");
save( fullfile(folder(1:length(folder)-28), matname), 'rawData');%% Saving to .mat file

end