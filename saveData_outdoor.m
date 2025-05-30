function rawData = saveData_outdoor(file, folder)
%% Setup
addpath('library\')

% Names of messages
globalMessage = '/mavros/global_position/local';
positionMessage = '/mavros/local_position/pose';
velocityMessage = '/mavros/local_position/velocity_body';
dataMessage = '/arm_4/stamped';
commandMessage = '/mavros/rc/in';
gpsMessage = '/mavros/global_position/global';
batteryMessage = '/mavros/battery';
imuMessage = '/mavros/imu/data';
stateMessage = '/mavros/state';
px4FlowMessage = '/mavros/px4flow/raw/optical_flow_rad';
escMessage = '/mavros/esc_status';
debugMessage = "/mavros/debug_value/debug_vector";

%% Loading bag file
bag = rosbag(fullfile(folder, file));

%% State
state = select(bag, 'Topic', stateMessage);
stateMsg = readMessages(state, 'DataFormat','struct');
tState = double( cellfun( @(x)x.Header.Stamp.Sec, stateMsg ) )+double( cellfun( @(x)x.Header.Stamp.Nsec, stateMsg ) )/10^9;

if ~isempty( tState )
    rawData.times.tState = tState - 3600; % adjusting for timezone
    rawData.data.state = double( cellfun( @(x)x.Armed, stateMsg ));
    
else
    rawData.times.tState = nan;
    rawData.data.state = zeros( 2, 0 );
end

%% PX4 Flow
px4flow = select(bag, 'Topic', px4FlowMessage);
px4flowMsg = readMessages(px4flow, 'DataFormat','struct');
tPX4flow = double( cellfun( @(x)x.Header.Stamp.Sec, px4flowMsg ) )+double( cellfun( @(x)x.Header.Stamp.Nsec, px4flowMsg ) )/10^9;

if ~isempty( tPX4flow )
    rawData.times.tPX4flow = tPX4flow - 3600; % adjusting for timezone
    rawData.data.px4flowXYZ = double( [cellfun( @(x)x.IntegratedX, px4flowMsg ),...
        cellfun( @(x)x.IntegratedY, px4flowMsg ),...
        cellfun( @(x)x.IntegratedXgyro, px4flowMsg ),...
        cellfun( @(x)x.IntegratedYgyro, px4flowMsg ),...
        cellfun( @(x)x.IntegratedZgyro, px4flowMsg ) ]);
    rawData.data.px4flowDist = double(cellfun( @(x)x.Distance, px4flowMsg));
    rawData.data.px4flowQual = double(cellfun( @(x)x.Quality, px4flowMsg ));
    
else
    rawData.times.tPX4flow = nan;
    rawData.data.px4flow = zeros( 2, 0 );
end

%% PX4 Flow
px4flow = select(bag, 'Topic', px4FlowMessage);
px4flowMsg = readMessages(px4flow, 'DataFormat','struct');
tPX4flow = double( cellfun( @(x)x.Header.Stamp.Sec, px4flowMsg ) )+double( cellfun( @(x)x.Header.Stamp.Nsec, px4flowMsg ) )/10^9;

if ~isempty( tPX4flow )
    rawData.times.tPX4flow = tPX4flow - 3600; % adjusting for timezone
    rawData.data.px4flowXYZ = double( [cellfun( @(x)x.IntegratedX, px4flowMsg ),...
        cellfun( @(x)x.IntegratedY, px4flowMsg ),...
        cellfun( @(x)x.IntegratedXgyro, px4flowMsg ),...
        cellfun( @(x)x.IntegratedYgyro, px4flowMsg ),...
        cellfun( @(x)x.IntegratedZgyro, px4flowMsg ) ]);
    rawData.data.px4flowDist = double(cellfun( @(x)x.Distance, px4flowMsg));
    rawData.data.px4flowQual = double(cellfun( @(x)x.Quality, px4flowMsg ));
    
else
    rawData.times.tPX4flow = nan;
    rawData.data.px4flow = zeros( 2, 0 );
end

%% Debug Message (used for instance where there are multiple distance sensors)
debug = select(bag, 'Topic', debugMessage);
debugMsg = readMessages(debug, 'DataFormat','struct');
tDebug = double( cellfun( @(x)x.Header.Stamp.Sec, debugMsg ) )+double( cellfun( @(x)x.Header.Stamp.Nsec, debugMsg ) )/10^9;

if ~isempty( tDebug )
    rawData.times.tDebug = tDebug - 3600; % adjusting for timezone
    rawData.data.debug = double(cell2mat(cellfun( @(x)x.Data.', debugMsg, 'UniformOutput', false)));
    
else
    rawData.times.tDebug = nan;
    rawData.data.debug = zeros( 3, 0 );
end

%% GPS Location
GPS = select( bag, 'Topic', gpsMessage );
gpsMes = readMessages( GPS, 'dataformat', 'struct' );
% extracting times from GPS data
tGPS = double( cellfun( @(x)x.Header.Stamp.Sec, gpsMes ) )+double( cellfun( @(x)x.Header.Stamp.Nsec, gpsMes ) )/10^9;

if ~isempty( tGPS )
    rawData.times.tGPS = tGPS - 3600; % adjusting for timezone
    rawData.data.GPS = double( [cellfun( @(x)x.Latitude, gpsMes ),...
        cellfun( @(x)x.Longitude, gpsMes ),...
        cellfun( @(x)x.Altitude, gpsMes )] );
    
    dateTimes = datetime(tGPS,'ConvertFrom','posixtime');
    startTime = tGPS(1);
    endTime = tGPS(end);
    
else
    rawData.times.tGPS = nan;
    rawData.data.GPS = zeros( 2, 3 );
    dateTimes = 0;
    startTime = 0;
    endTime = 0;
end

%% Position and Orientation
Pos = select( bag, 'Topic', positionMessage );
PosMes = readMessages( Pos, 'dataformat', 'struct' );
tPos = double( cellfun( @(x)x.Header.Stamp.Sec, PosMes ) )+double( cellfun( @(x)x.Header.Stamp.Nsec, PosMes ) )/10^9;
if ~isempty( tPos )
    rawData.times.tPos = tPos - 3600; % adjusting for timezone
    rawData.data.pos = [cellfun( @(x)x.Pose.Position.X, PosMes ),...
        cellfun( @(x)x.Pose.Position.Y, PosMes ),...
        cellfun( @(x)x.Pose.Position.Z, PosMes )];

    rawData.data.quat = [cellfun( @(x)x.Pose.Orientation.W, PosMes ),...
        cellfun( @(x)x.Pose.Orientation.X, PosMes ),...
        cellfun( @(x)x.Pose.Orientation.Y, PosMes ),...
        cellfun( @(x)x.Pose.Orientation.Z, PosMes )];

else
    rawData.times.tPos = nan;
    rawData.data.pos = zeros( 2, 3 );
    rawData.data.quat = zeros( 2, 4 );
    rawData.data.quat(:,1) = 1;
end

%% Velocity (Linear and Angular)
Vel = select( bag, 'Topic', velocityMessage );
VelMes = readMessages( Vel, 'dataformat', 'struct' );
tVel = double( cellfun( @(x)x.Header.Stamp.Sec, VelMes ) )+double( cellfun( @(x)x.Header.Stamp.Nsec, VelMes ) )/10^9;
if ~isempty( tVel )
    rawData.times.tVel = tVel - 3600; % adjusting for timezone
    rawData.data.linVel = [cellfun( @(x)x.Twist.Linear.X, VelMes ),...
        cellfun( @(x)x.Twist.Linear.Y, VelMes ),...
        cellfun( @(x)x.Twist.Linear.Z, VelMes )];

    rawData.data.angVel = [cellfun( @(x)x.Twist.Angular.X, VelMes ),...
        cellfun( @(x)x.Twist.Angular.Y, VelMes ),...
        cellfun( @(x)x.Twist.Angular.Z, VelMes )];

else
    rawData.times.tVel = nan;
    rawData.data.linVel = zeros( 2, 4 );
    rawData.data.angVel = zeros( 2, 4 );

end

%% Thrust, Strain, RPM, Current, and Voltage
Data = select( bag, 'Topic', dataMessage );
DataMes = readMessages( Data, 'dataformat', 'struct' );

if ~isempty(DataMes)
    tData = double( cellfun( @(x)x.Header.Stamp.Sec, DataMes ) )+double( cellfun( @(x)x.Header.Stamp.Nsec, DataMes ) )/10^9;
else
    dataMessage = '/arm_4';
    Data = select( bag, 'Topic', dataMessage );
    DataMes = readMessages( Data, 'dataformat', 'struct' );

    tData = double(Data.MessageList.Time);
end

if ~isempty( tData )
    rawData.times.tData = tData - 3600; % adjusting for timezone
    rawData.data.Voltage = double( [cellfun( @(x)x.V1, DataMes ),...
        cellfun( @(x)x.V2, DataMes ),...
        cellfun( @(x)x.V3, DataMes ),...
        cellfun( @(x)x.V4, DataMes )] );

    rawData.data.Current = double( [cellfun( @(x)x.C1, DataMes ),...
        cellfun( @(x)x.C3, DataMes ),...
        cellfun( @(x)x.C2, DataMes ),...
        cellfun( @(x)x.C4, DataMes )] );

    rawData.data.Thrust = double( [cellfun( @(x)x.THRUST1, DataMes ),...
        cellfun( @(x)x.THRUST2, DataMes ),...
        cellfun( @(x)x.THRUST3, DataMes ),...
        cellfun( @(x)x.THRUST4, DataMes )] );

    rawData.data.Strain = double( [cellfun( @(x)x.STRAIN1, DataMes ),...
        cellfun( @(x)x.STRAIN2, DataMes ),...
        cellfun( @(x)x.STRAIN3, DataMes ),...
        cellfun( @(x)x.STRAIN4, DataMes )] );

    rawData.data.RPM = double( [cellfun( @(x)x.RPM1, DataMes ),...
        cellfun( @(x)x.RPM2, DataMes ),...
        cellfun( @(x)x.RPM3, DataMes ),...
        cellfun( @(x)x.RPM4, DataMes )] );

else
    rawData.times.tData = nan;
    rawData.data.Voltage = zeros( 2, 4 );
    rawData.data.Current = zeros( 2, 4 );
    rawData.data.Thrust = zeros( 2, 4 );
    rawData.data.Strain = zeros( 2, 4 );
    rawData.data.Speed = zeros( 2, 4 );

end

%% Command
Command = select( bag, 'Topic', commandMessage );
commandMes = readMessages( Command, 'dataformat', 'struct' );

tCommand = double( cellfun( @(x)x.Header.Stamp.Sec, commandMes ) )+double( cellfun( @(x)x.Header.Stamp.Nsec, commandMes ) )/10^9;
if ~isempty( tCommand )
    channels = cellfun( @(x)x.Channels, commandMes, 'UniformOutput', false );
    indx = cellfun( @isempty, channels );

    channels(indx) = [];
    tCommand(indx) = [];

    rawData.times.tCommand = tCommand - 3600; % adjusting for timezone
    Command = double( cell2mat( channels ) );
    rawData.data.Command = reshape( Command, length( commandMes{1}.Channels ), [] )';
    rawData.data.armed = rawData.data.Command(:,8) - rawData.data.Command(1,8);

else
    rawData.times.tCommand = nan;
    rawData.data.Command = 0;

end

%% Battery data (from Pixhawk)
Battery = select( bag, 'Topic', batteryMessage );
BatteryMes = readMessages( Battery, 'dataformat', 'struct' );

tBattery = double( cellfun( @(x)x.Header.Stamp.Sec, BatteryMes ) )+double( cellfun( @(x)x.Header.Stamp.Nsec, BatteryMes ) )/10^9;
if ~isempty( tBattery )
    rawData.times.tBattery = tBattery - 3600; % adjusting for timezone
    rawData.data.pixhawkVoltage = double( cellfun( @(x)x.Voltage, BatteryMes ) );
    rawData.data.pixhawkCurrent = double( cellfun( @(x)x.Current, BatteryMes ) );

else
    rawData.times.tBattery = nan;
    rawData.data.pixhawkVoltage = zeros( 2, 1 );
    rawData.data.pixhawkCurrent = zeros( 2, 1 );

end

%% Global Position and Velocity
globalPos = select( bag, 'Topic', globalMessage );
globalPosMes = readMessages( globalPos, 'dataformat', 'struct' );

tGlobal = double( cellfun( @(x)x.Header.Stamp.Sec, globalPosMes ) )+double( cellfun( @(x)x.Header.Stamp.Nsec, globalPosMes ) )/10^9;
if ~isempty( tGlobal )
    rawData.times.tGlobal = tGlobal - 3600; % adjusting for timezone
    rawData.data.posGlobal = [cellfun( @(x)x.Pose.Pose.Position.X, globalPosMes ),...
        cellfun( @(x)x.Pose.Pose.Position.Y, globalPosMes ),...
        cellfun( @(x)x.Pose.Pose.Position.Z, globalPosMes )];

    rawData.data.quatGlobal = [cellfun( @(x)x.Pose.Pose.Orientation.W, globalPosMes ),...
        cellfun( @(x)x.Pose.Pose.Orientation.X, globalPosMes ),...
        cellfun( @(x)x.Pose.Pose.Orientation.Y, globalPosMes ),...
        cellfun( @(x)x.Pose.Pose.Orientation.Z, globalPosMes )];

    rawData.data.linVelGlobal = [cellfun( @(x)x.Twist.Twist.Linear.X, globalPosMes ),...
        cellfun( @(x)x.Twist.Twist.Linear.Y, globalPosMes ),...
        cellfun( @(x)x.Twist.Twist.Linear.Z, globalPosMes )];

    rawData.data.angVelGlobal = [cellfun( @(x)x.Twist.Twist.Angular.X, globalPosMes ),...
        cellfun( @(x)x.Twist.Twist.Angular.Y, globalPosMes ),...
        cellfun( @(x)x.Twist.Twist.Angular.Z, globalPosMes )];

else
    rawData.times.tGlobal = nan;
    rawData.data.posGlobal = zeros( 2, 4 );
    rawData.data.quatGlobal = zeros( 2, 4 );
    rawData.data.quatGlobal(:,1) = 1;
    rawData.data.linVelGlobal = zeros( 2, 4 );
    rawData.data.angVelGlobal = zeros( 2, 4 );

end

%% Acceleration
IMU = select( bag, 'Topic', imuMessage );
imuMes = readMessages( IMU, 'dataformat', 'struct' );

tIMU = double( cellfun( @(x)x.Header.Stamp.Sec, imuMes ) )+double( cellfun( @(x)x.Header.Stamp.Nsec, imuMes ) )/10^9;
if ~isempty( tIMU )
    rawData.times.tIMU = tIMU - 3600; % adjusting for timezone
    rawData.data.linAccel = [cellfun( @(x) x.LinearAcceleration.X, imuMes ),...
        cellfun( @(x) x.LinearAcceleration.Y, imuMes ),...
        cellfun( @(x) x.LinearAcceleration.Z, imuMes ) ];

else
    rawData.times.tIMU = nan;
    rawData.data.linAccel = zeros( 2, 3 );

end

%% ESC RPM, Current, and Voltage
ESC = select(bag, 'Topic', escMessage);
escMsg = readMessages(ESC, 'DataFormat','struct');
tESC = double( cellfun( @(x)x.Header.Stamp.Sec, escMsg ) )+double( cellfun( @(x)x.Header.Stamp.Nsec, escMsg ) )/10^9;

if ~isempty( tESC )
    rawData.times.tESC = tESC - 3600; % adjusting for timezone
    rawData.data.ESC.RPM = double([cellfun( @(x)x.EscStatus(1).Rpm, escMsg),...
        cellfun( @(x)x.EscStatus(2).Rpm, escMsg),...
        cellfun( @(x)x.EscStatus(3).Rpm, escMsg),...
        cellfun( @(x)x.EscStatus(4).Rpm, escMsg)]);

    rawData.data.ESC.Current = double([cellfun( @(x)x.EscStatus(1).Current, escMsg),...
        cellfun( @(x)x.EscStatus(2).Current, escMsg),...
        cellfun( @(x)x.EscStatus(3).Current, escMsg),...
        cellfun( @(x)x.EscStatus(4).Current, escMsg)]);

    rawData.data.ESC.Voltage = double([cellfun( @(x)x.EscStatus(1).Voltage, escMsg),...
        cellfun( @(x)x.EscStatus(2).Voltage, escMsg),...
        cellfun( @(x)x.EscStatus(3).Voltage, escMsg),...
        cellfun( @(x)x.EscStatus(4).Voltage, escMsg)]);
else
    rawData.times.tESC = nan;
    rawData.data.ESC.RPM = zeros( 4, 0 );
end

%% Saving to .mat file
matname = strcat(file(1:19),".mat");
save( fullfile(folder, matname), 'rawData');

end