function avgVals = averageEWlegs_updated(data,pos,startMission,endMission,qCrop)

if qCrop == 1 % cropping data if went into hold during a leg
% Thrust_scale([59083:61698,100535:103270,134251:135845],:) = [];
% idxDiff = length(linVel) - length(Thrust_scale);
% linVel([59083:61698,100535:103270,134251:135845],:) = [];
% RPM_scale([59083:61698,100535:103270,134251:135845],:) = [];
% endMission = endMission - idxDiff;

% data(97278:99142,:) = [];
% idxDiff = length(pos) - length(data);
% pos(97278:99142,:) = [];
% endMission = endMission - idxDiff;

% 10/23 (4th flight):
% data(94430:97091,:) = [];
% idxDiff = length(pos) - length(data);
% pos(94430:97091,:) = [];
% endMission = endMission - idxDiff;

% % 09/22/23:
% data(43651:45372,:) = [];
% idxDiff = length(pos) - length(data);
% pos(43651:45372,:) = [];
% endMission = endMission - idxDiff;

% 09/26/23:
% data(27380:30874,:) = [];
% idxDiff = length(pos) - length(data);
% pos(27380:30874,:) = [];
% endMission = endMission - idxDiff;
end

% finding when vehicle is flying over home position (middle of each flight
% leg):
home_idx = []; 
for k = startMission:endMission
    previous = pos(k-1,1);
    current = pos(k,1);
    if ~isequal(sign(previous), sign(current))
        home_idx = cat(1,home_idx,k);
    end
end

% finding (end of) hover locations:
hov_pts_all = find(abs(diff(medfilt1(pos(startMission:endMission,1),100))) < 0.0005); % all hover points
hov_pts_all_diff = diff(hov_pts_all);
hov_idx = find(hov_pts_all_diff > 1000);
hov_idx = hov_idx - 1;
hov_idx = hov_pts_all(hov_idx);
hov_idx = hov_idx + startMission;

%data_avg = [];
for i = 1:width(data)
    data_avg.(strcat("Column",string(i))) = [];
end
xtick = [];
for j = 1:2:length(home_idx)-1
    ElegMidPt = home_idx(j);
    WlegMidPt = home_idx(j+1);

    EhovMidPt = hov_idx(j) - 550;
    WhovMidPt = hov_idx(j) - 550;

    for i = 1:width(data)
        Edata = []; %#ok<*NASGU> 
        Wdata = [];
        Edata = [data(floor(ElegMidPt-2000):floor(ElegMidPt+2000),i);...
            data(floor(EhovMidPt-500):floor(EhovMidPt+500),i)];
        Wdata = [data(floor(WlegMidPt-2000):floor(WlegMidPt+2000),i);...
            data(floor(WhovMidPt-500):floor(WhovMidPt+500),i)];
        data_avg.(strcat("Column",string(i))) = cat(1,data_avg.(strcat("Column",string(i))),mean([Edata,Wdata],2)); %#ok<*STRNU> 
    end

    %skewAngles_avg = cat(1,skewAngles_avg,skewAngles(ceil(j/2))*ones(length(Ethrust),1));
    xtick(ceil(j/2)) = 0.4*(length(Wdata)) + (length(xtick))*(length(Wdata));
end

% for i = 1:4
%     %thrust_avg.(strcat("arm",string(i))) = [Thrust_scale(1:fwdLocs(1),i);thrust_legs.(strcat("arm",string(i)));Thrust_scale(endMission:end,i)];
%     thrust_avg.(strcat("arm",string(i))) = [Thrust_scale(1:fwdLocs(1),i);thrust_legs.(strcat("arm",string(i)))];
%     rpm_avg.(strcat("arm",string(i))) = [RPM_scale(1:fwdLocs(1),i);rpm_legs.(strcat("arm",string(i)));RPM_scale(endMission:end,i)];
% end

avgVals.data = data_avg; avgVals.xtick = xtick;

end