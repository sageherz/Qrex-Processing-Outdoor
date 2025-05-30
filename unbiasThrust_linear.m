function Thrust_unbias = unbiasThrust_linear(Thrust_filt, flight_pts)

start_idx = [1, flight_pts(1)]; end_idx = [flight_pts(2), length(Thrust_filt)]; % start/end unloaded indices

for ARM = 1:width(Thrust_filt) % for each arm...
    Thrust_0 = [mean(Thrust_filt(start_idx(1):start_idx(2),ARM)),...
        mean(Thrust_filt(end_idx(1):end_idx(2),ARM))]; % unloaded thrust values (start and end)
    bias_fit = polyfit([mean(start_idx), mean(end_idx)], Thrust_0, 1); % linear bias fit
    bias_val = polyval(bias_fit, 1:length(Thrust_filt));
    Thrust_unbias(:,ARM) = Thrust_filt(:,ARM) - bias_val.'; % subtracting bias from filtered thrust data
end

end