function Thrust_unbias = unbiasThrust_constant(Thrust_filt, flight_pts)

[~, i] = min([flight_pts-1, length(Thrust_filt)-flight_pts]); % checking if takeoff or landing point is to be used for bias subtraction

if i == 1 % using takeoff point...
    Thrust_0 = mean(Thrust_filt(1:flight_pts,:)); % unloaded thrust value for each arm
    Thrust_unbias = Thrust_filt - Thrust_0;
else % using landing point...
    Thrust_0 = mean(Thrust_filt(flight_pts:end,:)); % unloaded thrust value for each arm
    Thrust_unbias = Thrust_filt - Thrust_0;
end

end