function avgPts = getTotalAverage(data,window,xMid)

halfWindow = floor(window/2);

for n = 1:1:length(xMid)
    avgPts.x(n)= xMid(n);
    for i = 1:width(data)
        avgPts.y(n,i) = mean(data(xMid(n)-halfWindow:xMid(n)+halfWindow,i));
    end
end

figure
plot(data)
hold on
plot(xMid-halfWindow,data(xMid-halfWindow,1),"k*","MarkerSize",8,"LineWidth",1)
plot(xMid+halfWindow,data(xMid+halfWindow,1),"k*","MarkerSize",8,"LineWidth",1)
end
