function [error] = errorCalculate(prevLatitude, prevLongitude, currentLatitude, currentLongitude, desiredLatitude, desiredLongitude)
    [~, ~, ~, a] = convertTo2DPlane(desiredLatitude, desiredLongitude, prevLatitude, prevLongitude);
    [~, ~, ~, b] = convertTo2DPlane(desiredLatitude, desiredLongitude, currentLatitude, currentLongitude);
    [~, ~, ~, c] = convertTo2DPlane(currentLatitude, currentLongitude, prevLatitude, prevLongitude);
    
    angle = acos((c^2 + a^2 - b^2)/(2*c*a));
    error = c*sin(angle);
end
