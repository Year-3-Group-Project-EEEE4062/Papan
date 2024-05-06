function [x, y, bearing, distance] = convertTo2DPlane(lat2, lon2, reference_lat, reference_lon)
    % Earth radius in meters.
    EARTH_RADIUS = 6371000.0;

    % Convert degrees to radians.
    reference_lat = deg2rad(reference_lat);
    reference_lon = deg2rad(reference_lon);
    lat2 = deg2rad(lat2);
    lon2 = deg2rad(lon2);

    % Calculate differences in coordinates.
    deltaLat = lat2 - reference_lat;
    deltaLon = lon2 - reference_lon;

    % Calculate distance between the two points.
    a = sin(deltaLat / 2)^2 + cos(reference_lat) * cos(lat2) * sin(deltaLon / 2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1 - a));
    distance = EARTH_RADIUS * c;

    % Calculate bearing.
    angle = atan2(sin(deltaLon) * cos(lat2), cos(reference_lat) * sin(lat2) - sin(reference_lat) * cos(lat2) * cos(deltaLon));
    bearing = rad2deg(angle);

    % Convert distance and bearing to x-y coordinates.
    x = distance * cos(angle);
    y = distance * sin(angle);
end