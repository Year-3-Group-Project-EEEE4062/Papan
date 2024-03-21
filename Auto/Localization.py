import math


class GPS:
    """GPS for boat."""
    def __init__(self):
        pass

    def parse_latitude(data):
        """
        Parse latitude from the NMEA data.

        Returns:
            float: Latitude in decimal degrees.
        """
        lat_data = data.split(',')[3:5]
        latitude = float(lat_data[0][0:2]) + float(lat_data[0][2:]) / 60
        if lat_data[1] == 'S':
            latitude = -latitude
        return latitude

    def parse_longitude(data):
        """
        Parse longitude from the NMEA data.

        Returns:
            float: Longitude in decimal degrees.
        """
        lon_data = data.split(',')[5:7]
        longitude = float(lon_data[0][0:3]) + float(lon_data[0][3:]) / 60
        if lon_data[1] == 'W':
            longitude = -longitude
        return longitude

    def parse_time(data):
        """
        Parse time from the NMEA data.

        Returns:
            tuple: Hour, minute, and second.
        """
        time_data = data.split(',')[1]
        hour = int(time_data[0:2])
        minute = int(time_data[2:4])
        second = int(time_data[4:6])
        return hour, minute, second

    def parse_date(data):
        """
        Parse date from the NMEA data.

        Returns:
            tuple: Day, month, and year.
        """
        date_data = data.split(',')[9]
        day = int(date_data[0:2])
        month = int(date_data[2:4])
        year = int(date_data[4:6])
        return day, month, year
    
    def convertTo2DPlane(self, lat, lon, reference_lat, reference_lon):
        """
        Converts latitude and longitude coordinates to 2D plane coordinates (x, y)
        with respect to a reference latitude and longitude.

        Parameters:
            lat (float): Latitude of the point to convert.
            lon (float): Longitude of the point to convert.
            reference_lat (float): Reference latitude.
            reference_lon (float): Reference longitude.

        Returns:
            tuple: A tuple containing the x and y coordinates in meters, distance in meters and the bearing in degrees.
        """

        # Earth radius in meters.
        EARTH_RADIUS = 6371000.0

        # Convert degrees to radians.
        reference_lat = math.radians(reference_lat)
        reference_lon = math.radians(reference_lon)
        lat = math.radians(lat)
        lon = math.radians(lon)

        # Calculate differences in coordinates.
        deltaLat = lat - reference_lat
        deltaLon = lon - reference_lon

        # Calculate distance between the two points.
        a = math.sin(deltaLat / 2)**2 + math.cos(reference_lat) * math.cos(lat) * math.sin(deltaLon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = EARTH_RADIUS * c

        # Calculate bearing.
        angle = math.atan2(math.sin(deltaLon) * math.cos(lat), math.cos(reference_lat) * math.sin(lat) - math.sin(reference_lat) * math.cos(lat) * math.cos(deltaLon))
        bearing = math.degrees(angle)

        # Convert distance and bearing to x-y coordinates.
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)

        return x, y, distance, bearing
