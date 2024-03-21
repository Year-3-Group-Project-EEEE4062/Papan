import math
import csv
import serial
import time
import string
import pynmea2


class GPSDataLogger:
    def __init__(self, port="/dev/ttyS0"):
        """
        Initialize the GPSDataLogger.

        Args:
            port (str): Port for GPS device.
        """
        self.port = port

    def parse_latitude(self, data):
        """
        Parse latitude from the NMEA data.

        Args:
            data (str): NMEA data string.

        Returns:
            float: Latitude in decimal degrees.
        """
        lat_data = data.split(',')[3:5]
        latitude = float(lat_data[0][0:2]) + float(lat_data[0][2:]) / 60
        if lat_data[1] == 'S':
            latitude = -latitude
        return latitude

    def parse_longitude(self, data):
        """
        Parse longitude from the NMEA data.

        Args:
            data (str): NMEA data string.

        Returns:
            float: Longitude in decimal degrees.
        """
        lon_data = data.split(',')[5:7]
        longitude = float(lon_data[0][0:3]) + float(lon_data[0][3:]) / 60
        if lon_data[1] == 'W':
            longitude = -longitude
        return longitude

    def parse_time(self, data):
        """
        Parse time from the NMEA data.

        Args:
            data (str): NMEA data string.

        Returns:
            tuple: Hour, minute, and second.
        """
        time_data = data.split(',')[1]
        hour = int(time_data[0:2])
        minute = int(time_data[2:4])
        second = int(time_data[4:6])
        return hour, minute, second

    def parse_date(self, data):
        """
        Parse date from the NMEA data.

        Args:
            data (str): NMEA data string.

        Returns:
            tuple: Day, month, and year.
        """
        date_data = data.split(',')[9]
        day = int(date_data[0:2])
        month = int(date_data[2:4])
        year = int(date_data[4:6])
        return day, month, year

    def log_gps_data(self):
        """
        Continuously receive and log GPS data.
        """
        with open(self.csv_filename, 'w', newline='') as csvfile:
            # Define the fieldnames for the CSV file.
            fieldnames = ['Date', 'Time', 'Latitude', 'Longitude']

            # Create a CSV writer object.
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            # Write the header row to the CSV file.
            writer.writeheader()

            while True:
                # Receive NMEA data.
                ser = serial.Serial(self.port, baudrate=9600, timeout=0.5)
                dataout = pynmea2.NMEAStreamReader()
                nmea_sentence_bytes = ser.readline()
                nmea_sentence_bytes = nmea_sentence_bytes.decode('utf-8')

                if '$GPRMC' in nmea_sentence_bytes:
                    try:
                        # Parse latitude, longitude, time, and date.
                        latitude = round(self.parse_latitude(nmea_sentence_bytes), 6)
                        longitude = round(self.parse_longitude(nmea_sentence_bytes), 6)
                        day, month, year = self.parse_date(nmea_sentence_bytes)
                        date_string = "{}/{}/{}".format(day, month, year)

                        hour, minute, second = self.parse_time(nmea_sentence_bytes)
                        time_string = "{:02d}:{:02d}:{:02d}".format(hour, minute, second)

                        # Write data to CSV file.
                        writer.writerow({'Date': date_string, 'Time': time_string, 'Latitude': latitude, 'Longitude': longitude})

                        # Print parsed data.
                        print("Date:", date_string, end=', ')
                        print("Time:", time_string, end=', ')
                        print("Latitude:", latitude, end=', ')
                        print("Longitude:" , longitude)
                    except ValueError as e:
                        print("Error:", e)

    def convertTo2DPlane(lat, lon, reference_lat, reference_lon):
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
