import math
import serial
import pynmea2


class GPS:
    """GPS for boat."""
    def __init__(self):
        # Port for serial 0.
        port = "/dev/ttyAMA0"
        
        self.ser = serial.Serial(port, baudrate=9600, timeout=0.5)

    def read_serial_gps(self):
        # GPS data.
        latitude = None
        longitude = None
        date_string = None
        time_string = None
        
        # Read GPS data.
        dataout = pynmea2.NMEAStreamReader()
        nmea_sentence_bytes = self.ser.readline()
        
        try:
            # Convert to string.
            nmea_sentence_bytes = nmea_sentence_bytes.decode('utf-8')
            
            #if '$GPRMC' in nmea_sentence_bytes:
            if '$GNRMC' in nmea_sentence_bytes:
                latitude = round(self.parse_latitude(nmea_sentence_bytes), 6)
                longitude = round(self.parse_longitude(nmea_sentence_bytes), 6)
                day, month, year = self.parse_date(nmea_sentence_bytes)
                date_string = "{}/{}/{}".format(day, month, year)
                hour, minute, second = self.parse_time(nmea_sentence_bytes)
                time_string = "{:02d}:{:02d}:{:02d}".format(hour, minute, second)
        except:
            pass

        return latitude, longitude, date_string, time_string

    def parse_latitude(self, data):
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

    def parse_longitude(self, data):
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

    def parse_time(self, data):
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

    def parse_date(self, data):
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