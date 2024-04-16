import csv
import serial
import time
import string
import pynmea2


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


# Open the CSV file in 'w' mode to create a new file.
with open('Lake1.csv', 'w', newline='') as csvfile:
    # Define the fieldnames for the CSV file.
    fieldnames = ['Date', 'Time', 'Latitude', 'Longitude']

    # Create a CSV writer object.   
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    # Write the header row to the CSV file.
    writer.writeheader()

    # Port for GPS.
    port = "/dev/ttyAMA0"
    
    i = 0
    # Example of a while loop to continuously receive and parse NMEA data.
    while i < 2500:
        # Receive NMEA data.
        ser = serial.Serial(port, baudrate=9600, timeout=0.5)
        dataout = pynmea2.NMEAStreamReader()
        nmea_sentence_bytes = ser.readline()
        nmea_sentence_bytes = nmea_sentence_bytes.decode('utf-8')
        #print(nmea_sentence_bytes)
        
        if '$GNRMC' in nmea_sentence_bytes:
            try:
                # Parse latitude, longitude, time, and date.
                latitude = round(parse_latitude(nmea_sentence_bytes), 6)
                longitude = round(parse_longitude(nmea_sentence_bytes), 6)
                day, month, year = parse_date(nmea_sentence_bytes)
                date_string = "{}/{}/{}".format(day, month, year)

                hour, minute, second = parse_time(nmea_sentence_bytes)
                time_string = "{:02d}:{:02d}:{:02d}".format(hour, minute, second)

                # Write data to CSV file.
                writer.writerow({'Date': date_string, 'Time': time_string, 'Latitude': latitude, 'Longitude': longitude})

                # Print parsed data.
                print('Index = ', i, end=', ')
                print("Date:", date_string, end=', ')
                print("Time:", time_string, end=', ')
                print("Latitude:", latitude, end=', ')
                print("Longitude:" , longitude)
                
                i += 1
                
            except ValueError as e:
                print("Error:", e)

