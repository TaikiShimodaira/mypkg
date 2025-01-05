# SPDX-FileCopyrightText: 2025 Taikii Shimodaira
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime, timedelta
import math
import random

def calculate_sunrise_sunset(lat, lon, date):
    # Constants
    zenith = 90.8333  # Official zenith for sunrise/sunset

    # Day of the year
    n = date.timetuple().tm_yday

    # Approximate time of sunrise and sunset
    lng_hour = lon / 15
    t_sunrise = n + ((6 - lng_hour) / 24)
    t_sunset = n + ((18 - lng_hour) / 24)

    # Sun's mean anomaly
    M_sunrise = (0.9856 * t_sunrise) - 3.289
    M_sunset = (0.9856 * t_sunset) - 3.289

    # Sun's true longitude
    def true_longitude(M):
        L = M + (1.916 * math.sin(math.radians(M))) + (0.020 * math.sin(math.radians(2 * M))) + 282.634
        return L % 360

    L_sunrise = true_longitude(M_sunrise)
    L_sunset = true_longitude(M_sunset)

    # Sun's right ascension
    def right_ascension(L):
        RA = math.degrees(math.atan(0.91764 * math.tan(math.radians(L))))
        RA = RA % 360
        L_quadrant = (math.floor(L / 90)) * 90
        RA_quadrant = (math.floor(RA / 90)) * 90
        RA = RA + (L_quadrant - RA_quadrant)
        return RA / 15

    RA_sunrise = right_ascension(L_sunrise)
    RA_sunset = right_ascension(L_sunset)

    # Sun's declination
    def declination(L):
        sin_dec = 0.39782 * math.sin(math.radians(L))
        cos_dec = math.cos(math.asin(sin_dec))
        return sin_dec, cos_dec

    sin_dec_sunrise, cos_dec_sunrise = declination(L_sunrise)
    sin_dec_sunset, cos_dec_sunset = declination(L_sunset)

    # Sun's local hour angle
    def local_hour_angle(sin_dec, cos_dec, lat):
        cos_h = (math.cos(math.radians(zenith)) - (sin_dec * math.sin(math.radians(lat)))) / (cos_dec * math.cos(math.radians(lat)))
        if cos_h > 1 or cos_h < -1:
            return None  # Sun does not rise/set on this day
        return math.degrees(math.acos(cos_h))

    H_sunrise = local_hour_angle(sin_dec_sunrise, cos_dec_sunrise, lat)
    H_sunset = local_hour_angle(sin_dec_sunset, cos_dec_sunset, lat)

    if H_sunrise is None or H_sunset is None:
        return None, None

    # Convert H to hours
    H_sunrise = H_sunrise / 15
    H_sunset = H_sunset / 15

    # Local mean time
    def local_mean_time(H, RA, t):
        T = H + RA - (0.06571 * t) - 6.622
        return T % 24

    T_sunrise = local_mean_time(360 - H_sunrise, RA_sunrise, t_sunrise)
    T_sunset = local_mean_time(H_sunset, RA_sunset, t_sunset)

    # Convert to UTC
    UTC_sunrise = T_sunrise - lng_hour
    UTC_sunset = T_sunset - lng_hour

    # Adjust for local time
    sunrise_time = UTC_sunrise % 24
    sunset_time = UTC_sunset % 24

    return sunrise_time, sunset_time

class SunriseSunsetInfo(Node):
    def __init__(self):
        super().__init__("sunrise_sunset_talker")
        self.pub = self.create_publisher(String, "sunrise_sunset_topic", 10)

        # List of random locations with (Name, Latitude, Longitude, Timezone Offset)
        self.locations = [
            ("Tokyo, Japan", 35.6895, 139.6917, 9),
            ("New York, USA", 40.7128, -74.0060, -5),
            ("London, UK", 51.5074, -0.1278, 0),
            ("Sydney, Australia", -33.8688, 151.2093, 11),
            ("Cape Town, South Africa", -33.9249, 18.4241, 2)
        ]
        self.create_timer(60.0, self.cb)  # Publish every 60 seconds

    def cb(self):
        now = datetime.now()

        # Choose a random location
        location = random.choice(self.locations)
        location_name, latitude, longitude, timezone_offset = location

        # Calculate sunrise and sunset times
        sunrise_time, sunset_time = calculate_sunrise_sunset(latitude, longitude, now.date())

        # Format times for output
        if sunrise_time is not None and sunset_time is not None:
            sunrise_hour = int(sunrise_time + timezone_offset) % 24
            sunrise_minute = int((sunrise_time - int(sunrise_time)) * 60)
            sunset_hour = int(sunset_time + timezone_offset) % 24
            sunset_minute = int((sunset_time - int(sunset_time)) * 60)

            message = f"Location: {location_name}\n"
            message += f"Sunrise: {sunrise_hour:02}:{sunrise_minute:02}\n"
            message += f"Sunset: {sunset_hour:02}:{sunset_minute:02}"
        else:
            message = f"Sun does not rise or set on this day for {location_name}."

        msg = String()
        msg.data = message
        self.pub.publish(msg)
        self.get_logger().info(f"Published sunrise and sunset info:\n{message}")


def main():
    rclpy.init()  # Ensure initialization
    node = SunriseSunsetInfo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

