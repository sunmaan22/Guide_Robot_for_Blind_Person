#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial

def parse_gpgga(line):
    try:
        parts = line.split(',')
        if len(parts) >= 6 and parts[2] and parts[4]:
            lat_raw = float(parts[2])
            lon_raw = float(parts[4])

            lat_deg = int(lat_raw / 100)
            lat_min = lat_raw - (lat_deg * 100)
            latitude = lat_deg + (lat_min / 60)

            lon_deg = int(lon_raw / 100)
            lon_min = lon_raw - (lon_deg * 100)
            longitude = lon_deg + (lon_min / 60)

            if parts[3] == 'S':
                latitude = -latitude
            if parts[5] == 'W':
                longitude = -longitude

            return latitude, longitude
    except Exception as e:
        rospy.logwarn(f"Failed to parse GPGGA: {e}")
    return None, None

def gps_node():
    rospy.init_node('gps_publisher', anonymous=True)
    pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)

    port = "/dev/ttyAMA0"
    baud = 9600
    timeout = 1

    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        rospy.loginfo("Serial port opened: %s", port)
    except serial.SerialException:
        rospy.logerr("Unable to open serial port.")
        return

    rate = rospy.Rate(5)  # 5Hz

    while not rospy.is_shutdown():
        if ser.in_waiting:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if not line or not line.startswith('$GPGGA'):
                continue

            lat, lon = parse_gpgga(line)
            if lat is not None and lon is not None:
                msg = NavSatFix()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "gps"
                msg.status.status = NavSatStatus.STATUS_FIX
                msg.status.service = NavSatStatus.SERVICE_GPS
                msg.latitude = lat
                msg.longitude = lon
                msg.altitude = 0.0  # GPGGA에 고도도 있음, 필요시 추가
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                pub.publish(msg)
                rospy.loginfo(f"Published: lat={lat:.7f}, lon={lon:.7f}")

        rate.sleep()

if __name__ == '__main__':
    try:
        gps_node()
    except rospy.ROSInterruptException:
        pass
