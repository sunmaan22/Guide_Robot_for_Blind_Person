#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import serial
import collections
import statistics
import math

class GPSPublisher:
    def __init__(self):
        rospy.init_node('gps_publisher', anonymous=True)
        
        # 기존 퍼블리셔들
        self.fix_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
        self.speed_pub = rospy.Publisher('/gps/speed', Float32, queue_size=10)
        self.course_pub = rospy.Publisher('/gps/course', Float32, queue_size=10)
        
        # RViz 시각화를 위한 퍼블리셔들
        self.marker_pub = rospy.Publisher('/gps/markers', MarkerArray, queue_size=10)
        self.current_pos_pub = rospy.Publisher('/gps/current_position', Marker, queue_size=10)
        
        # 시리얼 포트 설정
        self.port = "/dev/ttyUSB0"
        self.baud = 9600
        self.timeout = 1
        
        # 정확도 향상을 위한 필터링 설정
        self.filter_window_size = 5
        self.lat_buffer = collections.deque(maxlen=self.filter_window_size)
        self.lon_buffer = collections.deque(maxlen=self.filter_window_size)
        self.speed_buffer = collections.deque(maxlen=self.filter_window_size)
        self.course_buffer = collections.deque(maxlen=self.filter_window_size)
        
        # NEO-M8N 모듈 특성에 맞는 임계값 설정
        self.max_position_jump = 0.0005
        self.max_speed_change = 3.0
        
        # 이전 값 저장
        self.prev_lat = None
        self.prev_lon = None
        self.prev_speed = None
        
        # NEO-M8N에 최적화된 신호 품질 요구사항
        self.min_satellites = 4
        self.min_hdop = 2.5
        
        # 시각화를 위한 추가 변수들
        self.path_points = []  # GPS 경로 저장
        self.max_path_points = 1000  # 최대 경로점 수
        self.origin_lat = None  # 원점 위도
        self.origin_lon = None  # 원점 경도
        self.marker_id = 0
        
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            rospy.loginfo("Serial port opened: %s", self.port)
        except serial.SerialException:
            rospy.logerr("Unable to open serial port.")
            raise
    
    def lat_lon_to_xy(self, lat, lon):
        """위경도를 로컬 XY 좌표로 변환 (단순 투영)"""
        if self.origin_lat is None or self.origin_lon is None:
            self.origin_lat = lat
            self.origin_lon = lon
            return 0.0, 0.0
        
        # 간단한 평면 투영 (작은 영역에서만 유효)
        # 1도 ≈ 111.32km at equator
        R = 6371000  # 지구 반지름 (미터)
        
        dlat = lat - self.origin_lat
        dlon = lon - self.origin_lon
        
        x = dlon * R * math.cos(math.radians(self.origin_lat)) * math.pi / 180
        y = dlat * R * math.pi / 180
        
        return x, y
    
    def create_path_markers(self):
        """GPS 경로를 위한 마커 배열 생성"""
        marker_array = MarkerArray()
        
        if len(self.path_points) < 2:
            return marker_array
        
        # 경로 라인 마커
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = rospy.Time.now()
        path_marker.ns = "gps_path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        
        # 라인 스타일 설정
        path_marker.scale.x = 0.5  # 라인 두께
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0  # 녹색
        path_marker.color.b = 0.0
        path_marker.color.a = 0.8
        
        # 경로점들을 마커에 추가
        for point in self.path_points:
            path_marker.points.append(point)
        
        marker_array.markers.append(path_marker)
        
        # 경로상의 점들을 작은 구로 표시
        for i, point in enumerate(self.path_points[::10]):  # 10개마다 하나씩만 표시
            point_marker = Marker()
            point_marker.header.frame_id = "map"
            point_marker.header.stamp = rospy.Time.now()
            point_marker.ns = "gps_points"
            point_marker.id = i + 1
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            
            point_marker.pose.position = point
            point_marker.pose.orientation.w = 1.0
            
            point_marker.scale.x = 1.0
            point_marker.scale.y = 1.0
            point_marker.scale.z = 1.0
            
            point_marker.color.r = 0.0
            point_marker.color.g = 0.0
            point_marker.color.b = 1.0  # 파란색
            point_marker.color.a = 0.6
            
            marker_array.markers.append(point_marker)
        
        return marker_array
    
    def create_current_position_marker(self, x, y, heading, speed):
        """현재 위치를 나타내는 마커 생성"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "current_position"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # 위치 설정
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 2.0  # 조금 위에 표시
        
        # 방향 설정 (heading을 쿼터니언으로 변환)
        heading_rad = math.radians(heading)
        marker.pose.orientation.z = math.sin(heading_rad / 2.0)
        marker.pose.orientation.w = math.cos(heading_rad / 2.0)
        
        # 크기 설정 (속도에 따라 크기 조절)
        scale = max(2.0, min(8.0, speed + 2.0))  # 속도에 따라 2~8 사이
        marker.scale.x = scale
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # 색상 설정 (속도에 따라 색상 변경)
        if speed < 1.0:  # 정지
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif speed < 5.0:  # 저속
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:  # 고속
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        marker.color.a = 1.0
        
        return marker
    
    def parse_gnrmc(self, line):
        """GNRMC 데이터 파싱 (개선된 오류 처리 포함)"""
        try:
            parts = line.split(',')
            if len(parts) < 12:
                return None, None, None, None, False
            
            if parts[2] != 'A':
                return None, None, None, None, False
            
            if not parts[3] or not parts[5]:
                return None, None, None, None, False
                
            # 위도/경도 파싱
            lat_raw = float(parts[3])
            lon_raw = float(parts[5])

            lat_deg = int(lat_raw / 100)
            lat_min = lat_raw - (lat_deg * 100)
            latitude = lat_deg + (lat_min / 60)

            lon_deg = int(lon_raw / 100)
            lon_min = lon_raw - (lon_deg * 100)
            longitude = lon_deg + (lon_min / 60)

            if parts[4] == 'S':
                latitude = -latitude
            if parts[6] == 'W':
                longitude = -longitude

            # 속도(knots → m/s)
            speed_knots = float(parts[7]) if parts[7] else 0.0
            speed_mps = speed_knots * 0.514444

            # 진행방향(방위각)
            course = float(parts[8]) if parts[8] else 0.0

            return latitude, longitude, speed_mps, course, True
            
        except (ValueError, IndexError) as e:
            rospy.logwarn(f"Failed to parse GNRMC: {e}")
            return None, None, None, None, False
    
    def parse_gpgga(self, line):
        """GPGGA 데이터에서 신호 품질 정보 추출"""
        try:
            parts = line.split(',')
            if len(parts) < 15:
                return None, None
            
            satellites = int(parts[7]) if parts[7] else 0
            hdop = float(parts[8]) if parts[8] else 99.0
            
            return satellites, hdop
            
        except (ValueError, IndexError):
            return None, None
    
    def is_outlier(self, lat, lon, speed):
        """이상치 검출"""
        if self.prev_lat is None or self.prev_lon is None:
            return False
        
        lat_diff = abs(lat - self.prev_lat)
        lon_diff = abs(lon - self.prev_lon)
        
        if lat_diff > self.max_position_jump or lon_diff > self.max_position_jump:
            rospy.logwarn(f"Position jump detected: lat_diff={lat_diff:.6f}, lon_diff={lon_diff:.6f}")
            return True
        
        if self.prev_speed is not None:
            speed_diff = abs(speed - self.prev_speed)
            if speed_diff > self.max_speed_change:
                rospy.logwarn(f"Speed jump detected: speed_diff={speed_diff:.2f}")
                return True
        
        return False
    
    def apply_smoothing_filter(self, lat, lon, speed, course):
        """이동평균 필터 적용"""
        self.lat_buffer.append(lat)
        self.lon_buffer.append(lon)
        self.speed_buffer.append(speed)
        self.course_buffer.append(course)
        
        if len(self.lat_buffer) >= 3:
            filtered_lat = statistics.median(self.lat_buffer)
            filtered_lon = statistics.median(self.lon_buffer)
            filtered_speed = statistics.mean(self.speed_buffer)
            
            if len(self.course_buffer) >= 3:
                filtered_course = statistics.median(self.course_buffer)
            else:
                filtered_course = course
                
            return filtered_lat, filtered_lon, filtered_speed, filtered_course
        else:
            return lat, lon, speed, course
    
    def run(self):
        """메인 루프"""
        rate = rospy.Rate(10)
        satellites = 0
        hdop = 99.0
        
        while not rospy.is_shutdown():
            # 한 epoch에 GPGGA/GNRMC 등 여러 문장이 들어오므로, if가 아니라 while로
            # 버퍼에 쌓인 줄을 모두 소진해야 처리 지연이 누적되지 않는다.
            while self.ser.in_waiting:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                
                # GPGGA 메시지에서 신호 품질 정보 추출
                if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                    sats, h = self.parse_gpgga(line)
                    if sats is not None:
                        satellites = sats
                        hdop = h
                    continue
                
                # GNRMC 메시지 처리
                if not line.startswith('$GNRMC'):
                    continue

                lat, lon, speed, course, valid = self.parse_gnrmc(line)
                
                if not valid or lat is None or lon is None:
                    continue
                
                # 신호 품질 확인
                if satellites < self.min_satellites:
                    rospy.logwarn(f"Insufficient satellites: {satellites} (need ≥{self.min_satellites})")
                    continue
                    
                if hdop > self.min_hdop:
                    rospy.logwarn(f"Poor HDOP: {hdop:.2f} (need ≤{self.min_hdop})")
                    continue
                
                # 이상치 검출
                if self.is_outlier(lat, lon, speed):
                    continue
                
                # 필터링 적용
                filtered_lat, filtered_lon, filtered_speed, filtered_course = \
                    self.apply_smoothing_filter(lat, lon, speed, course)
                
                # XY 좌표 변환
                x, y = self.lat_lon_to_xy(filtered_lat, filtered_lon)
                
                # 경로점 추가
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0
                self.path_points.append(point)
                
                # 경로점 수 제한
                if len(self.path_points) > self.max_path_points:
                    self.path_points.pop(0)
                
                # NavSatFix 퍼블리시
                fix_msg = NavSatFix()
                fix_msg.header.stamp = rospy.Time.now()
                fix_msg.header.frame_id = "gps"
                fix_msg.status.status = NavSatStatus.STATUS_FIX
                fix_msg.status.service = NavSatStatus.SERVICE_GPS
                fix_msg.latitude = filtered_lat
                fix_msg.longitude = filtered_lon
                fix_msg.altitude = 0.0
                
                base_accuracy = 2.5
                hdop_factor = max(1.0, hdop / 1.0)
                covariance_value = (base_accuracy * hdop_factor) ** 2
                
                fix_msg.position_covariance = [
                    covariance_value, 0, 0,
                    0, covariance_value, 0,
                    0, 0, covariance_value * 9
                ]
                fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                
                self.fix_pub.publish(fix_msg)

                # 속도 퍼블리시
                speed_msg = Float32()
                speed_msg.data = filtered_speed
                self.speed_pub.publish(speed_msg)

                # 진행 방향 퍼블리시
                course_msg = Float32()
                course_msg.data = filtered_course
                self.course_pub.publish(course_msg)

                # 시각화 마커들 퍼블리시
                if len(self.path_points) > 0:
                    # 경로 마커들
                    marker_array = self.create_path_markers()
                    self.marker_pub.publish(marker_array)
                    
                    # 현재 위치 마커
                    current_marker = self.create_current_position_marker(x, y, filtered_course, filtered_speed)
                    self.current_pos_pub.publish(current_marker)

                # 이전 값 업데이트
                self.prev_lat = filtered_lat
                self.prev_lon = filtered_lon
                self.prev_speed = filtered_speed

                rospy.loginfo(f"GPS: lat={filtered_lat:.7f}, lon={filtered_lon:.7f}, "
                            f"speed={filtered_speed:.2f}m/s, course={filtered_course:.1f}°, "
                            f"sats={satellites}, hdop={hdop:.2f}, xy=({x:.2f},{y:.2f})")

            rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher = GPSPublisher()
        gps_publisher.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in GPS publisher: {e}")