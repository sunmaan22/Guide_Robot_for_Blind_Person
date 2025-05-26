#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32
import serial
import collections
import statistics

class GPSPublisher:
    def __init__(self):
        rospy.init_node('gps_publisher', anonymous=True)
        
        # 필요한 퍼블리셔만 유지
        self.fix_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
        self.speed_pub = rospy.Publisher('/gps/speed', Float32, queue_size=10)
        self.course_pub = rospy.Publisher('/gps/course', Float32, queue_size=10)
        
        # 시리얼 포트 설정
        self.port = "/dev/ttyACM0"
        self.baud = 9600
        self.timeout = 1
        
        # 정확도 향상을 위한 필터링 설정
        self.filter_window_size = 5  # 이동평균 윈도우 크기
        self.lat_buffer = collections.deque(maxlen=self.filter_window_size)
        self.lon_buffer = collections.deque(maxlen=self.filter_window_size)
        self.speed_buffer = collections.deque(maxlen=self.filter_window_size)
        self.course_buffer = collections.deque(maxlen=self.filter_window_size)
        
        # NEO-M8N 모듈 특성에 맞는 임계값 설정
        self.max_position_jump = 0.0005  # 위도/경도 최대 변화량 (약 50m) - NEO-M8N 정확도 고려
        self.max_speed_change = 3.0      # 속도 최대 변화량 (m/s) - 차량/보행자 속도 고려
        
        # 이전 값 저장
        self.prev_lat = None
        self.prev_lon = None
        self.prev_speed = None
        
        # NEO-M8N에 최적화된 신호 품질 요구사항
        self.min_satellites = 5   # 최소 위성 수 (NEO-M8N은 보통 6-12개 수신)
        self.min_hdop = 2.5       # 최대 HDOP 값 (NEO-M8N 기준으로 조정)
        
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            rospy.loginfo("Serial port opened: %s", self.port)
        except serial.SerialException:
            rospy.logerr("Unable to open serial port.")
            raise
    
    def parse_gnrmc(self, line):
        """GNRMC 데이터 파싱 (개선된 오류 처리 포함)"""
        try:
            parts = line.split(',')
            if len(parts) < 12:
                return None, None, None, None, False
            
            # 데이터 유효성 확인 (A = Active, V = Void)
            if parts[2] != 'A':
                return None, None, None, None, False
            
            # 필수 데이터 존재 확인
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

            # 남반구/서반구 처리
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
            
            # 위성 수
            satellites = int(parts[7]) if parts[7] else 0
            
            # HDOP (Horizontal Dilution of Precision)
            hdop = float(parts[8]) if parts[8] else 99.0
            
            return satellites, hdop
            
        except (ValueError, IndexError):
            return None, None
    
    def is_outlier(self, lat, lon, speed):
        """이상치 검출"""
        if self.prev_lat is None or self.prev_lon is None:
            return False
        
        # 위치 변화량 확인
        lat_diff = abs(lat - self.prev_lat)
        lon_diff = abs(lon - self.prev_lon)
        
        if lat_diff > self.max_position_jump or lon_diff > self.max_position_jump:
            rospy.logwarn(f"Position jump detected: lat_diff={lat_diff:.6f}, lon_diff={lon_diff:.6f}")
            return True
        
        # 속도 변화량 확인
        if self.prev_speed is not None:
            speed_diff = abs(speed - self.prev_speed)
            if speed_diff > self.max_speed_change:
                rospy.logwarn(f"Speed jump detected: speed_diff={speed_diff:.2f}")
                return True
        
        return False
    
    def apply_smoothing_filter(self, lat, lon, speed, course):
        """이동평균 필터 적용"""
        # 버퍼에 새 데이터 추가
        self.lat_buffer.append(lat)
        self.lon_buffer.append(lon)
        self.speed_buffer.append(speed)
        self.course_buffer.append(course)
        
        # 충분한 데이터가 쌓이면 이동평균 계산
        if len(self.lat_buffer) >= 3:
            filtered_lat = statistics.median(self.lat_buffer)
            filtered_lon = statistics.median(self.lon_buffer)
            filtered_speed = statistics.mean(self.speed_buffer)
            
            # 방위각은 원형 데이터이므로 특별 처리
            if len(self.course_buffer) >= 3:
                # 간단한 원형 평균 (더 정확한 방법도 있지만 여기서는 단순화)
                filtered_course = statistics.median(self.course_buffer)
            else:
                filtered_course = course
                
            return filtered_lat, filtered_lon, filtered_speed, filtered_course
        else:
            return lat, lon, speed, course
    
    def run(self):
        """메인 루프"""
        rate = rospy.Rate(10)  # 10Hz로 증가 (더 많은 데이터 수집)
        satellites = 0
        hdop = 99.0
        
        while not rospy.is_shutdown():
            if self.ser.in_waiting:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                
                # GPGGA 메시지에서 신호 품질 정보 추출
                if line.startswith('$GPGGA'):
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
                
                # 신호 품질 확인 (NEO-M8N 특성 고려)
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
                
                # NavSatFix 퍼블리시
                fix_msg = NavSatFix()
                fix_msg.header.stamp = rospy.Time.now()
                fix_msg.header.frame_id = "gps"
                fix_msg.status.status = NavSatStatus.STATUS_FIX
                fix_msg.status.service = NavSatStatus.SERVICE_GPS
                fix_msg.latitude = filtered_lat
                fix_msg.longitude = filtered_lon
                fix_msg.altitude = 0.0
                
                # NEO-M8N 기반 정확도 정보 설정
                # NEO-M8N: 일반적으로 2.5m CEP (50% 확률로 2.5m 내), 99% 확률로 5-10m 내
                base_accuracy = 2.5  # 미터 단위
                hdop_factor = max(1.0, hdop / 1.0)  # HDOP 1.0 기준으로 스케일링
                covariance_value = (base_accuracy * hdop_factor) ** 2
                
                fix_msg.position_covariance = [
                    covariance_value, 0, 0,
                    0, covariance_value, 0,
                    0, 0, covariance_value * 9  # 고도는 수평 정확도의 3배 정도로 부정확
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

                # 이전 값 업데이트
                self.prev_lat = filtered_lat
                self.prev_lon = filtered_lon
                self.prev_speed = filtered_speed

                rospy.loginfo(f"GPS: lat={filtered_lat:.7f}, lon={filtered_lon:.7f}, "
                            f"speed={filtered_speed:.2f}m/s, course={filtered_course:.1f}°, "
                            f"sats={satellites}, hdop={hdop:.2f}")

            rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher = GPSPublisher()
        gps_publisher.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in GPS publisher: {e}")