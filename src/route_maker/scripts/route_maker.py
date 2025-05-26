#!/usr/bin/env python3
import rospy
import requests
import threading
import math
import tf2_ros
import numpy as np
from pyproj import Transformer
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String, Float32

KAKAO_REST_API_KEY = "e68a76ce44a72c8327a4835ef38b42af"

route_coords = []
has_route = False
origin_x, origin_y = None, None
transformer = Transformer.from_crs("epsg:4326", "epsg:32652", always_xy=True)
latest_lat, latest_lon = None, None
current_route_index = 0
off_route_count = 0
last_instruction_point = -1
current_destination = None
rerouting_in_progress = False
reroute_threshold = 15.0  # 동적으로 조정될 예정
MAX_OFF_ROUTE_COUNT = 3  # 5에서 3으로 줄임 (고품질 GPS로 더 빠른 반응)

# GPS 품질 관련 변수들
current_gps_accuracy = 2.5  # 기본값 (NEO-M8N 기준)
current_speed = 0.0
gps_quality_good = True

class GPS2UTM:
    def __init__(self):
        self.utm_x = None
        self.utm_y = None
        self.pose_publisher = rospy.Publisher("/gps/pose", PoseStamped, queue_size=1)
        rospy.Subscriber("/gps/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/gps/speed", Float32, self.speed_callback)

    def speed_callback(self, msg):
        """GPS 속도 정보 수신"""
        global current_speed
        current_speed = msg.data
        rospy.loginfo_throttle(5.0, f"현재 속도: {current_speed:.2f} m/s ({current_speed * 3.6:.1f} km/h)")

    def gps_callback(self, msg):
        global origin_x, origin_y, latest_lat, latest_lon, current_route_index, off_route_count
        global current_gps_accuracy, gps_quality_good, reroute_threshold
        
        # GPS 품질 확인 및 정확도 추출
        if hasattr(msg, 'position_covariance') and len(msg.position_covariance) > 0:
            current_gps_accuracy = math.sqrt(msg.position_covariance[0]) if msg.position_covariance[0] > 0 else 2.5
            
            # GPS 품질이 너무 나쁘면 데이터 무시
            if current_gps_accuracy > 15.0:  # 15m 이상 오차면 무시
                gps_quality_good = False
                rospy.logwarn_throttle(2.0, f"GPS 정확도 저하 ({current_gps_accuracy:.1f}m), 데이터 무시")
                return
            elif current_gps_accuracy > 8.0:  # 8m 이상이면 경고
                gps_quality_good = False
                rospy.logwarn_throttle(5.0, f"GPS 정확도 주의 ({current_gps_accuracy:.1f}m)")
            else:
                gps_quality_good = True
            
            # 동적 경로 이탈 임계값 조정
            reroute_threshold = max(12.0, current_gps_accuracy * 2.0)  # 최소 12m, GPS 정확도의 2배
            
            rospy.loginfo_throttle(10.0, f"GPS 정확도: {current_gps_accuracy:.1f}m, 경로이탈 임계값: {reroute_threshold:.1f}m")

        self.utm_x, self.utm_y = transformer.transform(msg.longitude, msg.latitude)
        latest_lat = msg.latitude
        latest_lon = msg.longitude

        if origin_x is None or origin_y is None:
            origin_x = self.utm_x
            origin_y = self.utm_y
            rospy.loginfo(f"[🧭 기준점 설정] origin_x = {origin_x:.2f}, origin_y = {origin_y:.2f}")

        # Publish current pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.utm_x - origin_x
        pose_msg.pose.position.y = self.utm_y - origin_y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self.pose_publisher.publish(pose_msg)

        # GPS 품질이 좋을 때만 경로 진행 상황 확인
        if has_route and len(route_coords) > 0 and gps_quality_good:
            self.check_route_progress()

        rospy.loginfo_throttle(1.0, f"현재 UTM 위치: ({self.utm_x:.2f}, {self.utm_y:.2f})")

    def check_route_progress(self):
        global current_route_index, off_route_count, rerouting_in_progress
        
        if rerouting_in_progress or not route_coords or current_route_index >= len(route_coords):
            return
            
        current_utm_x, current_utm_y = self.utm_x, self.utm_y
        
        # 경로상에서 가장 가까운 지점 찾기 (검색 범위 확대)
        min_dist = float('inf')
        best_idx = current_route_index
        
        # GPS 정확도에 따라 검색 윈도우 조정
        base_window = 30
        accuracy_factor = max(1.0, current_gps_accuracy / 5.0)  # 정확도가 나쁘면 더 넓게 검색
        search_window = min(int(base_window * accuracy_factor), len(route_coords) - current_route_index)
        
        for i in range(current_route_index, current_route_index + search_window):
            if i >= len(route_coords):
                break
                
            lon, lat = route_coords[i]
            route_utm_x, route_utm_y = latlon_to_utm(lat, lon)
            dist = math.hypot(route_utm_x - current_utm_x, route_utm_y - current_utm_y)
            
            if dist < min_dist:
                min_dist = dist
                best_idx = i
        
        # 경로 이탈 감지 (동적 임계값 사용)
        if min_dist > reroute_threshold:
            off_route_count += 1
            rospy.logwarn(f"[경로이탈 감지] 현재 위치가 경로에서 {min_dist:.2f}m 떨어져 있습니다. ({off_route_count}/{MAX_OFF_ROUTE_COUNT})")
            
            if off_route_count >= MAX_OFF_ROUTE_COUNT:
                rospy.logwarn(f"경로 재탐색을 시작합니다.")
                self.trigger_reroute()
                off_route_count = 0
        else:
            off_route_count = 0
            
        # 경로 인덱스 업데이트
        if best_idx > current_route_index:
            current_route_index = best_idx
            check_navigation_instructions(current_route_index)

    def trigger_reroute(self):
        """현재 위치에서 목적지까지 경로를 재탐색합니다."""
        global rerouting_in_progress, current_destination
        
        if current_destination and not rerouting_in_progress:
            rerouting_in_progress = True
            instruction_pub.publish("경로를 이탈하였습니다. 경로를 재탐색합니다.")
            
            threading.Thread(target=recalculate_route, args=(current_destination,), daemon=True).start()

    def get_utm(self):
        return self.utm_x, self.utm_y

def recalculate_route(destination):
    """현재 위치에서 목적지까지 경로를 다시 계산합니다."""
    global rerouting_in_progress, current_route_index, last_instruction_point
    
    try:
        rospy.loginfo(f"[경로 재탐색] 목적지: '{destination}'")
        
        if latest_lat is None or latest_lon is None:
            rospy.logwarn("[GPS 오류] 현재 GPS 위치를 알 수 없습니다.")
            rerouting_in_progress = False
            return
        
        # GPS 품질이 너무 나쁘면 재탐색 지연
        if not gps_quality_good:
            rospy.logwarn("[GPS 품질] GPS 신호가 불안정하여 재탐색을 잠시 지연합니다.")
            rospy.sleep(3.0)  # 3초 대기
        
        # 목적지 좌표 확인
        if hasattr(recalculate_route, 'destination_coords') and recalculate_route.destination_coords.get(destination):
            end_lat, end_lon = recalculate_route.destination_coords[destination]
            rospy.loginfo(f"[저장된 목적지 사용] 좌표: ({end_lat}, {end_lon})")
        else:
            end_lat, end_lon = kakao_search_place(destination, latest_lat, latest_lon, KAKAO_REST_API_KEY)
            
            if end_lat is None or end_lon is None:
                rospy.logerr(f"[목적지 검색 실패] '{destination}'을(를) 찾을 수 없습니다.")
                rerouting_in_progress = False
                instruction_pub.publish("목적지를 찾을 수 없습니다. 경로 재탐색에 실패했습니다.")
                return
            
            if not hasattr(recalculate_route, 'destination_coords'):
                recalculate_route.destination_coords = {}
            recalculate_route.destination_coords[destination] = (end_lat, end_lon)
        
        plan_navigation_route(latest_lat, latest_lon, end_lat, end_lon, destination)
        
        current_route_index = 0
        last_instruction_point = -1
        
        instruction_pub.publish("경로 재탐색이 완료되었습니다.")
        rospy.loginfo("[경로 재탐색 완료]")
    except Exception as e:
        rospy.logerr(f"[경로 재탐색 예외] {e}")
        instruction_pub.publish("경로 재탐색 중 오류가 발생했습니다.")
    finally:
        rerouting_in_progress = False

def latlon_to_utm(lat, lon):
    x, y = transformer.transform(lon, lat)
    return x, y

def interpolate_coords(coords, resolution=0.5):
    if not coords or len(coords) < 2:
        return coords
    interpolated = []
    for i in range(len(coords) - 1):
        x1, y1 = transformer.transform(coords[i][0], coords[i][1])
        x2, y2 = transformer.transform(coords[i + 1][0], coords[i + 1][1])
        dist = math.hypot(x2 - x1, y2 - y1)
        steps = max(int(dist / resolution), 1)
        for j in range(steps):
            t = j / steps
            lon = coords[i][0] * (1 - t) + coords[i + 1][0] * t
            lat = coords[i][1] * (1 - t) + coords[i + 1][1] * t
            interpolated.append((lon, lat))
    interpolated.append(coords[-1])
    return interpolated

def publish_static_transform():
    while origin_x is None or origin_y is None:
        rospy.logwarn_throttle(1.0, "[TF 대기] origin_x, origin_y 초기화 대기 중...")
        rospy.sleep(0.1)

    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transform = TransformStamped()
    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = "map"
    static_transform.child_frame_id = "base_link"
    static_transform.transform.translation.x = -origin_x
    static_transform.transform.translation.y = -origin_y
    static_transform.transform.translation.z = 0.0
    static_transform.transform.rotation.w = 1.0

    for _ in range(10):
        static_broadcaster.sendTransform(static_transform)
        rospy.sleep(0.1)

    rospy.loginfo(f"[TF] map → base_link 등록 완료: 기준점=({origin_x:.2f}, {origin_y:.2f})")

def kakao_search_place(keyword, lat, lon, api_key):
    url = "https://dapi.kakao.com/v2/local/search/keyword.json"
    headers = {"Authorization": f"KakaoAK {api_key}"}
    params = {"query": keyword, "x": lon, "y": lat, "radius": 5000, "size": 1}
    
    rospy.loginfo(f"[카카오맵 검색] 키워드: '{keyword}', 위치: ({lat}, {lon})")
    
    try:
        res = requests.get(url, headers=headers, params=params, timeout=10)  # 타임아웃 추가
        if res.status_code == 200:
            result_json = res.json()
            if result_json["documents"]:
                doc = result_json["documents"][0]
                place_name = doc["place_name"]
                address = doc["address_name"]
                category = doc["category_name"]
                result_lat = float(doc["y"])
                result_lon = float(doc["x"])
                
                rospy.loginfo(f"[카카오맵 결과] 장소명: '{place_name}'")
                rospy.loginfo(f"[카카오맵 결과] 주소: {address}")
                rospy.loginfo(f"[카카오맵 결과] 카테고리: {category}")
                rospy.loginfo(f"[카카오맵 결과] 좌표: ({result_lat}, {result_lon})")
                
                return result_lat, result_lon
            else:
                rospy.logwarn(f"[카카오맵 검색] '{keyword}'에 대한 검색 결과가 없습니다.")
        else:
            rospy.logwarn(f"[카카오맵 API] 상태 코드: {res.status_code}, 응답: {res.text}")
    except requests.exceptions.Timeout:
        rospy.logwarn(f"[카카오맵 API] 요청 시간 초과")
    except Exception as e:
        rospy.logwarn(f"[카카오맵 API 예외] {e}")
    
    return None, None

def find_nearest_road_point(lat, lon):
    """현재 위치에서 가장 가까운 도로 네트워크 지점을 찾습니다."""
    url = f"http://router.project-osrm.org/nearest/v1/walking/{lon},{lat}"
    params = {"number": 1}
    
    try:
        res = requests.get(url, params=params, timeout=10)  # 타임아웃 추가
        if res.status_code == 200:
            data = res.json()
            if data["code"] == "Ok" and len(data["waypoints"]) > 0:
                waypoint = data["waypoints"][0]
                nearest_lon, nearest_lat = waypoint["location"]
                distance = waypoint["distance"]
                
                rospy.loginfo(f"[도로 네트워크] 가장 가까운 도로까지 {distance:.2f}m")
                rospy.loginfo(f"[도로 네트워크] 좌표: ({nearest_lat}, {nearest_lon})")
                
                return nearest_lat, nearest_lon, distance
    except requests.exceptions.Timeout:
        rospy.logwarn(f"[OSRM Nearest] 요청 시간 초과")
    except Exception as e:
        rospy.logwarn(f"[OSRM Nearest 요청 실패] {e}")
    
    return None, None, None

def create_straight_path(start_lat, start_lon, end_lat, end_lon, points=10):
    """두 지점 사이의 직선 경로를 생성합니다."""
    path = []
    for i in range(points + 1):
        t = i / points
        lon = start_lon * (1 - t) + end_lon * t
        lat = start_lat * (1 - t) + end_lat * t
        path.append((lon, lat))
    return path

def get_osrm_route(start_lat, start_lon, end_lat, end_lon):
    url = f"http://router.project-osrm.org/route/v1/walking/{start_lon},{start_lat};{end_lon},{end_lat}"
    params = {"overview": "full", "geometries": "geojson", "steps": "true"}
    try:
        res = requests.get(url, params=params, timeout=15)  # 타임아웃 추가
        if res.status_code == 200:
            route_data = res.json()['routes'][0]
            
            steps = route_data['legs'][0]['steps']
            instructions = []
            
            for i, step in enumerate(steps):
                maneuver = step.get('maneuver', {})
                instruction_type = maneuver.get('type', '')
                instruction_modifier = maneuver.get('modifier', '')
                instruction_location = maneuver.get('location', [])
                
                if instruction_type and instruction_type != 'depart' and instruction_type != 'arrive':
                    if instruction_location:
                        instructions.append({
                            'index': i,
                            'type': instruction_type,
                            'modifier': instruction_modifier,
                            'location': instruction_location,
                            'distance': step.get('distance', 0)
                        })
            
            return route_data['geometry']['coordinates'], instructions
    except requests.exceptions.Timeout:
        rospy.logwarn(f"[OSRM Route] 요청 시간 초과")
    except Exception as e:
        rospy.logwarn(f"[OSRM 요청 실패] {e}")
    return [], []

def publish_path():
    global route_coords
    pub = rospy.Publisher("/gps/route", Path, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if has_route and route_coords and origin_x is not None:
            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = "map"
            for lon, lat in route_coords:
                x, y = latlon_to_utm(lat, lon)
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"
                pose.pose.position.x = x - origin_x
                pose.pose.position.y = y - origin_y
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            pub.publish(path_msg)
            rospy.loginfo_throttle(1.0, f"[GPS 경로 퍼블리시] 총 {len(path_msg.poses)} 지점")
        rate.sleep()

instruction_pub = None
distance_pub = None  # 다음 waypoint 거리 퍼블리셔
route_instructions = []
instruction_points = []

def initialize_instruction_publisher():
    global instruction_pub
    instruction_pub = rospy.Publisher("/gps/instruction", String, queue_size=10)
    # 다음 waypoint까지 거리 정보를 별도 토픽으로 퍼블리시
    global distance_pub
    distance_pub = rospy.Publisher("/gps/next_waypoint_distance", Float32, queue_size=10)

def process_navigation_instructions(instructions):
    global route_instructions, instruction_points, route_coords
    
    route_instructions = []
    instruction_points = []
    
    if not instructions:
        return
    
    for instruction in instructions:
        instr_type = instruction['type']
        modifier = instruction.get('modifier', '')
        
        if 'location' in instruction:
            instr_lon, instr_lat = instruction['location']
            closest_idx = -1
            min_dist = float('inf')
            
            for i, (lon, lat) in enumerate(route_coords):
                dist = math.hypot(lon - instr_lon, lat - instr_lat)
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
            
            if closest_idx >= 0:
                if instr_type == 'depart' and modifier == 'straight':
                    instruction_text = f"도로까지 {instruction['distance']:.0f}미터를 직진하세요"
                else:
                    instruction_text = generate_instruction_text(instr_type, modifier)
                
                route_instructions.append({
                    'index': closest_idx,
                    'text': instruction_text,
                    'announced_preview': False,
                    'announced_action': False,
                    'type': instr_type
                })
                instruction_points.append(closest_idx)
    
    rospy.loginfo(f"[안내 지시사항] {len(route_instructions)}개 생성 완료")

def generate_instruction_text(instr_type, modifier):
    if instr_type == "turn":
        if modifier == "left":
            return "좌회전하세요"
        elif modifier == "right":
            return "우회전하세요"
        elif modifier == "slight left":
            return "약간 좌회전하세요"
        elif modifier == "slight right":
            return "약간 우회전하세요"
        elif modifier == "sharp left":
            return "급좌회전하세요"
        elif modifier == "sharp right":
            return "급우회전하세요"
    elif instr_type == "roundabout":
        return "로터리에서 진행하세요"
    elif instr_type == "merge":
        return "합류하세요"
    elif instr_type == "fork":
        if modifier == "left":
            return "왼쪽 갈림길로 가세요"
        elif modifier == "right":
            return "오른쪽 갈림길로 가세요"
        else:
            return "갈림길에서 진행하세요"
    
    return "직진하세요"

def check_navigation_instructions(current_index):
    """속도 기반 적응형 안내 시스템 (거리 정보 포함)"""
    global instruction_pub, distance_pub, route_instructions, last_instruction_point, latest_lat, latest_lon, route_coords
    
    if not instruction_pub or not route_instructions or current_index >= len(route_coords):
        return
    
    current_utm_x, current_utm_y = latlon_to_utm(latest_lat, latest_lon)
    
    # 속도 기반 안내 거리 계산
    walking_speed = max(current_speed, 1.0)  # 최소 1m/s로 가정
    preview_distance = max(15.0, walking_speed * 4)  # 최소 15m, 4초 전 미리 안내
    action_distance = max(5.0, walking_speed * 1.5)   # 최소 5m, 1.5초 전 실행 안내
    
    # GPS 정확도에 따른 안내 거리 조정
    accuracy_factor = max(1.0, current_gps_accuracy / 5.0)
    preview_distance *= accuracy_factor
    action_distance *= accuracy_factor
    
    # 가장 가까운 다음 waypoint 찾기 및 거리 퍼블리시
    next_waypoint_distance = float('inf')
    for i, instruction in enumerate(route_instructions):
        instruction_idx = instruction['index']
        
        if instruction['announced_action']:
            continue
            
        if instruction_idx >= current_index and instruction_idx < len(route_coords):
            instr_lon, instr_lat = route_coords[instruction_idx]
            instr_utm_x, instr_utm_y = latlon_to_utm(instr_lat, instr_lon)
            dist_to_instr = math.hypot(instr_utm_x - current_utm_x, instr_utm_y - current_utm_y)
            
            if dist_to_instr < next_waypoint_distance:
                next_waypoint_distance = dist_to_instr
    
    # 다음 waypoint까지 거리 퍼블리시 (별도 토픽)
    if next_waypoint_distance != float('inf') and distance_pub:
        distance_msg = Float32()
        distance_msg.data = next_waypoint_distance
        distance_pub.publish(distance_msg)
    
    for i, instruction in enumerate(route_instructions):
        instruction_idx = instruction['index']
        
        if instruction['announced_action']:
            continue
        
        if instruction_idx < current_index - 10:  # 너무 뒤에 있는 지시사항 제외
            continue
        
        if instruction_idx < len(route_coords):
            instr_lon, instr_lat = route_coords[instruction_idx]
            instr_utm_x, instr_utm_y = latlon_to_utm(instr_lat, instr_lon)
            dist_to_instr = math.hypot(instr_utm_x - current_utm_x, instr_utm_y - current_utm_y)
            
            # 시작 안내 (도로까지 직진) - 거리 포함
            if instruction.get('type') == 'depart' and not instruction['announced_action'] and i == 0:
                message_with_distance = f"{instruction['text']} (거리: {int(dist_to_instr)}미터)"
                instruction_pub.publish(message_with_distance)
                route_instructions[i]['announced_action'] = True
                last_instruction_point = instruction_idx
                rospy.loginfo(f"[시작 안내] {message_with_distance}")
                continue
            
            # 미리 안내 - 거리 포함
            if not instruction['announced_preview'] and preview_distance - 5 <= dist_to_instr <= preview_distance + 5:
                preview_msg = f"{int(dist_to_instr)}미터 앞에서 {instruction['text']}"
                instruction_pub.publish(preview_msg)
                route_instructions[i]['announced_preview'] = True
                rospy.loginfo(f"[안내 미리알림] {preview_msg}")
            
            # 실행 안내 - 현재 거리 포함
            elif not instruction['announced_action'] and dist_to_instr <= action_distance:
                action_msg = f"{instruction['text']} (현재 {int(dist_to_instr)}미터 전방)"
                instruction_pub.publish(action_msg)
                route_instructions[i]['announced_action'] = True
                last_instruction_point = instruction_idx
                rospy.loginfo(f"[안내 지시사항] {action_msg}")

def plan_navigation_route(start_lat, start_lon, end_lat, end_lon, destination_name):
    """개선된 경로 계획 함수"""
    global route_coords, has_route, current_route_index, last_instruction_point, route_instructions
    
    # GPS 품질 확인 후 경로 계획
    if not gps_quality_good:
        rospy.logwarn("[GPS 품질] GPS 신호가 불안정합니다. 경로 계획을 진행하지만 주의가 필요합니다.")
        instruction_pub.publish("GPS 신호가 불안정합니다. 경로 안내가 부정확할 수 있습니다.")
    
    nearest_start_lat, nearest_start_lon, distance_to_road = find_nearest_road_point(start_lat, start_lon)
    nearest_end_lat, nearest_end_lon, _ = find_nearest_road_point(end_lat, end_lon)
    
    complete_route = []
    
    # 도로까지의 거리 임계값을 GPS 정확도에 따라 조정
    road_threshold = max(10.0, current_gps_accuracy * 2)
    
    if nearest_start_lat and distance_to_road > road_threshold:
        instruction_pub.publish(f"도로까지 {distance_to_road:.0f}미터를 직진하세요")
        rospy.loginfo(f"[도로까지 안내] {distance_to_road:.2f}m 직진")
        
        straight_path = create_straight_path(
            start_lat, start_lon, nearest_start_lat, nearest_start_lon, 
            points=max(20, int(distance_to_road/2))
        )
        complete_route.extend(straight_path)
        
        routing_start_lat, routing_start_lon = nearest_start_lat, nearest_start_lon
    else:
        routing_start_lat, routing_start_lon = start_lat, start_lon
    
    if nearest_end_lat and nearest_end_lon:
        routing_end_lat, routing_end_lon = nearest_end_lat, nearest_end_lon
    else:
        routing_end_lat, routing_end_lon = end_lat, end_lon
    
    rospy.loginfo(f"[경로 요청] 출발: ({routing_start_lat}, {routing_start_lon}) → 도착: ({routing_end_lat}, {routing_end_lon})")
    osrm_route_coords, instructions = get_osrm_route(routing_start_lat, routing_start_lon, routing_end_lat, routing_end_lon)
    
    if osrm_route_coords:
        complete_route.extend(osrm_route_coords)
        
        if nearest_end_lat and nearest_end_lon and (nearest_end_lat != end_lat or nearest_end_lon != end_lon):
            straight_path_to_dest = create_straight_path(
                nearest_end_lat, nearest_end_lon, end_lat, end_lon, points=5
            )
            complete_route.extend(straight_path_to_dest)
        
        # GPS 정확도에 따른 경로 보간 해상도 조정
        resolution = max(0.5, current_gps_accuracy / 3.0)  # 정확도가 나쁘면 더 성긴 보간
        route_coords = interpolate_coords(complete_route, resolution=resolution)
        
        if nearest_start_lat and distance_to_road > road_threshold:
            pre_instruction = {
                'index': 0,
                'type': 'depart',
                'modifier': 'straight',
                'location': [start_lon, start_lat],
                'distance': distance_to_road
            }
            instructions = [pre_instruction] + instructions
        
        process_navigation_instructions(instructions)
        has_route = True
        rospy.loginfo(f"[경로 설정 완료] 총 {len(route_coords)} 지점")
        return True
    else:
        rospy.logerr("[경로 실패] OSRM에서 경로를 가져올 수 없습니다.")
        return False

def calculate_remaining_distance():
    """목적지까지 남은 거리 계산"""
    global route_coords, current_route_index, latest_lat, latest_lon
    
    if not route_coords or current_route_index >= len(route_coords) or not latest_lat or not latest_lon:
        return 0.0
    
    remaining_dist = 0.0
    current_utm_x, current_utm_y = latlon_to_utm(latest_lat, latest_lon)
    
    # 현재 위치에서 다음 경로점까지의 거리
    if current_route_index < len(route_coords):
        next_lon, next_lat = route_coords[current_route_index]
        next_utm_x, next_utm_y = latlon_to_utm(next_lat, next_lon)
        remaining_dist += math.hypot(next_utm_x - current_utm_x, next_utm_y - current_utm_y)
    
    # 나머지 경로점들 사이의 거리
    for i in range(current_route_index, len(route_coords) - 1):
        if i + 1 < len(route_coords):
            lon1, lat1 = route_coords[i]
            lon2, lat2 = route_coords[i + 1]
            utm_x1, utm_y1 = latlon_to_utm(lat1, lon1)
            utm_x2, utm_y2 = latlon_to_utm(lat2, lon2)
            remaining_dist += math.hypot(utm_x2 - utm_x1, utm_y2 - utm_y1)
    
    return remaining_dist

def provide_distance_updates():
    """주기적으로 남은 거리 안내"""
    global instruction_pub
    last_announced_distance = 0
    
    while not rospy.is_shutdown():
        if has_route and gps_quality_good:
            remaining_dist = calculate_remaining_distance()
            
            # 100m 단위로 안내 (500m, 400m, 300m, 200m, 100m)
            if remaining_dist > 100:
                distance_milestone = int(remaining_dist / 100) * 100
                if distance_milestone != last_announced_distance and distance_milestone <= 500:
                    instruction_pub.publish(f"목적지까지 약 {distance_milestone}미터 남았습니다.")
                    last_announced_distance = distance_milestone
                    rospy.loginfo(f"[거리 안내] 목적지까지 {distance_milestone}m")
            elif remaining_dist <= 20 and last_announced_distance != -1:
                instruction_pub.publish("목적지에 도착했습니다.")
                last_announced_distance = -1
                rospy.loginfo("[도착 안내] 목적지 도착")
        
        rospy.sleep(5.0)  # 5초마다 체크

def navigation_command_callback(msg):
    global route_coords, has_route, current_route_index, last_instruction_point, route_instructions, current_destination
    
    destination = msg.data.strip()
    
    # 네비게이션 중지 명령 처리
    if destination.lower() in ['stop', 'cancel', '중지', '취소']:
        has_route = False
        route_coords = []
        current_destination = None
        current_route_index = 0
        last_instruction_point = -1
        route_instructions = []
        instruction_pub.publish("네비게이션을 중지합니다.")
        rospy.loginfo("[네비게이션 중지] 사용자 요청에 의해 중지됨")
        return
    
    rospy.loginfo(f"[새 목적지 입력] '{destination}'")
    
    # GPS 품질 확인
    if not gps_quality_good:
        instruction_pub.publish("GPS 신호가 불안정합니다. 잠시 후 다시 시도해주세요.")
        rospy.logwarn("[GPS 품질] GPS 신호 불안정으로 인한 경로 설정 지연")
        return
    
    current_destination = destination
    
    if latest_lat is None or latest_lon is None:
        rospy.logwarn("[GPS 오류] 현재 GPS 위치를 알 수 없습니다.")
        instruction_pub.publish("GPS 위치를 찾을 수 없습니다. 잠시 후 다시 시도해주세요.")
        return
    
    rospy.loginfo(f"[출발 위치] 현재 GPS: ({latest_lat}, {latest_lon})")
    
    # 목적지 검색 (재시도 로직 추가)
    max_retries = 3
    for attempt in range(max_retries):
        end_lat, end_lon = kakao_search_place(destination, latest_lat, latest_lon, KAKAO_REST_API_KEY)
        
        if end_lat is not None and end_lon is not None:
            break
        
        if attempt < max_retries - 1:
            rospy.logwarn(f"[카카오맵 검색] 재시도 {attempt + 1}/{max_retries}")
            rospy.sleep(1.0)
    
    if end_lat is None or end_lon is None:
        rospy.logerr(f"[목적지 검색 실패] '{destination}'을(를) 찾을 수 없습니다.")
        instruction_pub.publish(f"'{destination}'을(를) 찾을 수 없습니다. 다른 키워드로 시도해주세요.")
        return
    
    # 목적지 좌표 저장
    if not hasattr(recalculate_route, 'destination_coords'):
        recalculate_route.destination_coords = {}
    recalculate_route.destination_coords[destination] = (end_lat, end_lon)
    
    # 경로 계획
    if plan_navigation_route(latest_lat, latest_lon, end_lat, end_lon, destination):
        distance = calculate_remaining_distance()
        if distance > 0:
            instruction_pub.publish(f"'{destination}'까지 약 {distance:.0f}미터 경로로 안내를 시작합니다.")
        else:
            instruction_pub.publish(f"'{destination}'까지의 경로 안내를 시작합니다.")
        current_route_index = 0
        last_instruction_point = -1
    else:
        instruction_pub.publish("경로를 계산할 수 없습니다. 다시 시도해주세요.")

def main():
    global route_coords, has_route
    rospy.init_node("utm_route_publisher")
    gps_converter = GPS2UTM()
    
    initialize_instruction_publisher()
    
    rospy.Subscriber("/navigation_command", String, navigation_command_callback)
    
    # GPS 기준 좌표 대기
    while latest_lat is None or latest_lon is None:
        rospy.logwarn_throttle(1.0, "[GPS 대기] GPS 초기값 수신 중...")
        rospy.sleep(0.1)
    
    # GPS 품질이 안정될 때까지 대기
    stable_count = 0
    while stable_count < 5:  # 5번 연속 안정적인 신호 필요
        if gps_quality_good:
            stable_count += 1
        else:
            stable_count = 0
        rospy.loginfo_throttle(2.0, f"[GPS 품질 확인] 안정화 대기 중... ({stable_count}/5)")
        rospy.sleep(1.0)
    
    rospy.loginfo("[GPS 품질] GPS 신호가 안정화되었습니다.")
    
    publish_static_transform()
    threading.Thread(target=publish_path, daemon=True).start()
    threading.Thread(target=provide_distance_updates, daemon=True).start()
    
    rospy.loginfo("[준비 완료] 네비게이션 명령을 기다립니다...")
    rospy.loginfo("[사용법] 목적지: rostopic pub /navigation_command std_msgs/String \"data: '강남역'\"")
    rospy.loginfo("[사용법] 중지: rostopic pub /navigation_command std_msgs/String \"data: 'stop'\"")
    
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass