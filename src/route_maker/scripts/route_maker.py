#!/usr/bin/env python3
import rospy
import requests
import math
import threading
import re
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# 카카오 API 키
KAKAO_REST_API_KEY = "e68a76ce44a72c8327a4835ef38b42af"

# 전역 변수
destination_lat = None
destination_lon = None
route_coords = []
steps = []
has_route = False
off_route_counter = 0
OFF_ROUTE_DISTANCE_THRESHOLD = 0.0005
OFF_ROUTE_CONFIRM_COUNT = 3
current_step_index = 0

path_pub = None
instruction_pub = None
trigger_msg_received = False
trigger_text = ""

def extract_keyword(user_input):
    cleaned = re.sub(r"(키워드\s*말고|으로\s*안내해줘|안내해줘|가고싶어|찾아줘|알려줘|근처|목적지|가자|가줘|좀)", "", user_input)
    return cleaned.strip()

def distance(lat1, lon1, lat2, lon2):
    return math.sqrt((lat1 - lat2)**2 + (lon1 - lon2)**2) * 111139

def min_distance_to_route(current_lat, current_lon, route_coords):
    return min(distance(current_lat, current_lon, lat, lon) for lon, lat in route_coords)

def kakao_search_place(keyword, current_lat, current_lon, api_key):
    url = "https://dapi.kakao.com/v2/local/search/keyword.json"
    headers = {"Authorization": f"KakaoAK {api_key}"}
    params = {
        "query": keyword,
        "x": current_lon,
        "y": current_lat,
        "radius": 5000,
        "size": 1,
        "sort": "distance"
    }
    try:
        res = requests.get(url, headers=headers, params=params)
        if res.status_code == 200:
            result = res.json()
            if result["documents"]:
                place = result["documents"][0]
                name = place["place_name"]
                lat = float(place["y"])
                lon = float(place["x"])
                rospy.loginfo(f"'{keyword}' → {name} ({lat}, {lon})")
                return lat, lon
        else:
            rospy.logwarn(f"카카오 API 실패: {res.status_code}")
    except Exception as e:
        rospy.logwarn(f"KakaoMap API 예외 발생: {e}")
    return None, None

def get_osrm_route(start_lat, start_lon, end_lat, end_lon):
    url = f"http://router.project-osrm.org/route/v1/walking/{start_lon},{start_lat};{end_lon},{end_lat}"
    params = {"overview": "full", "geometries": "geojson", "steps": "true"}
    try:
        res = requests.get(url, params=params)
        if res.status_code == 200:
            route = res.json()['routes'][0]
            return route['geometry']['coordinates'], route['legs'][0]['steps']
    except Exception as e:
        rospy.logwarn(f"OSRM 요청 실패: {e}")
    return [], []

def translate_instruction(step):
    m = step.get('maneuver', {})
    road = step.get('name', '')
    type_ = m.get('type', '')
    modifier = m.get('modifier', '')

    dir_kor = {
        'left': '좌회전', 'right': '우회전', 'straight': '직진', 'uturn': '유턴',
        'slight left': '약간 좌회전', 'slight right': '약간 우회전',
        'sharp left': '급하게 좌회전', 'sharp right': '급하게 우회전',
    }

    if not road or road.strip() == '' or road == '(도로명 없음)':
        road = '다음 도로'

    if type_ == 'turn':
        direction = dir_kor.get(modifier, '방향 전환')
        return f"{road}에서 {direction}하세요."
    elif type_ == 'depart':
        return "길안내를 시작합니다."
    elif type_ == 'arrive':
        return "목적지에 도착했습니다."
    elif type_ == 'continue':
        return f"{road}를 따라 직진하세요."
    elif type_ == 'roundabout':
        return f"회전 교차로 진입 후, {road} 방면으로 나가세요."
    else:
        return f"{road}로 이동하세요."

def publish_path():
    global route_coords
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        if has_route and route_coords:
            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = "map"
            for lon, lat in route_coords:
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"
                pose.pose.position.x = lon
                pose.pose.position.y = lat
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            path_pub.publish(path_msg)
        rate.sleep()

def publish_instruction():
    global steps, current_step_index
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if has_route and steps:
            step0 = steps[current_step_index]
            step1 = steps[current_step_index + 1] if current_step_index + 1 < len(steps) else None
            current_lat = rospy.get_param("/current_lat", 0.0)
            current_lon = rospy.get_param("/current_lon", 0.0)

            dist0 = distance(current_lat, current_lon, step0['maneuver']['location'][1], step0['maneuver']['location'][0])
            chosen_step = step0
            min_dist = dist0

            if step1:
                dist1 = distance(current_lat, current_lon, step1['maneuver']['location'][1], step1['maneuver']['location'][0])
                if dist1 < dist0:
                    chosen_step = step1
                    min_dist = dist1

            msg = f"{min_dist:.1f}미터 남음 - {translate_instruction(chosen_step)}"
            instruction_pub.publish(String(data=msg))
            rospy.loginfo(msg)
        rate.sleep()

def gps_callback(msg):
    global has_route, route_coords, steps, off_route_counter, current_step_index
    current_lat = msg.latitude
    current_lon = msg.longitude

    rospy.set_param("/current_lat", current_lat)
    rospy.set_param("/current_lon", current_lon)

    if not has_route or not steps:
        return

    step0 = steps[current_step_index]
    lat0, lon0 = step0['maneuver']['location'][1], step0['maneuver']['location'][0]
    dist0 = distance(current_lat, current_lon, lat0, lon0)

    if dist0 < 15:
        current_step_index += 1
        if current_step_index >= len(steps):
            rospy.loginfo("목적지에 도착했습니다.")
            has_route = False
            return

    dist_from_path = min_distance_to_route(current_lat, current_lon, route_coords)
    if dist_from_path > OFF_ROUTE_DISTANCE_THRESHOLD:
        off_route_counter += 1
        rospy.logwarn(f"경로에서 {dist_from_path:.1f}미터 벗어났습니다. 경로 이탈 횟수: {off_route_counter}")
    else:
        off_route_counter = 0

    if off_route_counter >= OFF_ROUTE_CONFIRM_COUNT:
        rospy.logwarn("경로를 벗어났습니다. 경로를 재계산합니다.")
        route_coords, steps = get_osrm_route(current_lat, current_lon, destination_lat, destination_lon)
        current_step_index = 0
        off_route_counter = 0
        instruction_pub.publish(String(data="경로를 재설정했습니다. 다시 안내를 시작합니다."))

def trigger_callback(msg):
    global trigger_msg_received, trigger_text
    trigger_text = msg.data
    trigger_msg_received = True
    rospy.loginfo(f"트리거 수신: {trigger_text}")

def main():
    global destination_lat, destination_lon
    global route_coords, steps, has_route
    global path_pub, instruction_pub, current_step_index
    global trigger_msg_received, trigger_text

    rospy.init_node('kakao_osrm_route_planner')
    path_pub = rospy.Publisher("/gps/route", Path, queue_size=1)
    instruction_pub = rospy.Publisher("/gps/instruction", String, queue_size=1)

    rospy.Subscriber("/gps/fix", NavSatFix, gps_callback)
    rospy.Subscriber("/guidebot/trigger", String, trigger_callback)

    while not rospy.is_shutdown():
        try:
            gps = rospy.wait_for_message("/gps/fix", NavSatFix, timeout=5)
            current_lat = gps.latitude
            current_lon = gps.longitude
            break
        except rospy.ROSException:
            rospy.logwarn("GPS 수신 실패, 재시도 중...")

    rospy.loginfo("트리거 메시지 수신 대기 중 (/guidebot/trigger)")
    while not rospy.is_shutdown() and not trigger_msg_received:
        rospy.sleep(0.1)

    user_input = trigger_text.strip()
    keyword = extract_keyword(user_input)
    lat, lon = kakao_search_place(keyword, current_lat, current_lon, KAKAO_REST_API_KEY)
    if lat and lon:
        destination_lat = lat
        destination_lon = lon
    else:
        rospy.logerr("장소 검색에 실패했습니다.")
        return

    route_coords, steps = get_osrm_route(current_lat, current_lon, destination_lat, destination_lon)
    if route_coords:
        has_route = True
        current_step_index = 0
        threading.Thread(target=publish_path).start()
        threading.Thread(target=publish_instruction).start()
        rospy.spin()
    else:
        rospy.logerr("경로 계산에 실패했습니다.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
