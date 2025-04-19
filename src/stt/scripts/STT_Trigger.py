#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import speech_recognition as sr
import serial

def receive_wav_from_serial(port='/dev/ttyUSB0', baudrate=115200, timeout=30, output_path='/home/username/input.wav'):
    ser = serial.Serial(port, baudrate, timeout=1)
    rospy.loginfo("STM32로부터 WAV 수신 대기 중...")

    with open(output_path, 'wb') as f:
        start_flag = False
        total_bytes = 0

        start_time = rospy.get_time()
        while rospy.get_time() - start_time < timeout:
            if ser.in_waiting:
                chunk = ser.read(1024)
                if not start_flag and b'RIFF' in chunk:
                    # WAV 헤더 시작 감지
                    idx = chunk.find(b'RIFF')
                    f.write(chunk[idx:])
                    total_bytes += len(chunk[idx:])
                    start_flag = True
                elif start_flag:
                    f.write(chunk)
                    total_bytes += len(chunk)
            rospy.sleep(0.01)

        rospy.loginfo(f"WAV 수신 완료: 총 {total_bytes} 바이트 저장됨.")
    ser.close()
    return output_path if total_bytes > 0 else None

def recognize_speech_from_wav(wav_path):
    recognizer = sr.Recognizer()
    with sr.AudioFile(wav_path) as source:
        audio = recognizer.record(source)
    try:
        text = recognizer.recognize_google(audio, language='ko-KR')
        rospy.loginfo(f"STT 변환 결과: {text}")
        return text
    except sr.UnknownValueError:
        rospy.logwarn("음성을 인식할 수 없습니다.")
        return ""
    except sr.RequestError as e:
        rospy.logerr(f"Google STT 요청 실패: {e}")
        return ""

def main():
    rospy.init_node('stt_trigger_publisher')
    pub = rospy.Publisher("/guidebot/trigger", String, queue_size=1)

    wav_path = receive_wav_from_serial()
    if not wav_path:
        rospy.logerr("WAV 파일 수신 실패")
        return

    text = recognize_speech_from_wav(wav_path)
    if text:
        pub.publish(String(data=text))
        rospy.loginfo("텍스트를 /guidebot/trigger 토픽으로 퍼블리시했습니다.")
    else:
        rospy.logwarn("텍스트가 비어 있어 퍼블리시하지 않았습니다.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
