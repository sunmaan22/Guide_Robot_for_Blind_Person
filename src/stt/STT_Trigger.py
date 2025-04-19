#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import speech_recognition as sr

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

    wav_file_path = "/home/username/input.wav"  # WAV 파일 경로 수정 필요

    rospy.loginfo("WAV 파일을 STT로 변환 중...")
    text = recognize_speech_from_wav(wav_file_path)
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
