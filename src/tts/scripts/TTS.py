#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from gtts import gTTS
import os
import time
import threading

# AirPods sink 이름 (pactl list short sinks 명령어로 확인 후 정확히 입력)
AIRPODS_SINK = "bluez_sink.7C_C1_80_05_43_18.a2dp_sink"
mp3_path = "/tmp/instruction.mp3"
wav_path = "/tmp/instruction.wav"

# 전역 변수
last_text = ""

def play_audio_loop():
    global last_text

    while not rospy.is_shutdown():
        if last_text:
            try:
                # TTS 생성 및 저장
                tts = gTTS(text=last_text, lang='ko')
                tts.save(mp3_path)
                rospy.loginfo(f"[TTS] MP3 생성 완료: {mp3_path}")

                # MP3 → WAV 변환
                os.system(f"ffmpeg -y -i {mp3_path} {wav_path}")
                rospy.loginfo(f"[TTS] WAV 변환 완료: {wav_path}")

                # WAV 파일을 에어팟으로 재생
                os.system(f"paplay --device={AIRPODS_SINK} {wav_path}")
                rospy.loginfo("[TTS] 음성 재생 완료")

                # 재생 후 바로 삭제
                os.remove(mp3_path)
                os.remove(wav_path)
                rospy.loginfo("[TTS] 음성 파일 삭제 완료")

                # 다음 명령이 들어올 때까지 대기
                last_text = ""
            except Exception as e:
                rospy.logerr(f"[TTS] 오류 발생: {e}")
        else:
            time.sleep(0.5)

def instruction_callback(msg):
    global last_text
    if msg.data != last_text:
        last_text = msg.data
        rospy.loginfo(f"[TTS] 새로운 지시 수신: {last_text}")

def main():
    rospy.init_node("tts_instruction_subscriber")
    rospy.Subscriber("/gps/instruction", String, instruction_callback)

    # 재생 쓰레드 실행
    t = threading.Thread(target=play_audio_loop)
    t.daemon = True
    t.start()

    rospy.loginfo("TTS 노드 실행 중... '/gps/instruction' 수신 대기")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
