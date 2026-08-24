#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from gtts import gTTS
import os
import subprocess
import time
import threading
from dotenv import load_dotenv

load_dotenv()

AIRPODS_SINK = os.getenv("TTS_AUDIO_SINK", "bluez_sink.28_11_A5_BC_BE_F9.a2dp_sink")

mp3_path = "/tmp/instruction.mp3"
wav_path = "/tmp/instruction.wav"

# 전역 변수 (last_text_lock으로 보호)
last_text_lock = threading.Lock()
last_text = ""

def play_audio_loop():
    global last_text

    while not rospy.is_shutdown():
        with last_text_lock:
            text_to_play = last_text

        if text_to_play:
            try:
                # TTS 생성 및 저장
                tts = gTTS(text=text_to_play, lang='ko')
                tts.save(mp3_path)
                rospy.loginfo(f"[TTS] MP3 생성 완료: {mp3_path}")

                # MP3 → WAV 변환
                convert_result = subprocess.run(
                    ["ffmpeg", "-y", "-i", mp3_path, wav_path],
                    stdout=subprocess.DEVNULL, stderr=subprocess.PIPE
                )
                if convert_result.returncode != 0:
                    rospy.logerr(f"[TTS] ffmpeg 변환 실패: {convert_result.stderr.decode(errors='ignore')}")
                else:
                    rospy.loginfo(f"[TTS] WAV 변환 완료: {wav_path}")

                    # WAV 파일을 에어팟으로 재생
                    play_result = subprocess.run(["paplay", f"--device={AIRPODS_SINK}", wav_path])
                    if play_result.returncode != 0:
                        rospy.logerr(f"[TTS] paplay 재생 실패 (return code {play_result.returncode})")
                    else:
                        rospy.loginfo("[TTS] 음성 재생 완료")

                # 생성된 임시 파일 정리 (존재할 때만)
                for path in (mp3_path, wav_path):
                    if os.path.exists(path):
                        os.remove(path)
                rospy.loginfo("[TTS] 음성 파일 삭제 완료")

                # 재생하는 동안 새 안내가 들어왔다면(last_text가 바뀌었다면) 지우지 않고
                # 다음 루프에서 그 새 안내를 재생한다. 그렇지 않으면 재생 완료로 비운다.
                with last_text_lock:
                    if last_text == text_to_play:
                        last_text = ""
            except Exception as e:
                rospy.logerr(f"[TTS] 오류 발생: {e}")
        else:
            time.sleep(0.5)

def instruction_callback(msg):
    global last_text
    with last_text_lock:
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
