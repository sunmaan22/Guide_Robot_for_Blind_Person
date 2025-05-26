#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty
import pvporcupine
import pyaudio
import numpy as np
from scipy.signal import resample_poly
import threading

ACCESS_KEY = "s/Wlero0+Jv8EUj66HR89gxYx7COLCNRl5E7Diixx4yqgasz7GJBVg=="
KEYWORD = "하이 바둑"
MODEL_PATH = "/home/sunmaan/catkin_ws/src/wake_word/porcupine_params_ko.pv"
PPN_PATH = "/home/sunmaan/catkin_ws/src/wake_word/하이-바둑_ko_raspberry-pi_v3_0_0.ppn"

MIC_RATE = 48000
TARGET_RATE = 16000
INPUT_DEVICE_INDEX = 1  # 본인의 장치 번호에 맞게 조정

class WakeWordNode:
    def __init__(self):
        rospy.init_node('wake_word_node')
        self.pub = rospy.Publisher('/wake_word_detected', Empty, queue_size=1)
        rospy.Subscriber('/stt_done', Empty, self.stt_done_callback)
        rospy.loginfo("[🚀] Porcupine 기반 wake word 감지 노드 시작 (키워드: '%s')", KEYWORD)

        self.porcupine = pvporcupine.create(
            access_key=ACCESS_KEY,
            keyword_paths=[PPN_PATH],
            model_path=MODEL_PATH
        )

        self.buffer_size = int(MIC_RATE / TARGET_RATE * self.porcupine.frame_length)
        self.pa = pyaudio.PyAudio()
        self.audio_stream = None
        self.lock = threading.Lock()
        self.stt_active = False

        self.open_audio_stream()
        self.run()

    def open_audio_stream(self, retry_count=3, delay_sec=1.0):
        if self.audio_stream:
            return

        for attempt in range(retry_count):
            try:
                self.audio_stream = self.pa.open(
                    rate=MIC_RATE,
                    channels=1,
                    format=pyaudio.paInt16,
                    input=True,
                    frames_per_buffer=self.buffer_size,
                    input_device_index=INPUT_DEVICE_INDEX
                )
                rospy.loginfo("[🎤] 마이크 스트림 열기 성공 (시도 %d)", attempt + 1)
                return
            except OSError as e:
                rospy.logwarn("[⚠️] 마이크 열기 실패 (시도 %d/%d): %s", attempt + 1, retry_count, e)
                rospy.sleep(delay_sec)

        rospy.logerr("[💥] 마이크 열기 실패: 장치가 여전히 사용 중입니다.")

    def close_audio_stream(self):
        if self.audio_stream:
            self.audio_stream.stop_stream()
            self.audio_stream.close()
            self.audio_stream = None

    def stt_done_callback(self, msg):
        with self.lock:
            rospy.loginfo("[🎧] STT 완료됨 → Wake word 감지 재개")
            rospy.sleep(1.0)  # 마이크 반납 대기
            self.open_audio_stream()
            self.stt_active = False

    def run(self):
        try:
            while not rospy.is_shutdown():
                with self.lock:
                    if self.stt_active:
                        continue

                data = self.audio_stream.read(self.buffer_size, exception_on_overflow=False)
                audio_data = np.frombuffer(data, dtype=np.int16).astype(np.float32)
                resampled = resample_poly(audio_data, TARGET_RATE, MIC_RATE)
                resampled = resampled[:self.porcupine.frame_length].astype(np.int16)

                if self.porcupine.process(resampled) >= 0:
                    rospy.loginfo("[🔔] Wake word 감지됨!")
                    with self.lock:
                        self.stt_active = True
                        self.close_audio_stream()
                    self.pub.publish(Empty())
        except KeyboardInterrupt:
            pass
        finally:
            self.close_audio_stream()
            self.pa.terminate()
            self.porcupine.delete()

if __name__ == "__main__":
    WakeWordNode()
