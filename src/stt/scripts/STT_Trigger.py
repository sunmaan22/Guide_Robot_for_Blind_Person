#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty, String
import pyaudio
import wave
import webrtcvad
import speech_recognition as sr

class VADAudio:
    def __init__(self, rate=16000, frame_duration=30):
        self.rate = rate
        self.format = pyaudio.paInt16
        self.channels = 1
        self.frame_duration = frame_duration  # ms (must be 10, 20, or 30)
        self.frame_size = int(rate * frame_duration / 1000)  # in samples
        self.frame_bytes = self.frame_size * 2  # 16-bit = 2 bytes
        self.vad = webrtcvad.Vad(1)

        self.pa = pyaudio.PyAudio()
        self.stream = self.pa.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.frame_size
        )

    def read_frame(self):
        return self.stream.read(self.frame_size, exception_on_overflow=False)

    def record_until_silence(self, max_silence_frames=30):
        print("[🎙️] 음성 녹음 시작 (무음 감지)")
        frames = []
        silence_counter = 0
        speech_detected = False  # 💡 음성이 처음 감지되었는지 여부

        while True:
            frame = self.read_frame()
            if len(frame) != self.frame_bytes:
                continue

            is_speech = self.vad.is_speech(frame, self.rate)
            frames.append(frame)

            if is_speech:
                speech_detected = True
                silence_counter = 0
            else:
                if speech_detected:  # ✅ 음성 감지 후 무음만 센다
                    silence_counter += 1
                    if silence_counter > max_silence_frames:
                        break

        print("[✅] 무음 감지로 녹음 종료")
        return b''.join(frames)

    def save_wav(self, audio_data, filename):
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(self.pa.get_sample_size(pyaudio.paInt16))  # 🔧 수정된 부분
            wf.setframerate(self.rate)
            wf.writeframes(audio_data)

    def close(self):
        self.stream.stop_stream()
        self.stream.close()
        self.pa.terminate()


def run_google_stt(filename):
    recognizer = sr.Recognizer()
    try:
        with sr.AudioFile(filename) as source:
            audio = recognizer.record(source)
        text = recognizer.recognize_google(audio, language='ko-KR')
        rospy.loginfo(f"[📝] 인식 결과: {text}")
        return text
    except Exception as e:
        rospy.logwarn(f"[⚠️] STT 실패: {e}")
        return ""


def callback(msg):
    vad = VADAudio()
    audio_data = vad.record_until_silence()
    vad.save_wav(audio_data, "record.wav")
    vad.close()

    text = run_google_stt("record.wav")
    if text:
        pub.publish(text)
    stt_done_pub.publish(Empty())  # STT 완료되면 wake node에게 알림
    nav_pub.publish(text)


if __name__ == '__main__':
    rospy.init_node('stt_node')
    pub = rospy.Publisher('/recognized_text', String, queue_size=1)
    stt_done_pub = rospy.Publisher('/stt_done', Empty, queue_size=1)
    rospy.Subscriber('/wake_word_detected', Empty, callback)
    nav_pub = rospy.Publisher('/navigation_command', String, queue_size=1)
    rospy.loginfo("[🧠] STT 노드 대기 중...")
    rospy.spin()
