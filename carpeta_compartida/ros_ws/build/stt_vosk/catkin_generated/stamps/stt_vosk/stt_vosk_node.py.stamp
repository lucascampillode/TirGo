#!/usr/bin/env python3
import os, json
import pyaudio
from vosk import Model, KaldiRecognizer
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("stt_vosk")

    model_path = rospy.get_param("~model_path", os.path.expanduser("~/models/vosk-es-small"))
    sample_rate = int(rospy.get_param("~sample_rate", 16000))
    device_index = rospy.get_param("~device_index", None)   # None: por defecto
    chunk = int(rospy.get_param("~chunk", 8000))            # ~0.5 s a 16kHz
    publish_partial = rospy.get_param("~publish_partial", True)
    wake_word = rospy.get_param("~wake_word", "").lower()   # ej: "hola tirgo"

    if not os.path.isdir(model_path):
        rospy.logerr("Modelo Vosk no encontrado en: %s", model_path)
        return

    rospy.loginfo("Cargando modelo Vosk: %s", model_path)
    model = Model(model_path)
    rec = KaldiRecognizer(model, sample_rate)
    rec.SetWords(True)

    pub_text = rospy.Publisher("stt/text", String, queue_size=10)
    pub_partial = rospy.Publisher("stt/partial", String, queue_size=10)

    pa = pyaudio.PyAudio()
    stream = pa.open(format=pyaudio.paInt16,
                     channels=1,
                     rate=sample_rate,
                     input=True,
                     frames_per_buffer=chunk,
                     input_device_index=device_index)
    stream.start_stream()

    rospy.loginfo("STT Vosk escuchando… (Ctrl+C para salir)")
    try:
        while not rospy.is_shutdown():
            data = stream.read(chunk, exception_on_overflow=False)
            if rec.AcceptWaveform(data):
                res = json.loads(rec.Result())
                text = (res.get("text") or "").strip()
                if text:
                    # Wake word opcional
                    if wake_word and wake_word in text.lower():
                        pub_text.publish("__WAKE__")
                        rospy.loginfo("✔ Wake word detectada")
                    pub_text.publish(text)
                    rospy.loginfo("✔ Final: %s", text)
            else:
                if publish_partial:
                    par = json.loads(rec.PartialResult()).get("partial", "").strip()
                    if par:
                        pub_partial.publish(par)
    except KeyboardInterrupt:
        pass
    finally:
        stream.stop_stream()
        stream.close()
        pa.terminate()

if __name__ == "__main__":
    main()
