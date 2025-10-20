#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pyaudio

def main():
    p = pyaudio.PyAudio()
    print("Dispositivos de entrada disponibles:\n")
    for i in range(p.get_device_count()):
        d = p.get_device_info_by_index(i)
        if d.get('maxInputChannels', 0) > 0:
            rate = int(d.get('defaultSampleRate', 0))
            ch   = d.get('maxInputChannels', 0)
            print(f"[{i}] {d['name']}  |  rate max: {rate} Hz  |  channels: {ch}")
    p.terminate()

if __name__ == "__main__":
    main()
