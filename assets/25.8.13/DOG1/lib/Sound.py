#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    :2024/10/14
# @Author  :WuJunYi
# @File    :Sound.py
# @Software:PyCharm
from xgoedu import XGOEDU

class Sound():
    @staticmethod
    def play_sound(animal_name):
        XGO_EDU = XGOEDU()

        if animal_name == "dog":
            XGO_EDU.xgoSpeaker("dog.wav")

        elif animal_name == "cat":
            XGO_EDU.xgoSpeaker("cat.wav")

        elif animal_name == "bird":
            XGO_EDU.xgoSpeaker("bird.wav")

        elif animal_name == "elephant":
            XGO_EDU.xgoSpeaker("elephant.wav")

        elif animal_name == "giraffe":
            XGO_EDU.xgoSpeaker("giraffe.wav")

        elif animal_name == "horse":
            XGO_EDU.xgoSpeaker("horse.wav")
