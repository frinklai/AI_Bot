#encoding:utf-8

import speech_recognition as sr
import os
import re
# ros
import rospy
# from speech.msg import SR
cnt = 0
check = 0
name = " "
catch = " "

# def SpeechRecog():
#     #Record Audio
#     r = sr.Recognizer()
#     with sr.Microphone() as source:
#         r.adjust_for_ambient_noise(source, duration=0.5)
#         print("Say something!")
#         audio = r.listen(source)
#     # Speech recognition using Google Speech Recognition
#     # for testing purposes, we're just using the default API key
#     # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
#     # instead of `r.recognize_google(audio)`
#     try:
#         global name
#         name = r.recognize_google(audio,language = 'zh-TW')
#         print(name)
#         #name = "x前進5公分 y後退4公分 z上升3公分"
#     except sr.UnknownValueError:
#         print("Google Speech Recognition could not understand audio")
#     except sr.RequestError as e:
#         print("Could not request results from Google Speech Recognition service; {0}".format(e))

if __name__ == '__main__':
    try:
        # SpeechRecog()
        name = "去抓手機"
        catch = name.find(u'抓')
        if catch != -1:
            cnt = catch + 1
            for i in range(len(name) - catch - 1):
                if name[cnt+i] == '瓶':
                    print('去抓瓶子')
                    check = 1
                elif name[cnt+i] == '手' and name[cnt+i+1] == '機':
                    print('去抓手機')
                    check = 2
                elif name[cnt+i] == '滑' and name[cnt+i+1] == '鼠':
                    print('去抓滑鼠')
                    check = 3
                
        else:
            print('請說要"抓"什麼')
        if check == 0:
            print('沒有這樣東西')
        print(check)
    except rospy.ROSInterruptException:
        pass