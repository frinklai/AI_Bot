#!/usr/bin/env python3
#encoding:utf-8

import speech_recognition as sr
import time
import os
# import re
# ros
import rospy
from speech.msg import SR
from strategy.msg import speech_status
name1 = " "
name2 = " "
catch = -1

def SpeechRecog():
    #Record Audio
    r = sr.Recognizer()
    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(source, duration=0.5)
        print("Say something!")
        audio = r.listen(source)
    # Speech recognition using Google Speech Recognition
    # for testing purposes, we're just using the default API key
    # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
    # instead of `r.recognize_google(audio)`
    try:
        name = r.recognize_google(audio,language = 'zh-TW')
        print(name)
        return name
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

def get_speech_status(data):
    speech.status = data.status

if __name__ == '__main__':
    try:
        rospy.init_node('speech_pub', anonymous=True)
        speech_recognition_pub = rospy.Publisher("/speech/check", SR, queue_size=10)
        rate = rospy.Rate(1) # hz
        check = SR()
        global speech
        speech = speech_status()
        rospy.Subscriber('/speech/status',speech_status , get_speech_status)
        while not rospy.is_shutdown():
            check.speech_check = 0
            check.confirm = 0

            if speech.status == 0:
                print("wait!!")
                rospy.sleep(1)
                # print(speech.status)
            
            elif speech.status == 1:
                print("\n抓什麼")
                name1 = SpeechRecog()
                # print(name1)
                # name1 = input('抓什麼\n')
                if name1 != None:
                    catch = name1.find(u'抓')
                if catch != -1:
                    cnt = catch + 1
                    for i in range(len(name1) - catch - 1):
                        if (name1[cnt+i] == '瓶') or (name1[cnt+i] == '塑' and name1[cnt+i+1] == '膠'):
                            print('catch bottle')
                            check.speech_check = 1
                            break
                        elif (name1[cnt+i] == '紙') or (name1[cnt+i] == '鋁' and name1[cnt+i+1] == '箔'):
                            print('catch paper')
                            check.speech_check = 2
                            break
                        elif (name1[cnt+i] == "鐵") or (name1[cnt+i] == '金' and name1[cnt+i+1] == '屬'):
                            print('catch metal')
                            check.speech_check = 3
                            break
                    if check.speech_check == 0:
                        print('沒有這樣東西')
                else:
                    print('請說要"抓"什麼')
                print('check = ' + str(check.speech_check))
                if check.speech_check != 0:
                    speech.status = 0

            elif speech.status == 2:
                print('\n是or不是')
                name2 = SpeechRecog()
                if name2 == None:
                    name2 = "不確定"
                # name2 = input('是or不是\n')
                for i in range(len(name2)):
                    if name2[i] == '是' or name2[i] == '對':
                        check.confirm = 1
                        break
                    elif (name2[i] == '不' and name2[i+1] == '是') or (name2[i] == '不' and name2[i+1] == '對'):
                        check.confirm = 2
                        break
                    else:
                        check.confirm = 0
                # print(check.confirm)
                if check.confirm == 0:
                    print('不確定,再說一次')
                elif check.confirm != 0:
                    speech.status = 0

            speech_recognition_pub.publish(check)
            rospy.sleep(1)

    except rospy.ROSInterruptException:
        pass
