#encoding:utf-8

import speech_recognition as sr
import os
import re
# ros
import rospy
from speech.msg import SR
name = " "

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
        global name
        name = r.recognize_google(audio,language = 'zh-TW')
        print(name)
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

if __name__ == '__main__':
    try:
        rospy.init_node('speech_pub', anonymous=True)
        speech_recognition_pub = rospy.Publisher("/speech/check", SR, queue_size=10)
        # SpeechRecog()
        name = "去抓保特瓶"
        check = SR()
        check.speech_check = 0
        catch = name.find(u'抓')
        if catch != -1:
            cnt = catch + 1
            for i in range(len(name) - catch - 1):
                if name[cnt+i] == '瓶':
                    print('抓瓶子')
                    check.speech_check = 1
                    break
                elif name[cnt+i] == '手' and name[cnt+i+1] == '機':
                    print('抓手機')
                    check.speech_check = 2
                    break
                elif name[cnt+i] == '滑' and name[cnt+i+1] == '鼠':
                    print('抓滑鼠')
                    check.speech_check = 3
                    break
        else:
            print('請說要"抓"什麼')
        if check.speech_check == 0 and catch != -1:
            print('沒有這樣東西')
        print("check = " + str(check.speech_check))
        speech_recognition_pub.publish(check)
    except rospy.ROSInterruptException:
        pass