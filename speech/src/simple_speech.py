#encoding:utf-8

import speech_recognition as sr
import os
import re
# ros
import rospy
from speech.msg import SR

count = 0
dis = 0
control = 0
if __name__ == '__main__':
    try:
        for count in range(0,10):  
        # Record Audio
            r = sr.Recognizer()
            with sr.Microphone() as source:
                r.adjust_for_ambient_noise(source, duration=0.5)
                print("Say something!")
                audio = r.listen(source)
        
        # Speech recognition using Google Speech Recognition
            # try:
        # for testing purposes, we're just using the default API key
        # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        # instead of `r.recognize_google(audio)`

            #print("You said: " + r.recognize_google(audio,language = 'zh-TW'))
            
            name = r.recognize_google(audio,language = 'zh-TW')
            print(name)
            #name = "x前進5公分 y後退4公分 z上升3公分 z下降2公分"
            if name.isdigit():         
                dis = re.search('\d+', name).group()       
                if name.find(u'x') != -1:
                    dic = 1
                    if name.find(u'前進') != -1:
                        control = 1
                    elif name.find(u'後退') != -1:
                        control = -1
                    else:
                        control = 0
                elif name.find(u'y') != -1:
                    dic = 2
                    if name.find(u'前進') != -1:
                        control = 1
                    elif name.find(u'後退') != -1:
                        control = -1
                    else:
                        control = 0
                elif name.find(u'外') != -1:
                    dic = 2
                    if name.find(u'前進') != -1:
                        control = 1
                    elif name.find(u'後退') != -1:
                        control = -1
                    else:
                        control = 0
                elif name.find(u'z') != -1:
                    dic = 3
                    if name.find(u'上升') != -1:
                        control = 1
                    elif name.find(u'下降') != -1:
                        control = -1    
                    else:
                        control = 0 
                else:
                    control = 0
                print(control)
                print(dis)
            else:
                continue


    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))
    except rospy.ROSInterruptException:
        pass