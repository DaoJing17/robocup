#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import json
import os
from gtts import gTTS
import speech_recognition as sr
from cv_bridge import CvBridge, CvBridgeError
import cv2
from math import radians


class Receptionist:
    def __init__(self):
        rospy.init_node('receptionist_main', anonymous=True)
        self.human_result_pub = rospy.Publisher('human_detected', String, queue_size = 10)
        self.task_status_pub = rospy.Publisher('task_status', String, queue_size=10)

        image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.camera_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=10000000)

        self.sub = rospy.Subscriber('done_analysis', String, self.handle_payload)

        self.nav_pub = rospy.Publisher("nav_cmd", String, queue_size=10)
        self.nav_pub.publish("PinkBox")
        rospy.wait_for_message("nav_cmd", String)

        self.empty_seat_pub = rospy.Publisher("start_empty_seat", String, queue_size=10)
        
        self.RECORD = True    

    # ------------------- Detect Human ---------------------- 
    def image_callback(self, msg):
        if self.RECORD:
            try:
                # 将ROS图像数据转换为OpenCV图像
                cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
                # 调整图像大小以加快处理速度
                # cv_image = cv2.resize(cv_image, (640, 480))
                # if self.select_detect==0:

                # Check if human is present
                humans = self.detect_human(cv_image)
                if len(humans) > 0:
                    rospy.sleep(1)
                    payload = json.dumps("True")
                    self.human_result_pub.publish(payload)
                    self.RECORD = False
                    self.getNameAndDrink()

                print("Done")

            except CvBridgeError as e:
                print(e)


    def handle_payload(self, received_payload):
        rospy.loginfo(received_payload.data)
        self.done_analysis=True
        self.task_status_pub.publish(f'True-{received_payload.data}')
        
    def detect_human(self, image):
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        (humans, _) = hog.detectMultiScale(image, winStride=(8, 8),
        padding=(16, 16), scale=1.05)
        
        return humans
    
    # ---------------------------------------------------- 

    # -------------- get Name & Drink--------------------- 
    
    def text2audio(self, text):
        tts = gTTS(text)
        tts.save("main_audio.mp3")
        os.system("mpg321 main_audio.mp3")
        os.remove("main_audio.mp3")


    def audio2text(self):
        result = ""
        r = sr.Recognizer()
        with sr.Microphone() as source:
            print(">>> Say something!")
            #audio = r.listen(source)
            audio = r.record(source, duration=3) #5 sec
        # recognize speech using Google Speech Recognition
        try:
            result = r.recognize_google(audio).lower()
            print("SR result: " + result)
        except sr.UnknownValueError:
            print("SR could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
        return result


    def getNameAndDrink(self):
        drink_list = ['coffee','tea','beer']

        self.text2audio("Hello,what is your name")
        name = self.audio2text()
        while not name:
            self.text2audio('Sorry, can you please come again?')
            rospy.sleep(3)
            name = self.googlesr()

        hello_str = 'Hello {} What is your favourite drink?'.format(name)
        self.text2audio(hello_str)

        drink = self.googlesr()
        while drink not in drink_list:
            self.text2audio('Sorry, can you please come again?')
            rospy.sleep(3)
            drink = self.googlesr()

        with open('/home/mustar/catkin_ws/src/robocup/script/info.txt', 'w') as f:
            f.write(self.count+" "+name+" "+drink)
            f.close()

        self.text2audio('Nice to meet you. Please wait for me to get your details')
        while not self.done_analysis:
            rospy.sleep(2)
        
        self.text2audio('Thanks for your waiting. Please follow me and stand on my left side')
        self.nav_pub.publish("EmptySeat")
        rospy.wait_for_message("nav_feedback", String)

        self.read_guestlist()
        self.empty_seat_pub.publish("start")
        rospy.wait_for_message("empty_seat_status", String)
        self.nav_pub.publish("PinkBox")
        self.RECORD = True


    def read_guestlist(self):
        with open('/home/mustar/catkin_ws/src/robocup/script/info.txt', 'r') as file:
            lines = file.readlines()

        self.rotate(0)
        
        if lines:
            if len(lines) == 1:
                # guest_details = [count, name, drink, gender, iden1, iden2, age, outwear]
                # 0 = count, 1 = name, 2 = drink, 3 = gender, 4 = iden1
                # 5 = iden2, 6 = age, 7 = outwear
                info = lines[0].split(" ")
                msg = f'Hello everyone! Introducing new guest which called {info[1]} and 
                {info[5]} favourite drink is {info[2]}!'
                self.text2audio(msg)

            else:
                info = lines[-1].split(" ")
                # introduce last guest 
                msg = f'Hello everyone! Introducing new guest which called {info[1]} and 
                {info[5]} favourite drink is {info[2]}!'
                self.text2audio(msg)

                self.text2audio(f"Alright {info[1]}, let me introduce other guest to you")
                self.rotate(1)
                for i in range(len(lines)-1):
                    other = lines[i].split(" ")
                    other_guest_msg = f'We have {other[1]} here.
                    {info[4]} is a {info[3]} with age around {info[6]}. 
                    {info[4]} is wearing a {info[7]} today.'
                    self.text2audio(other_guest_msg)
                self.text2audio("That's all")
        
        self.text2audio("Please wait me to get an empty seat for you.")
            
    
    # ----------------------------------------------------

    # -------------- rotate to face guest --------------------- 
    def rotate(self, clockwise):
        # clockwise = 1, anticlowise = 0
        r = rospy.Rate(5)
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        turn_cmd = Twist()
        turn_cmd.linear.x = 0

        # by default angular.z is 0 so setting this isn't required
        if clockwise == 1:
            rospy.loginfo("Turning clockwise")
            turn_cmd.angular.z = radians(90)
        else:
            rospy.loginfo("Turning anti-clockwise")
            turn_cmd.angular.z = radians(-90)
        self.cmd_vel.publish(turn_cmd)
        r.sleep() 

    # ----------------------------------------------------

            
if __name__ == "__main__":
    try:
        f = Receptionist()
        rospy.spin()
        # f.set_up_waiting()

    except rospy.ROSInterruptException:
        rospy.loginfo("Main Error")
        