#! /usr/bin/env python

import rospy
import json
from std_msgs.msg import String
import os
import cv2
from deepface import DeepFace
import speech_recognition as sr
import dlib
import numpy as np
import pandas as pd
from ultralytics import YOLO
from collections import Counter
from gtts import gTTS

def top_one_frequent_value(lst):
    count = Counter(lst)
    unique_values = list(count.keys())

    if len(unique_values) == 1:
        return unique_values
    else:
        sorted_items = count.most_common(1)
        return sorted_items[0]

def top_two_frequent_values(lst):
    count = Counter(lst)
    unique_values = list(count.keys())

    if len(unique_values) == 1:
        return unique_values
    else:
        # sorted_items = count.most_common(2)
        # top_values = [item[0] for item in sorted_items]
        # return top_values
        sorted_items = count.most_common(1)
        top_values = [item[0] for item in sorted_items]
        return top_values

class FaceRecognition:
    def __init__(self):
        rospy.init_node('get_details')
        
        self.model = YOLO('/home/mustar/catkin_ws/src/face_rec/src/models/shirt_detection.pt')
        self.sub = rospy.Subscriber('video_recorded', String, self.handle_payload)
        self.pub = rospy.Publisher('done_analysis', String, queue_size = 10)
        self.FRAME_SAMPLE_RATE = 2
        
    def handle_payload(self, received_payload):
        print('HELLOOOOOO')
        rospy.loginfo(received_payload.data)
        try:
            data = json.loads(received_payload.data)
            frames = self.sample_frames(data['video'], sample_rate=self.FRAME_SAMPLE_RATE)
            rospy.loginfo(f'Processing {len(frames)} frames...')

            deepFace_result = self.detect_face_emotions(frames)
            gender, iden1, iden2 = self.process_genders(deepFace_result)
            rospy.loginfo(f"GENDER: {gender}")
            age = self.process_age(deepFace_result)
            rospy.loginfo(f"AGE: {age}")

            outwear = self.detect_outwears(frames)
            rospy.loginfo(outwear)
                
            guest_details = [gender, iden1, iden2, age, outwear]
            self.update_guest_details(guest_details)

            self.pub.publish(guest_details)

        except:
            rospy.loginfo("Error with face recognition node...")


    # -------------- Update guest list Function -------------- 
    def update_guest_details(self, list):
        with open('/home/mustar/catkin_ws/src/robocup/script/info.txt', 'r') as file:
            lines = file.readlines()

        # Check if the file is not empty
        if lines:
            # Append new info to the new guest
            lines[-1] = lines[-1] + " ".join(list) + '\n'

        # Write the modified content back to the file
        with open('/home/mustar/catkin_ws/src/robocup/script/info.txt', 'w') as file:
            file.writelines(lines)

    # -------------- Sample Frame Function -------------- 
    def sample_frames(self, video_file, sample_rate=2):
        cap = cv2.VideoCapture(video_file)
        frames = []
        count = 0
        
        while(cap.isOpened()):
            ret, frame = cap.read()
            if not ret:
                break
            if count % sample_rate == 0:
                frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            count += 1
        cap.release()
        
        return frames
    
    # -------------- Gender Classification Function -------------- 
    def process_genders(self, genders):
        gender_list = []
        for frame_result in genders:
            gender = frame_result[0]['dominant_gender']
            gender_list.append(gender)
        gender = top_one_frequent_value(gender_list)

        iden1, iden2=""
        if gender[0] == "Man":
            iden1 = "He"
            iden2 = "His"
        else:
            iden1 = "She"
            iden2 = "Her"

        return gender[0], iden1, iden2
    
    # -------------- Age Classification Function -------------- 
    
    def process_age(self, ages):
        age_list = []
        for frame_result in ages:
            age = frame_result[0]['age']
            age_list.append(age)
        rospy.loginfo(age_list)
        total_sum = sum(age_list)
        length = len(age_list)
        age = int(total_sum/length)
        return age
    
    # -------------- Age & Gender Init Function -------------- 
    def detect_face_emotions(self, frames):
        emotions = []
        rospy.loginfo('DETECTING EMOTIONS...')
        for frame in frames:
            frame_result = DeepFace.analyze(frame, actions=['emotion', 'gender', 'age'], enforce_detection = False)
            rospy.loginfo(frame_result)
            emotions.append(frame_result)
        
        return emotions
    
    # -------------- Outwear Classification Function -------------- 
    def detect_outwears(self, frames):
        outwears = []
        rospy.loginfo('DETECTING OUTWEARS...')
        for frame in frames:
            detected_shirts = self.model.track(frame, persist=True)
            for detected_shirt in detected_shirts:
                try:
                    shirt_json = detected_shirt.tojson()
                    shirt_dict = json.loads(shirt_json)
                    rospy.loginfo(f'Shirt Dict: {shirt_dict}')
                    if len(shirt_dict) == 0:
                        continue
                    for result in shirt_dict:
                        rospy.loginfo(result['name']) 
                        outwears.append(result['name'])      
                except Exception as e:
                    rospy.logerr(f'Error processing detected shirt: {e}')
        
        result = top_two_frequent_values(outwears)
        
        return result
        
if __name__ == "__main__":
    try:
        FaceRecognition()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Face Recognition Node Terminated.")