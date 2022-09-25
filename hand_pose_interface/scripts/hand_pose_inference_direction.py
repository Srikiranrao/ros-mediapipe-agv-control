#!/usr/bin/env python3
# hand_pose_inference.py subscribes to image topic from ros and runs hand pose detection
# After detecting the hand pose it concludes which direction hand is posing 
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import mediapipe as mp
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from google.protobuf.json_format import MessageToDict

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.control_pub = rospy.Publisher('status_hand', String, queue_size=10) # Topic to publish the hand pose direction
    self.bridge = CvBridge()
    rospy.Subscriber("/usb_cam/image_raw",Image,self.callback) # change the topic name to any image topics 
    self.mp_drawing = mp.solutions.drawing_utils
    self.mp_hands = mp.solutions.hands

  def status(self, input):
     self.control_pub.publish(input)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)

    try:
      with self.mp_hands.Hands(                        # Detection configuration for hand pose direction 
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        max_num_hands=2) as hands:

        cv_image = cv2.cvtColor(cv2.flip(cv_image, 1), cv2.COLOR_BGR2RGB)
      
        cv_image.flags.writeable = False
        results = hands.process(cv_image)

        # Draw the hand annotations on the image.
        cv_image.flags.writeable = True
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        #print("here",results.multi_handedness.classification)
        if results.multi_hand_landmarks:
          for idx, hand_handedness in enumerate(results.multi_handedness):
              #print(hand_handedness.classification[0].label)
              if( hand_handedness.classification[0].label== "Left"):
                for hand_landmarks in results.multi_hand_landmarks:
                  if (hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y < hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].y and hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].y > hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y):
                    self.status("Front")
                  elif (hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y > hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].y and hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].y < hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y):
                    self.status("Back")
                  elif (hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].x < hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].x and hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].y < hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y):
                    self.status("Left")
                  elif (hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].x > hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].x and hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].y < hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y):
                    self.status("Right")
                  elif (hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP].y <  hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP].y):
                    self.status("Stop")
                  else:
                    self.status("waiting for command")
                    #print("waiting for command")        
                  self.mp_drawing.draw_landmarks(
                      cv_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
              else:
                self.status("Change to left hand")
                #print(hand_handedness.classification[0].label)

      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      #cv2.imshow("Image window", cv_image)
      #cv2.waitKey(3)
    except CvBridgeError as e:
        print(e)

def main():
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
