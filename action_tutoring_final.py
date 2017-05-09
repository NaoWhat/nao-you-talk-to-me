#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: A Simple class to get & read FaceDetected Events"""

import qi
import time
import sys
import argparse
import operator
import ftplib
from os import path
import threading

from scipy.spatial import ConvexHull
from matplotlib.path import Path
import numpy as np

from nao_arm_controller import *

from wit import Wit
client = Wit(access_token="3HHBQKF75VSSVNMF5YWHAUG3HSEENTGB")



from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime


#need to scale from me to the nao.
#I am 1.69 meters, nao is 0.58.
## nao is 0.3432 of me.
## kinect units in m, nao in m.
# magic number:

#HUMAN_TO_NAO_SCALE = 0.0003432
HUMAN_TO_NAO_SCALE = 0.3486205445
AUDIO_FILE = path.join(path.dirname(path.realpath(__file__)), "english.wav")

NAO_IP = "lapis.local"
NAO_AUDIO_FILE = "/home/nao/recordings/microphones/test.wav"
LOCAL_AUDIO_FILE = path.join(path.dirname(path.realpath(__file__)), "test.wav")
# CLIENT_ACCESS_TOKEN = "6178e0ebf1a44a8bbc24d51f1fb31dab"


class ActionTutor(object):
    """
    A simple class to tutor actions
    """
    current_mode = "Perform Mode"
    channels = [0, 0, 1, 0] #[left, right, front, rear]

    def __init__(self, app):
        """
        Initialisation of qi framework and event detection.
        """
        super(ActionTutor, self).__init__()
        app.start()
        session = app.session
        # Get the service ALMemory.
        self.memory = session.service("ALMemory")
        # Connect the event callback.

        # self.subscriber = self.memory.subscriber("WordRecognized")
        # self.subscriber.signal.connect(self.on_word_recognized)

        # Get the services ALTextToSpeech, ALAudioRecorder and ALFaceDetection.
        self.tts = session.service("ALTextToSpeech")
        self.audio_recorder = session.service("ALAudioRecorder")
        #self.face_detection = session.service("ALFaceDetection")
        #self.face_detection.subscribe("HumanGreeter")
        self.got_face = False

        '''
        ####initialize speech recognition
        self.asr = session.service("ALSpeechRecognition")
        # Example: set the language of the speech recognition engine to English:
        self.asr.setLanguage("English")
        self.vocabulary = ["Hello, Yes, No"]
        # Or, if you want to enable word spotting:
        self.asr.setVocabulary(self.vocabulary, True)
        self.asr.subscribe("ActionTutor")
        '''
        self.isRecordingAction = False
        #self.tts.say("Ready")


        ####initialize counters
        self.count = 0
        self.start_time = time.time()
        self.elapsed_time = time.time()

        ####initialize kinect
        self.kinect =  PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)

        ####initialize movement modules

        self.moveDictionary = {} #
        self.currentMove = {}

        ####initialize arm movements
        self.arm_controller = NaoArmController(session)

        #### Safety bound initialization

        self.use_safety_bounds = True
        if self.use_safety_bounds:
            #rarm
            self.rarm_points = np.load('rarm_points.npy')
            #larm
            self.larm_points = np.load('larm_points.npy')
        else:
            self.rarm_hull = None
            self.larm_hull = None

    def on_word_recognized(self, value):
        """
        Callback for event WordRecognized.
        """
        print ("recognized word")
        print (value)


    def check_kinect(self):
        print("hi from loop")
        if self.kinect.has_new_body_frame() and self.isRecordingAction:
            bodies = self.kinect.get_last_body_frame()
            self.post_kinect_frame(bodies)
            time.sleep(.1)


    def run(self):
        """
        Loop on, wait for events until manual interruption.
        """
        print "Starting ActionTutor"
        t = threading.Thread(target=self.check_kinect)
        t.start()
        try:
            while True:
                print("Hit ENTER to begin recording!")
                wait = raw_input()
                self.audio_recorder.startMicrophonesRecording(NAO_AUDIO_FILE, "wav", 16000, self.channels)
                print("Hit ENTER to stop recording!")
                wait = raw_input()
                self.audio_recorder.stopMicrophonesRecording()

                ftp = ftplib.FTP(NAO_IP, 'nao', 'admin')
                testFile = open(LOCAL_AUDIO_FILE, "wb")
                ftp.retrbinary('RETR recordings/microphones/test.wav', testFile.write)
                testFile.close()
                ftp.quit()

                resp = None
                with open(LOCAL_AUDIO_FILE, 'rb') as f:
                  resp = client.speech(f, None, {'Content-Type': 'audio/wav'})
                print('Yay, got Wit.ai response: ' + str(resp))
                self.act_on_response(resp)

                # if self.kinect.has_new_body_frame():
                #     bodies = self.kinect.get_last_body_frame()
                #     self.post_kinect_frame(bodies)
                # time.sleep(1)
        except KeyboardInterrupt:
            print "Interrupted by user, stopping ActionTutor"
            #self.face_detection.unsubscribe("HumanGreeter")
            #stop
            sys.exit(0)

    def act_on_response(self, resp):
        # global self.current_mode
        spoken_text = resp['_text']

        max_part_confidence = None
        max_part_value = None   # arm, leg, torso
        max_direction_confidence = None
        max_direction_value = None # left, right, up, down
        max_intent_confidence = None
        max_intent_value = None #

        if 'intent' in resp['entities'].keys():
            intent = resp['entities']['intent']

        if 'direction' in resp['entities'].keys():
            direction = resp['entities']['direction']
            max_direction_confidence = 0.0
            for items in direction:
                if items['confidence'] > max_direction_confidence:
                    max_direction_confidence = items['confidence']
                    max_direction_value = items['value']

        if 'part' in resp['entities'].keys():
            part = resp['entities']['part']
            max_part_confidence = 0.0
            for items in part:
                if items['confidence'] > max_part_confidence:
                    max_part_confidence = items['confidence']
                    max_part_value = items['value']

        max_intent_confidence = 0.0
        for items in intent:
            if items['confidence'] > max_intent_confidence:
                max_intent_confidence = items['confidence']
                max_intent_value = items['value']

        if max_intent_value == "perform":
            if self.current_mode == "Perform Mode":
                self.tts.say("I am already in perform mode!")
            else:
                self.current_mode = "Perform Mode"
                self.tts.say("Switching to perform mode!")

            if max_part_value == "arm":
                self.tts.say("You said " + str(resp['_text']) + ". I understood that you want me to " + str(max_intent_value) + " an action with my " + str(max_direction_value) + str(max_part_value) + ".")
                self.tts.say(" My learning model reported a confidence of " + str(int(max_intent_confidence * 100)) + "percent for perform, " + str(int(max_part_confidence * 100)) + " percent for arm and " + str(int(max_direction_confidence * 100)) + " percent for direction")

                self.perform_action()
            else:
                self.tts.say("done")

        elif max_intent_value == "learn":
            if self.current_mode == "Learn Mode":
                self.tts.say("I am already in learn mode")
            else:
                self.current_mode = "Learn Mode"
                self.tts.say("You said" + str(resp['_text']) + ". I understand that you want me to " + str(max_intent_value) + ". I have switched to my learn mode now.")
                self.tts.say("My learning model reported a confidence of " + str(int(max_intent_confidence * 100)) + " percent for " +  str(max_intent_value))
                print("LEARN MODE: \t Press ENTER to start learning...")
                wait = raw_input()
                self.tts.say("Learning now!")

                # KINECT RECORDING STARTS...
                self.isRecordingAction = True

                print("LEARN MODE: \t Hit ENTER to stop learning...")
                wait = raw_input()

                # Save Action now...
                self.save_action()

    def post_kinect_frame(self, bodies):
        """Get skeleton events from the Kinect device and post them into the PyGame
        event queue."""

        # --- draw skeletons to _frame_surface
        if bodies is not None:
            for i in range(0, self.kinect.max_body_count):
                body = bodies.bodies[i]
                if not body.is_tracked:
                    continue

                joints = body.joints
                # convert joint coordinates to color space


                print(self.count)
                if self.count % 3 == 0:
                    human_spinemid = joints[PyKinectV2.JointType_SpineMid].Position
                    human_righthand = joints[PyKinectV2.JointType_WristRight].Position
                    human_lefthand = joints[PyKinectV2.JointType_WristLeft].Position
                    nao_right_arm = calculate_nao_right_arm_coords(human_spinemid, human_righthand)
                    nao_left_arm = calculate_nao_left_arm_coords(human_spinemid, human_lefthand)
                    if self.use_safety_bounds:


                        ###check right arm
                        rarm_is_in_bounds = is_point_in_hull(list(nao_right_arm), self.rarm_points)
                        if rarm_is_in_bounds:
                            print("right good")
                            ###record action
                            if self.isRecordingAction:
                                self.elapsed_time = (time.time() - self.start_time)
                                self.append_to_current_move('RArm', (self.elapsed_time, nao_right_arm))
                            self.arm_controller.moveArmName('RArm', nao_right_arm[0],nao_right_arm[1],nao_right_arm[2])
                        else:
                            print("right out of bounds")
                        '''

                        ###check left arm
                        larm_is_in_bounds = is_point_in_hull(list(nao_left_arm), self.larm_points)
                        if larm_is_in_bounds:
                            print("left good")
                            ###record action
                            if self.isRecordingAction:
                                self.elapsed_time = (time.time() - self.start_time)
                                self.append_to_current_move('LArm', (self.elapsed_time, nao_left_arm))
                            self.arm_controller.moveArmName('LArm', nao_left_arm[0],nao_left_arm[1],nao_left_arm[2])
                        else:
                            print("left out of bounds")

                        '''
                    print(nao_left_arm)
                    print(self.arm_controller.getPosition("LArm"))
                self.count += 1

    def append_to_current_move(self, joint_name, position):
        if self.currentMove.has_key('RArm'):
            self.currentMove[joint_name] = self.currentMove[joint_name].append(position)
        else:
            self.currentMove[joint_name] = [position]

    def save_action(self):
        self.isRecordingAction = False
        self.moveDictionary = self.currentMove
        self.currentMove = {} # clear current move

    def start_recording(self):
        self.isRecordingAction = True
        self.start_time = time.time()

    def perform_action(self):
        self.arm_controller.performAction(self.moveDictionary)



def vector_to_tuple(vector):
    return (vector.x, vector.y, vector.z)


def is_point_in_hull(point, points):
    hull = ConvexHull(points)
    new_points = np.vstack([points,point])
    new_hull = ConvexHull(new_points)
    if len(hull.vertices)==len(new_hull.vertices) and (hull.vertices, new_hull.vertices):
        return True
    else:
        return False

def calculate_nao_right_arm_coords(human_torso_xyz, human_right_hand_xyz):
    human_torso_xyz = vector_to_tuple(human_torso_xyz)
    human_right_hand_xyz = vector_to_tuple(human_right_hand_xyz)
    right_hand_torso  = tuple(map(operator.sub, human_torso_xyz, human_right_hand_xyz))
    right_hand_torso  = tuple(HUMAN_TO_NAO_SCALE*x for x in right_hand_torso)
    nao_right_hand_xyz = (right_hand_torso[2], right_hand_torso[0], -right_hand_torso[1])
    return nao_right_hand_xyz

def calculate_nao_left_arm_coords(human_torso_xyz, human_left_hand_xyz):
    human_torso_xyz = vector_to_tuple(human_torso_xyz)
    human_left_hand_xyz = vector_to_tuple(human_left_hand_xyz)
    left_hand_torso  = tuple(map(operator.sub, human_torso_xyz, human_left_hand_xyz))
    left_hand_torso  = tuple(HUMAN_TO_NAO_SCALE*x for x in left_hand_torso)
    nao_left_hand_xyz = (left_hand_torso[2], left_hand_torso[0], -left_hand_torso[1])
    return nao_left_hand_xyz



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default=NAO_IP,
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    try:
        # Initialize qi framework.
        connection_url = "tcp://" + args.ip + ":" + str(args.port)
        app = qi.Application(["ActionTutor", "--qi-url=" + connection_url])
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    human_greeter = ActionTutor(app)
    human_greeter.run()