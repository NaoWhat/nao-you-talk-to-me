__author__ = 'Leandra'


#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: A Simple class to get & read FaceDetected Events"""

import qi
import time
import sys
import argparse
import operator

from scipy.spatial import ConvexHull
from matplotlib.path import Path
import numpy as np

from nao_arm_controller import *

from pykinect import nui
from pykinect.nui import JointId
from pykinect.nui import SkeletonTrackingState
from pykinect.nui.structs import TransformSmoothParameters


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





class ActionTutor(object):
    """
    A simple class to react to face detection events.
    """

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
        '''
        self.subscriber = self.memory.subscriber("WordRecognized")
        self.subscriber.signal.connect(self.on_word_recognized)
        # Get the services ALTextToSpeech and ALFaceDetection.
        self.tts = session.service("ALTextToSpeech")
        #self.face_detection = session.service("ALFaceDetection")
        #self.face_detection.subscribe("HumanGreeter")
        self.got_face = False

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
            #self.rarm_hull = ConvexHull(points)
            #larm
            self.larm_points = np.load('larm_points.npy')
            #self.larm_hull = ConvexHull(points)
        else:
            self.rarm_hull = None
            self.larm_hull = None




    def on_word_recognized(self, value):
        """
        Callback for event WordRecognized.
        """
        print ("recognized word")
        print (value)


    def run(self):
        """
        Loop on, wait for events until manual interruption.
        """
        print "Starting ActionTutor"
        try:
            while True:
                if self.kinect.has_new_body_frame():
                    bodies = self.kinect.get_last_body_frame()
                    self.post_kinect_frame(bodies)
                time.sleep(1)
        except KeyboardInterrupt:
            print "Interrupted by user, stopping ActionTutor"
            #self.face_detection.unsubscribe("HumanGreeter")
            #stop
            sys.exit(0)

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

                if self.count % 25 == 0:
                    human_spinemid = joints[PyKinectV2.JointType_SpineMid].Position
                    human_righthand = joints[PyKinectV2.JointType_HandRight].Position
                    human_lefthand = joints[PyKinectV2.JointType_HandLeft].Position
                    nao_right_arm = calculate_nao_right_arm_coords(human_spinemid, human_righthand)
                    nao_left_arm = calculate_nao_left_arm_coords(human_spinemid, human_lefthand)
                    if self.isRecordingAction:
                        self.append_to_current_move('RArm', nao_right_arm)
                        self.append_to_current_move('LArm', nao_left_arm)
                    if self.use_safety_bounds:
                        rarm_is_in_bounds = is_point_in_hull(list(nao_right_arm), self.rarm_points)
                        if rarm_is_in_bounds:
                            arm_controller.moveArm(nao_right_arm[0],nao_right_arm[1],nao_right_arm[2])
                        else:
                            print("out of bounds")
                    print(nao_right_arm)
                self.count += 1

    def append_to_current_move(self, joint_name, position):
        if self.currentMove.has_key('RArm'):
            self.currentMove[joint_name] = self.currentMove[joint_name].append(position)
        else:
            self.currentMove[joint_name] = [position]

def vector_to_tuple(vector):
    return (vector.x, vector.y, vector.z)


def is_point_in_hull(point, points):
    hull = ConvexHull(points)
    new_points = np.vstack([points,point])
    print(new_points)
    new_hull = ConvexHull(new_points)
    if np.equal(hull.vertices, new_hull.vertices):
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
    nao_left_hand_xyz = (-left_hand_torso[2], -left_hand_torso[0], left_hand_torso[1])
    return nao_left_hand_xyz


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
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