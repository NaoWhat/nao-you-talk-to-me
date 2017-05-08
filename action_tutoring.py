__author__ = 'Leandra'


#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: A Simple class to get & read FaceDetected Events"""

import qi
import time
import sys
import argparse

from pykinect import nui
from pykinect.nui import JointId
from pykinect.nui import SkeletonTrackingState
from pykinect.nui.structs import TransformSmoothParameters


SMOOTH_PARAMS_SMOOTHING = 0.7
SMOOTH_PARAMS_CORRECTION = 0.4
SMOOTH_PARAMS_PREDICTION = 0.7
SMOOTH_PARAMS_JITTER_RADIUS = 0.1
SMOOTH_PARAMS_MAX_DEVIATION_RADIUS = 0.1
SMOOTH_PARAMS = TransformSmoothParameters(SMOOTH_PARAMS_SMOOTHING,
                                          SMOOTH_PARAMS_CORRECTION,
                                          SMOOTH_PARAMS_PREDICTION,
                                          SMOOTH_PARAMS_JITTER_RADIUS,
                                          SMOOTH_PARAMS_MAX_DEVIATION_RADIUS)


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
        self.vocabulary = ["Hello Nao"]
        # Or, if you want to enable word spotting:
        self.asr.setVocabulary(self.vocabulary, True)
        self.asr.subscribe("ActionTutor")
        self.isRecordingAction = False
        #self.tts.say("Ready")

        ####initialize counters
        self.count = 0
        self.start_time = time.time()
        self.elapsed_time = time.time()

        ####initialize kinect
        self.kinect =  nui.Runtime()
        self.kinect.skeleton_engine.enabled = True
        self.kinect.skeleton_frame_ready += self.post_kinect_frame

        ####initialize movement modules

        self.moveDictionary = {}
        self.currentMove = {}



    def on_word_recognized(self, value):
        """
        Callback for event FaceDetected.
        """
        print (value)
        if value == []:  # empty value when the face disappears
            self.got_face = False
        elif not self.got_face:  # only speak the first time a face appears
            self.got_face = True
            print "I saw a face!"
            self.tts.say("Hello, you!")
            # First Field = TimeStamp.
            timeStamp = value[0]
            print "TimeStamp is: " + str(timeStamp)

            # Second Field = array of face_Info's.
            faceInfoArray = value[1]
            for j in range( len(faceInfoArray)-1 ):
                faceInfo = faceInfoArray[j]

                # First Field = Shape info.
                faceShapeInfo = faceInfo[0]

                # Second Field = Extra info (empty for now).
                faceExtraInfo = faceInfo[1]

                print "Face Infos :  alpha %.3f - beta %.3f" % (faceShapeInfo[1], faceShapeInfo[2])
                print "Face Infos :  width %.3f - height %.3f" % (faceShapeInfo[3], faceShapeInfo[4])
                print "Face Extra Infos :" + str(faceExtraInfo)

    def run(self):
        """
        Loop on, wait for events until manual interruption.
        """
        print "Starting ActionTutor"
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print "Interrupted by user, stopping ActionTutor"
            #self.face_detection.unsubscribe("HumanGreeter")
            #stop
            sys.exit(0)

    def post_kinect_frame(self, skeleton_frame):
        """Get skeleton events from the Kinect device and post them into the PyGame
        event queue."""
        try:
            self.kinect._nui.NuiTransformSmooth(skeleton_frame, SMOOTH_PARAMS)
            self.elapsed_time = int((time.time() - self.start_time) * 1000)
            skeletons = skeleton_frame.SkeletonData
            for index, skeleton_info in enumerate(skeletons):

                # test if the current skeleton is tracked or not
                if skeleton_info.eTrackingState == SkeletonTrackingState.TRACKED:

                    if self.count % 25 == 0:
                        human_hipcenter = skeleton_info.SkeletonPositions[JointId.HipCenter]
                        human_spine = skeleton_info.SkeletonPositions[JointId.Spine]
                        human_righthand = skeleton_info.SkeletonPositions[JointId.HandRight]
                        human_lefthand = skeleton_info.SkeletonPositions[JointId.HandLeft]
                        nao_right_arm = calculate_nao_right_arm_coords(human_hipcenter, human_spine, human_righthand)
                        nao_left_arm = calculate_nao_left_arm_coords(human_hipcenter,human_spine, human_lefthand)
                        if self.isRecordingAction:
                            self.append_to_current_move('RArm', nao_right_arm)
                            self.append_to_current_move('LArm', nao_left_arm)
                        arm_controller.moveArm(nao_right_arm[0],nao_right_arm[1],nao_right_arm[2])
                        print(nao_right_arm)
                    self.count += 1


        except:
            # event queue full
            pass

    def append_to_current_move(self, joint_name, position):
        if self.currentMove.has_key('RArm'):
            self.currentMove[joint_name] = self.currentMove[joint_name].append(position)
        else:
            self.currentMove[joint_name] = [position]

def vector_to_tuple(vector):
    return (vector.x, vector.y, vector.z)

def calculate_nao_right_arm_coords(human_center_hip_xyz, human_spine_xyz, human_right_hand_xyz):
    human_spine_xyz = vector_to_tuple(human_spine_xyz)
    human_center_hip_xyz = vector_to_tuple(human_center_hip_xyz)
    human_right_hand_xyz = vector_to_tuple(human_right_hand_xyz)
    human_torso_xyz = tuple(map(operator.add, human_center_hip_xyz, human_spine_xyz))
    human_torso_xyz = tuple(x/2 for x in human_torso_xyz) #get mid spine, which is the torso
    right_hand_torso  = tuple(map(operator.sub, human_torso_xyz, human_right_hand_xyz))
    right_hand_torso  = tuple(HUMAN_TO_NAO_SCALE*x for x in right_hand_torso)
    nao_right_hand_xyz = (right_hand_torso[2], right_hand_torso[0], -right_hand_torso[1])
    return nao_right_hand_xyz

def calculate_nao_left_arm_coords(human_center_hip_xyz, human_spine_xyz, human_left_hand_xyz):
    human_spine_xyz = vector_to_tuple(human_spine_xyz)
    human_center_hip_xyz = vector_to_tuple(human_center_hip_xyz)
    human_left_hand_xyz = vector_to_tuple(human_left_hand_xyz)
    human_torso_xyz = tuple(map(operator.add, human_center_hip_xyz, human_spine_xyz))
    human_torso_xyz = tuple(x/2 for x in human_torso_xyz) #get mid spine, which is the torso
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