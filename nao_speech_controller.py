import nao_arm_controller
import nao_torso_controller

__author__ = 'Leandra'
#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Use positionInterpolations Method"""

import qi
import argparse
import sys
import almath
import motion


class NaoSpeechController():
    def __init__(self, session):
        self.session = session
        self.asr_service = self.session.service("ALSpeechRecognition")

        self.asr_service.setLanguage("English")
        vocabulary = ["yes", "no", "please"]
        self.asr.setVocabulary(vocabulary, False)

    def setVocabulary(self, vocabulary, word_spotting=False):
        # Example: Adds "yes", "no" and "please" to the vocabulary
        vocabulary = ["yes", "no", "please"]
        self.asr_service.setVocabulary(vocabulary, word_spotting)
        # Or, if you want to enable word spotting:
        #asr.setVocabulary(vocabulary, True)

    def startRecognition(self, user="Test_ASR"):
         # Start the speech recognition engine with user Test_ASR
        self.asr_service.subscribe(user)
        print 'Speech recognition engine started'

    def stopRecognition(self, user="Test_ASR"):
        self.asr_service.unsubscribe(user)

    def moveArm(self, dx, dy, dz):
        # Example showing how to use positionInterpolations
        frame = motion.FRAME_ROBOT
        useSensorValues = False

        # Motion of arm with block process
        axisMaskList = [motion.AXIS_MASK_VEL]
        timeList    = [[1.0]] # seconds

        effectorList = []
        pathList     = []


        effectorList.append("RArm")
        torsoList  = []
        currentPos = self.motion_service.getPosition("RArm", frame, useSensorValues)
        targetPos  = almath.Position6D(currentPos)

        targetPos.x += dx
        targetPos.y += dy
        targetPos.z += dz
        torsoList.append(list(targetPos.toVector()))

        pathList.append(torsoList)

        self.motion_service.positionInterpolations(effectorList, frame, pathList,
                                         axisMaskList, timeList)

    def moveArmY(self, dy):
        # Example showing how to use positionInterpolations
        frame = motion.FRAME_ROBOT
        useSensorValues = False

        # Motion of arm with block process
        axisMaskList = [motion.AXIS_MASK_Y]
        timeList    = [[1.0]] # seconds

        effectorList = []
        pathList     = []


        effectorList.append("RArm")
        torsoList  = []
        currentPos = self.motion_service.getPosition("RArm", frame, useSensorValues)
        targetPos  = almath.Position6D(currentPos)

        targetPos.y += dy
        torsoList.append(list(targetPos.toVector()))

        pathList.append(torsoList)

        self.motion_service.positionInterpolations(effectorList, frame, pathList,
                                         axisMaskList, timeList)

    def moveArmZ(self, dz):
        # Example showing how to use positionInterpolations
        frame = motion.FRAME_ROBOT
        useSensorValues = False

        # Motion of arm with block process
        axisMaskList = [motion.AXIS_MASK_Z]
        timeList    = [[1.0]] # seconds

        effectorList = []
        pathList     = []


        effectorList.append("RArm")
        torsoList  = []
        currentPos = self.motion_service.getPosition("RArm", frame, useSensorValues)
        targetPos  = almath.Position6D(currentPos)

        targetPos.y += dy
        torsoList.append(list(targetPos.toVector()))

        pathList.append(torsoList)

        self.motion_service.positionInterpolations(effectorList, frame, pathList,
                                         axisMaskList, timeList)



if __name__ == "__main__":
    ip = 'lapis.local'
    session = qi.Session()
    port = 9559
    session.connect("tcp://" + ip + ":" + str(port))
    torso_controller = nao_torso_controller.NaoTorsoController(session)
    arm_controller = nao_arm_controller.NaoArmController(session)
    arm_controller.moveArm(0, -0.05, 0)
    torso_controller.moveTorso(0, -0.001, 0)