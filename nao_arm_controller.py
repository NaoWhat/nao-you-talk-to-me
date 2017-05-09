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
import time


class NaoArmController():
    def __init__(self, session):
        self.session = session
        self.motion_service  = self.session.service("ALMotion")
        self.posture_service = self.session.service("ALRobotPosture")
        # Wake up robot
        self.motion_service.wakeUp()

        # Send robot to Pose Init
        self.posture_service.goToPosture("StandInit", 0.5)

        # Activate Whole Body Balancer.
        isEnabled  = True
        self.motion_service.wbEnable(isEnabled)

    def moveArmDifference(self, dx, dy, dz):
        # Example showing how to use positionInterpolations
        frame = motion.FRAME_TORSO
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

    def moveArm(self, x, y, z):
        # Example showing how to use positionInterpolations
        frame = motion.FRAME_TORSO
        useSensorValues = False

        # Motion of arm with block process
        axisMaskList = [motion.AXIS_MASK_VEL]
        timeList    = [[1]] # seconds

        effectorList = []
        pathList     = []


        effectorList.append("RArm")
        torsoList  = []
        currentPos = self.motion_service.getPosition("RArm", frame, useSensorValues)
        targetPos  = almath.Position6D(currentPos)

        targetPos.x = x
        targetPos.y = y
        targetPos.z = z
        torsoList.append(list(targetPos.toVector()))

        pathList.append(torsoList)

        #self.motion_service.positionInterpolations(effectorList, frame, pathList,
                                         #axisMaskList, timeList)

        self.motion_service.setPositions("RArm", frame, targetPos.toVector(), 0.25, axisMaskList)

    def moveArmName(self, arm_name, x, y, z):

        # Example showing how to set LArm Position, using a fraction of max speed
        frame     = motion.FRAME_TORSO
        useSensor = False

        # Get the current position of the chainName in the same frame
        current = self.motion_service.getPosition(arm_name, frame, useSensor)

        target = [
            x,
            y,
            z,
            current[3] + 0.0,
            current[4] + 0.0,
            current[5] + 0.0]

        currentPos = self.motion_service.getPosition(arm_name, frame, useSensor)
        targetPos  = almath.Position6D(currentPos)

        targetPos.x = x
        targetPos.y = y
        targetPos.z = z
        fractionMaxSpeed = 0.75
        axisMask         = 7 # just control position

        print("moving")
        self.motion_service.setPositions(arm_name, frame, targetPos.toVector(), fractionMaxSpeed, axisMask)

        #time.sleep(1.0)

    def performAction(self, move_dictionary):
        #move has form move["RArm"] , move["LArm"], which are each lists of x, y, z coordinates
        # Example showing how to set LArm Position, using a fraction of max speed

        # Wake up robot
        self.motion_service.wakeUp()

        # Send robot to Pose Init
        self.posture_service.goToPosture("StandInit", 0.5)

        # Example showing how to use positionInterpolations
        frame = motion.FRAME_TORSO
        useSensorValues = False

        # Motion of Arms with block process
        effectorList = []
        pathList     = []

        axisMaskList = [motion.AXIS_MASK_VEL, motion.AXIS_MASK_VEL]
        timeList     = []         # seconds

        rarm_pathList = []
        larm_pathList = []
        rarm_timeList = []
        larm_timeList = []

        if move_dictionary.has_key("RArm"):
            effectorList.append("RArm")
            current = self.motion_service.getPosition("RArm", frame, useSensorValues)
            position_list = move_dictionary["RArm"]
            for item in position_list:
                timestamp = item[0]
                position = item[1]
                target = [
                    position[0],
                    position[1],
                    position[2],
                    current[3] + 0.0,
                    current[4] + 0.0,
                    current[5] + 0.0]
                rarm_pathList.append(target)
                rarm_timeList.append(timestamp)

        if move_dictionary.has_key("LArm"):
            effectorList.append("LArm")
            current = self.motion_service.getPosition("LArm", frame, useSensorValues)
            position_list = move_dictionary["LArm"]
            for item in position_list:
                timestamp = item[0]
                position = item[1]
                target = [
                    position[0],
                    position[1],
                    position[2],
                    current[3] + 0.0,
                    current[4] + 0.0,
                    current[5] + 0.0]
                rarm_pathList.append(target)
                rarm_timeList.append(timestamp)

        timeList = [rarm_timeList, larm_timeList]
        pathList = [rarm_pathList, larm_pathList]
        self.motion_service.positionInterpolations(effectorList, frame, pathList,
                                     axisMaskList, timeList)



    def getPosition(self):
        frame = motion.FRAME_TORSO
        useSensorValues = False
        currentPos = self.motion_service.getPosition("RArm", frame, useSensorValues)
        currentPos  = almath.Position6D(currentPos)
        return (currentPos.x, currentPos.y, currentPos.z)

    def getPosition(self, arm_name): #"RArm" or "LArm"
        frame = motion.FRAME_TORSO
        useSensorValues = False
        currentPos = self.motion_service.getPosition(arm_name, frame, useSensorValues)
        currentPos  = almath.Position6D(currentPos)
        return (currentPos.x, currentPos.y, currentPos.z)



def main(session):
    """
    This example uses the positionInterpolations method.
    """
    # Get the services ALMotion & ALRobotPosture.

    motion_service  = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")

    # Wake up robot
    motion_service.wakeUp()

    # Send robot to Pose Init
    posture_service.goToPosture("StandInit", 0.5)

    # Example showing how to use positionInterpolations
    frame = motion.FRAME_ROBOT
    useSensorValues = False

    dx = 0.05 # translation axis X (meters)
    dy = 0.04 # translation axis Y (meters)

    # Motion of Arms with block process
    effectorList = []
    pathList     = []

    axisMaskList = [motion.AXIS_MASK_VEL, motion.AXIS_MASK_VEL]
    timeList     = [[1.0], [1.0]]         # seconds

    effectorList.append("LArm")
    currentPos = motion_service.getPosition("LArm", frame, useSensorValues)
    targetPos = almath.Position6D(currentPos)
    targetPos.y -= dy
    pathList.append(list(targetPos.toVector()))

    effectorList.append("RArm")
    currentPos = motion_service.getPosition("RArm", frame, useSensorValues)
    targetPos = almath.Position6D(currentPos)
    targetPos.y += dy
    pathList.append(list(targetPos.toVector()))

    motion_service.positionInterpolations(effectorList, frame, pathList,
                                 axisMaskList, timeList)

    # Motion of Arms and Torso with block process
    axisMaskList = [motion.AXIS_MASK_VEL,
                    motion.AXIS_MASK_VEL,
                    motion.AXIS_MASK_ALL]
    timeList    = [[4.0],
                    [4.0],
                    [1.0, 2.0, 3.0, 4.0]] # seconds

    effectorList = []
    pathList     = []

    effectorList.append("LArm")
    pathList.append([motion_service.getPosition("LArm", frame, useSensorValues)])

    effectorList.append("RArm")
    pathList.append([motion_service.getPosition("RArm", frame, useSensorValues)])

    effectorList.append("Torso")
    torsoList  = []
    currentPos = motion_service.getPosition("Torso", frame, useSensorValues)
    targetPos  = almath.Position6D(currentPos)
    targetPos.y += dy
    torsoList.append(list(targetPos.toVector()))

    targetPos = almath.Position6D(currentPos)
    #targetPos.x -= dx
    torsoList.append(list(targetPos.toVector()))

    targetPos = almath.Position6D(currentPos)
    targetPos.y -= dy
    torsoList.append(list(targetPos.toVector()))

    targetPos = almath.Position6D(currentPos)
    torsoList.append(list(targetPos.toVector()))
    pathList.append(torsoList)

    motion_service.positionInterpolations(effectorList, frame, pathList,
                                 axisMaskList, timeList)

    # Go to rest position
    motion_service.rest()


def main_original():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="lapis.local",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session)

if __name__ == "__main__":
    ip = 'lapis.local'
    session = qi.Session()
    port = 9559
    session.connect("tcp://" + ip + ":" + str(port))
    torso_controller = nao_torso_controller.NaoTorsoController(session)
    arm_controller = NaoArmController(session)
    for i in range(10):
        print(arm_controller.getPosition())
    arm_controller.moveArm(0.058936119079589844, -0.12321364134550095, -0.1028701439499855)
    for i in range(10):
        print(arm_controller.getPosition())
