from __future__ import print_function
import qi
from nao_arm_controller import *

import nao_torso_controller
import itertools
import time
import operator
import pygame
import pygame.color

from pygame.color import THECOLORS
from pykinect import nui
from pykinect.nui import JointId
from pykinect.nui import SkeletonTrackingState
from pykinect.nui.structs import TransformSmoothParameters

PREVIOUS = 0
START_TIME = 0
SKELETON_INDEX = -1
OUTPUT_FILE = None
KINECTEVENT = pygame.USEREVENT
WINDOW_SIZE = 640, 480
COLLECT_DATA = False
COUNT = 0

#need to scale from me to the nao.
#I am 1.69 meters, nao is 0.58.
## nao is 0.3432 of me.
## kinect units in m, nao in m.
# magic number:

#HUMAN_TO_NAO_SCALE = 0.0003432
HUMAN_TO_NAO_SCALE = 0.3486205445

SKELETON_COLORS = [THECOLORS["red"],
                   THECOLORS["blue"],
                   THECOLORS["green"],
                   THECOLORS["orange"],
                   THECOLORS["purple"],
                   THECOLORS["yellow"],
                   THECOLORS["violet"]]

LEFT_ARM = (JointId.ShoulderCenter,
            JointId.ShoulderLeft,
            JointId.ElbowLeft,
            JointId.WristLeft,
            JointId.HandLeft)
RIGHT_ARM = (JointId.ShoulderCenter,
             JointId.ShoulderRight,
             JointId.ElbowRight,
             JointId.WristRight,
             JointId.HandRight)
LEFT_LEG = (JointId.HipCenter,
            JointId.HipLeft,
            JointId.KneeLeft,
            JointId.AnkleLeft,
            JointId.FootLeft)
RIGHT_LEG = (JointId.HipCenter,
             JointId.HipRight,
             JointId.KneeRight,
             JointId.AnkleRight,
             JointId.FootRight)
SPINE = (JointId.HipCenter,
         JointId.Spine,
         JointId.ShoulderCenter,
         JointId.Head)

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

skeleton_to_depth_image = nui.SkeletonEngine.skeleton_to_depth_image


def post_frame(frame):
    """Get skeleton events from the Kinect device and post them into the PyGame
    event queue."""
    try:
        pygame.event.post(
            pygame.event.Event(KINECTEVENT, skeleton_frame=frame)
        )
    except:
        # event queue full
        pass


def draw_skeleton_data(dispInfo, screen, pSkeleton, index, positions, width=4):
    start = pSkeleton.SkeletonPositions[positions[0]]

    for position in itertools.islice(positions, 1, None):
        next = pSkeleton.SkeletonPositions[position.value]

        curstart = skeleton_to_depth_image(start, dispInfo.current_w, dispInfo.current_h)
        curend = skeleton_to_depth_image(next, dispInfo.current_w, dispInfo.current_h)

        pygame.draw.line(screen, SKELETON_COLORS[index], curstart, curend, width)

        start = next


def draw_skeletons(dispInfo, arm_controller, screen, skeletons):
    global SKELETON_INDEX
    global PREVIOUS
    # initialize time if uninitialized
    if COLLECT_DATA == True:
        elapsed_time = 0
        global START_TIME
        if START_TIME == 0:
            START_TIME = time.time()
        else:
            elapsed_time = int((time.time() - START_TIME) * 1000)
    # clean the screen
    screen.fill(pygame.color.THECOLORS["black"])

    for index, skeleton_info in enumerate(skeletons):
        # only track one skeleton
        if SKELETON_INDEX == -1:
            SKELETON_INDEX = index
        # test if the current skeleton is tracked or not
        if skeleton_info.eTrackingState == SkeletonTrackingState.TRACKED:
            global COUNT
            if COLLECT_DATA == True:
                global COUNT
                data_lines = ''
                for joint_id, pos in enumerate(skeleton_info.SkeletonPositions):
                    if pos.x == 0 and pos.y == 0 and pos.z == 0:
                        data_lines = ''
                        break
                    depth_x, depth_y = skeleton_to_depth_image(pos, dispInfo.current_w, dispInfo.current_h)
                    details = [COUNT, elapsed_time, joint_id, pos.x, pos.y, pos.z, pos.w, depth_x, depth_y, index]
                    data_lines += ','.join(map(str, details)) + "\n"
                if data_lines != '':
                    print(data_lines, end='')
                    COUNT += 1
            if COUNT % 25 == 0:
                human_hipcenter = skeleton_info.SkeletonPositions[JointId.HipCenter]
                human_spine = skeleton_info.SkeletonPositions[JointId.Spine]
                human_righthand = skeleton_info.SkeletonPositions[JointId.HandRight]
                human_lefthand = skeleton_info.SkeletonPositions[JointId.HandLeft]
                nao_right_arm = calculate_nao_right_arm_coords(human_hipcenter, human_spine, human_righthand)
                nao_left_arm = calculate_nao_left_arm_coords(human_hipcenter,human_spine, human_lefthand)
                arm_controller.moveArm(nao_right_arm[0],nao_right_arm[1],nao_right_arm[2])
                print(nao_right_arm)
            COUNT+= 1
            # draw the Head
            HeadPos = skeleton_to_depth_image(skeleton_info.SkeletonPositions[JointId.Head], dispInfo.current_w,
                                              dispInfo.current_h)
            draw_skeleton_data(dispInfo, screen, skeleton_info, index, SPINE, 10)
            pygame.draw.circle(screen, SKELETON_COLORS[index], (int(HeadPos[0]), int(HeadPos[1])), 20, 0)

            # drawing the limbs
            draw_skeleton_data(dispInfo, screen, skeleton_info, index, LEFT_ARM)
            draw_skeleton_data(dispInfo, screen, skeleton_info, index, RIGHT_ARM)
            draw_skeleton_data(dispInfo, screen, skeleton_info, index, LEFT_LEG)
            draw_skeleton_data(dispInfo, screen, skeleton_info, index, RIGHT_LEG)

def start_nao_connection(ip):
    session = qi.Session()
    port = 9559
    session.connect("tcp://" + ip + ":" + str(port))
    return session

def main():


    """Initialize and run the game."""
    pygame.init()

    # Initialize PyGame
    screen = pygame.display.set_mode(WINDOW_SIZE, 0, 16)
    pygame.display.set_caption('PyKinect Skeleton')
    screen.fill(pygame.color.THECOLORS["black"])

    ip = 'lapis.local'
    #ip = 'localhost'
    session = start_nao_connection(ip)
    arm_controller = NaoArmController(session)

    with nui.Runtime() as kinect:
        kinect.skeleton_engine.enabled = True
        kinect.skeleton_frame_ready += post_frame

        # Main game loop
        while True:
            event = pygame.event.wait()

            if event.type == pygame.QUIT:
                break
            elif event.type == KINECTEVENT:
                # apply joint filtering
                kinect._nui.NuiTransformSmooth(event.skeleton_frame, SMOOTH_PARAMS)

                draw_skeletons(pygame.display.Info(), arm_controller, screen, event.skeleton_frame.SkeletonData)
                pygame.display.update()
                pass


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


if __name__ == '__main__':
    main()
