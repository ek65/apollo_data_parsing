import math
import os
import sys

import lgsvl

from scenic.simulators.lgsvl.map import setMapPath
setMapPath(__file__, '../GoMentum.xodr')
from scenic.simulators.lgsvl.lgsvl_model import *

from scenic.simulators.lgsvl.simulator import MoveAction, FollowWaypointsAction, SetDestinationAction, SetVelocityAction, CancelWaypointsAction, TrackWaypoints
from scenic.simulators.lgsvl.utils import scenicToLGSVLPosition, scenicToLGSVLRotation
from scenic.core.regions import everywhere

param apolloHDMap = 'Gomentum'
param time_step = 1.0/2

def DriveTo(target):
    def behavior(self):
        nonlocal target
        action = SetDestinationAction(target)
        invoke DoForever(action)
    return behavior

startPos = 41.65625 @ 181.1901245
endPos = 18.71875 @ 220.0730286
positionNoise = 0@0
#positionNoise = Normal(0, 0.1) @ Normal(0, 0.1)


ego = EgoCar at startPos offset by positionNoise,
                  facing 10 deg relative to roadDirection,
                  with behavior DriveTo(endPos),
                  with apolloModules ['Camera', 'Localization', 'Perception', 'Transform', 'Routing', 'Prediction', 'Planning'],
                  with target endPos

def Hesitate(self):
    goal = (self offset by 0 @ 20).toVector()
    goalElev = simulation().groundElevationAt(goal)
    goalPoint = scenicToLGSVLPosition(goal, y=goalElev)
    startPoint = scenicToLGSVLPosition(self.position, y=self.elevation)

    hesitation_point = (Point ahead of self by self.walkDistance).toVector()
    hesitation_point_elevation= simulation().groundElevationAt(hesitation_point)
    hesitation_point_lgsvl = scenicToLGSVLPosition(hesitation_point, y=hesitation_point_elevation)
    waypoints = [
        lgsvl.WalkWaypoint(startPoint, self.startDelay),
        lgsvl.WalkWaypoint(hesitation_point_lgsvl, self.hesitateTime)
    ]
    if self.resumeForward:
        waypoints.append(lgsvl.WalkWaypoint(goalPoint, 0))
    else:
        waypoints.append(lgsvl.WalkWaypoint(startPoint, 0))
    hesitationAction = FollowWaypointsAction(waypoints)

    invoke DoForever(hesitationAction)

ped = Pedestrian at 16.8125 @ 223.7737732,
           facing 90 deg relative to roadDirection,
           with behavior Hesitate,
           with startDelay 10.94,
           with walkDistance 4.504,
           with hesitateTime 2.6655,
           with resumeForward True,
           with regionContainedIn everywhere,   # allow pedestrian to be anywhere
           with requireVisible False

def DoForever(action):
    while True:
        yield action
