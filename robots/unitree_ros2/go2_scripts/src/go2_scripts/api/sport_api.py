#!/usr/bin/env python3

import json

# API ID constants
ROBOT_SPORT_API_ID_DAMP = 1001
ROBOT_SPORT_API_ID_BALANCESTAND = 1002
ROBOT_SPORT_API_ID_STOPMOVE = 1003
ROBOT_SPORT_API_ID_STANDUP = 1004
ROBOT_SPORT_API_ID_STANDDOWN = 1005
ROBOT_SPORT_API_ID_RECOVERYSTAND = 1006
ROBOT_SPORT_API_ID_EULER = 1007
ROBOT_SPORT_API_ID_MOVE = 1008
ROBOT_SPORT_API_ID_SIT = 1009
ROBOT_SPORT_API_ID_RISESIT = 1010
ROBOT_SPORT_API_ID_SWITCHGAIT = 1011
ROBOT_SPORT_API_ID_TRIGGER = 1012
ROBOT_SPORT_API_ID_BODYHEIGHT = 1013
ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT = 1014
ROBOT_SPORT_API_ID_SPEEDLEVEL = 1015
ROBOT_SPORT_API_ID_HELLO = 1016
ROBOT_SPORT_API_ID_STRETCH = 1017
ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW = 1018
ROBOT_SPORT_API_ID_CONTINUOUSGAIT = 1019
ROBOT_SPORT_API_ID_CONTENT = 1020
ROBOT_SPORT_API_ID_WALLOW = 1021
ROBOT_SPORT_API_ID_DANCE1 = 1022
ROBOT_SPORT_API_ID_DANCE2 = 1023
ROBOT_SPORT_API_ID_GETBODYHEIGHT = 1024
ROBOT_SPORT_API_ID_GETFOOTRAISEHEIGHT = 1025
ROBOT_SPORT_API_ID_GETSPEEDLEVEL = 1026
ROBOT_SPORT_API_ID_SWITCHJOYSTICK = 1027
ROBOT_SPORT_API_ID_POSE = 1028
ROBOT_SPORT_API_ID_SCRAPE = 1029
ROBOT_SPORT_API_ID_FRONTFLIP = 1030
ROBOT_SPORT_API_ID_FRONTJUMP = 1031
ROBOT_SPORT_API_ID_FRONTPOUNCE = 1032

class PathPoint:
    def __init__(self):
        self.timeFromStart = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0

class SportAPI:
    def damp(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_DAMP

    def balance_stand(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_BALANCESTAND

    def stop_move(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_STOPMOVE

    def stand_up(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDUP

    def stand_down(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDDOWN

    def recovery_stand(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_RECOVERYSTAND

    def euler(self, req, roll, pitch, yaw):
        js = {"x": roll, "y": pitch, "z": yaw}
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_EULER

    def move(self, req, vx, vy, vyaw):
        js = {"x": vx, "y": vy, "z": vyaw}
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE

    def sit(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_SIT

    def rise_sit(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_RISESIT

    def switch_gait(self, req, d):
        js = {"data": d}
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHGAIT

    def trigger(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_TRIGGER

    def body_height(self, req, height):
        js = {"data": height}
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_BODYHEIGHT

    def foot_raise_height(self, req, height):
        js = {"data": height}
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT

    def speed_level(self, req, level):
        js = {"data": level}
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_SPEEDLEVEL

    def hello(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_HELLO

    def stretch(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_STRETCH

    def trajectory_follow(self, req, path):
        js_path = []
        for point in path:
            js_point = {
                "t_from_start": point.timeFromStart,
                "x": point.x,
                "y": point.y,
                "yaw": point.yaw,
                "vx": point.vx,
                "vy": point.vy,
                "vyaw": point.vyaw
            }
            js_path.append(js_point)
        req.parameter = json.dumps(js_path)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW

    def switch_joystick(self, req, flag):
        js = {"data": flag}
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHJOYSTICK

    def continuous_gait(self, req, flag):
        js = {"data": flag}
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_CONTINUOUSGAIT

    def wallow(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_WALLOW

    def content(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_CONTENT

    def pose(self, req, flag):
        js = {"data": flag}
        req.parameter = json.dumps(js)
        req.header.identity.api_id = ROBOT_SPORT_API_ID_POSE

    def scrape(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_SCRAPE

    def front_flip(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTFLIP

    def front_jump(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTJUMP

    def front_pounce(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTPOUNCE

    def dance1(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_DANCE1

    def dance2(self, req):
        req.header.identity.api_id = ROBOT_SPORT_API_ID_DANCE2