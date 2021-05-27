#!/usr/bin/env python

# --------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.
# --------------------------------------------------------------------------------------------

import os, sys, select, termios, tty
import threading

import rosgraph
import roslaunch
import roslib
import rosnode
import rospy

from time import sleep
from collections import OrderedDict
from flask import Flask, render_template
from geventwebsocket import WebSocketServer, WebSocketApplication, Resource, WebSocketError

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from pprint import pprint

import settings as settings
from webSocket import webSocket

try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

roslib.load_manifest('holo_nav_dash')

# -----------------------------------------------------------------------------
#
class application:

    # -----------------------------------------------------------------------------
    #
    def __init__(self, port):
        print('Hololens Navigation Dashboard. version ' + settings.appVersion + '\r\n')

        self.context = "HoloNavDash"
        self.doneInitializingEvent = threading.Event()
        self.shutdownEvent = threading.Event()
        self.server = None
        self.fnSendMessage = None
        self.fnQueueMessage = None

        # keeping track of nodes
        self.hololens_ros_bridge = False
        self.hololens_sequence = 0

        self.count = 0
        self.last_floor_normal = None

        #
        # start a thread to monitor nodes
        self.fKeepRunning = True
        self.threadNodes = threading.Thread(target = self.threadMonitorNodes, args=(1,))
        self.threadNodes.start()

        #
        # start a thread for the web server
        self.thread = threading.Thread(target = self.threadWebServer, args=(port,))
        self.thread.start()

        rospy.Subscriber("/hololens/floor_normal", PoseStamped, self.cbFloorNormal)

    # -----------------------------------------------------------------------------
    #
    def threadWebServer(self, port):
        address = '0.0.0.0'

        webSocket.setApplication(self)

        self.flask_app = Flask(__name__)
        self.flask_app.debug = False
    
        @self.flask_app.route("/")
        def index():
           return render_template('index.html')

        self.server = WebSocketServer(
                (address, port),
                Resource(OrderedDict({
                    '/'  : self.flask_app,
                    '/ws': webSocket
                    })),
                debug=False
                )

        print("Web Dashboard started on http://localhost:" + str(port) + ".")

        #
        # signal done initializing...
        self.doneInitializingEvent.set()

        self.server.serve_forever()

    # -----------------------------------------------------------------------------
    #
    def setCallbackSendMessage(self, fnSendMessage):
        self.fnSendMessage = fnSendMessage

    # -----------------------------------------------------------------------------
    #
    def setCallbackQueueMessage(self, fnQueueMessage):
        self.fnQueueMessage = fnQueueMessage

    # -----------------------------------------------------------------------------
    #
    def quitApplication(self):
        self.requestShutdownApplication()

    # -----------------------------------------------------------------------------
    #
    def requestShutdownApplication(self):
        self.shutdownEvent.set()

    # -----------------------------------------------------------------------------
    #
    def close(self):
        self.fKeepRunning = False

        if (self.server is not None):
            print("closing web server...")
            self.server.close()
            self.server = None

        if (self.fnQueueMessage):
            msg = { 'msgType': "exit_handle_queue_messages" }
            self.fnQueueMessage(msg)

    # -----------------------------------------------------------------------------
    #
    def run(self):
        #
        # loop until shutdown request or Ctrl-C...
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if (self.shutdownEvent.wait(timeout=0.5)):
                break;

            rate.sleep()

        print("\r\n")
        self.close()


    # -----------------------------------------------------------------------------
    #
    def threadMonitorNodes(self, arg):
        ID = '/rosnode'
        master = rosgraph.Master(ID) # , master_uri=args.ROS_MASTER_URI)
        print ("Using master at {}".format(master.getUri()))        

        while self.fKeepRunning:
            pepper_robot = False
            hololens_ros_bridge = False
            anchor_localizer = False
            static_calibration = False
            dynamic_adjuster = False
            localizer = False

            nodes = rosnode.get_node_names()
            # print ("Active nodes: " + ', '.join(nodes))
            for node in nodes:
                if (node == "/pepper_robot"):
                    pepper_robot = True
                elif (node == "/hololens_ros_bridge") or (node == "/hololens_ros_bridge_node"):
                    hololens_ros_bridge = True
                elif (node == "/anchor_localizer"):
                    anchor_localizer = True
                elif (node == "/static_calibration"):
                    static_calibration = True
                elif (node == "/online_adjuster"):
                    dynamic_adjuster = True
                elif (node == "/localizer"):
                    localizer = True

                # print ("-- " + node + " --")

                #node_api = rosnode.get_api_uri(master, node)
                #if not node_api:
                #    print("    API URI: error (unknown node: {}?)".format(node))
                #    continue
                # print ("    API URI: " + node_api)

                # node = ServerProxy(node_api)
                # pid = rosnode._succeed(node.getPid(ID))
                # print ("    PID    : {}".format(pid))

            self.hololens_ros_bridge = hololens_ros_bridge

            if (self.fnSendMessage is not None):
                msg = {
                    "msgType": "status", 
                    "status":
                        {
                            "pepper_robot" : pepper_robot,
                            "hololens_ros_bridge": hololens_ros_bridge,
                            "anchor_localizer": anchor_localizer,
                            "static_calibration": static_calibration,
                            "dynamic_adjuster": dynamic_adjuster,
                            "localizer": localizer,

                            "hololens_sequence": self.hololens_sequence,
                        }
                }

                self.fnSendMessage(msg)

            sleep(1.0)

    # -----------------------------------------------------------------------------
    #
    def cbFloorNormal(self, msg):
        self.last_floor_normal = msg

        #msg.header.seq
        #msg.header.stamp.secs
        #msg.pos.position.x
        #msg.pos.position.y
        #msg.pos.position.z

        try:
            sequence = msg.header.seq
        except Exception as e:
            #if (e.message):
            #    print("Error: '" + e.message)
            #else:
            #    print("Error: " + e.strerror)
            sequence = None

        self.hololens_sequence = sequence

        # print("floor_normal: sequence=" + str(sequence) + "")
        # pprint(msg)

    # -----------------------------------------------------------------------------
    #
    def StartNode(self, name):
        pass

    # -----------------------------------------------------------------------------
    #
    def CalibrateHoloLens(self):
        # initiate auto hololens calibration
        rospy.wait_for_service('hololens_auto_calibration')
        hololens_auto_calibration = rospy.ServiceProxy('hololens_auto_calibration', HololensAutoCalibration)
        try:
            response = hololens_auto_calibration()
        except rospy.ServiceException as exc:
            print("HololensAutoCalibration service did not process request: " + str(exc))

# -----------------------------------------------------------------------------
#
if __name__=="__main__":
    rospy.init_node('holo_nav_dash')

    # initialize global variables
    settings.initialize()

    port = rospy.get_param("~hnd_port", 8000)

    # create main application object, then run it
    settings.application = application(port)
    settings.application.run()

    # settings = termios.tcgetattr(sys.stdin)
    # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
