#!/usr/bin/env python

import time, sys, threading, math
import copy
import datetime
import socket, select
import struct
import traceback, code
import optparse
import socketserver
from pathlib import Path

import numpy as np

#from dynamic_reconfigure.server import Server
#from ur_driver.cfg import URDriverConfig

from deserialize import RobotState, RobotMode
from deserializeRT import RobotStateRT
import RobotRaconteur as RR
#Convenience shorthand to the default node.
#RRN is equivalent to RR.RobotRaconteurNode.s
RRN=RR.RobotRaconteurNode.s

#from ur_msgs.srv import SetPayload, SetIO
#from ur_msgs.msg import *

# renaming classes
"""DigitalIn = Digital
DigitalOut = Digital
Flag  = Digital
"""
prevent_programming = False

# Joint offsets, pulled from calibration information stored in the URDF
#
# { "joint_name" : offset }
#
# q_actual = q_from_driver + offset
joint_offsets = {}

PORT=30002       # 10 Hz, RobotState 
RT_PORT=30003    #125 Hz, RobotStateRT
DEFAULT_REVERSE_PORT = 50001     #125 Hz, custom data (from prog)

MSG_OUT = 1
MSG_QUIT = 2
MSG_JOINT_STATES = 3
MSG_MOVEJ = 4
MSG_WAYPOINT_FINISHED = 5
MSG_STOPJ = 6
MSG_SERVOJ = 7
MSG_SET_PAYLOAD = 8
MSG_WRENCH = 9
MSG_SET_DIGITAL_OUT = 10
MSG_GET_IO = 11
MSG_SET_FLAG = 12
MSG_SET_TOOL_VOLTAGE = 13
MSG_SET_ANALOG_OUT = 14
MULT_payload = 1000.0
MULT_wrench = 10000.0
MULT_jointstate = 10000.0
MULT_time = 1000000.0
MULT_blend = 1000.0
MULT_analog = 1000000.0
MULT_analog_robotstate = 0.1

#Max Velocity accepted by ur_driver
MAX_VELOCITY = 10.0
#Using a very high value in order to not limit execution of trajectories being sent from MoveIt!

#Bounds for SetPayload service
MIN_PAYLOAD = 0.0
MAX_PAYLOAD = 1.0
#Using a very conservative value as it should be set throught the parameter server


IO_SLEEP_TIME = 0.05

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

Q1 = [2.2,0,-1.57,0,0,0]
Q2 = [1.5,0,-1.57,0,0,0]
Q3 = [1.5,-0.2,-1.57,0,0,0]
  
pub_state=None
connected_robot = None
connected_robot_lock = threading.Lock()
connected_robot_cond = threading.Condition(connected_robot_lock)
last_joint_states = None
update=0
last_joint_state_time=0
last_joint_states_lock = threading.Lock()
"""
pub_joint_states = rospy.Publisher('joint_states', JointState, queue_size=1)
pub_wrench = rospy.Publisher('wrench', WrenchStamped, queue_size=1)
pub_io_states = rospy.Publisher('io_states', IOStates, queue_size=1)
"""
#dump_state = open('dump_state', 'wb')

class EOF(Exception): pass

def dumpstacks():
    id2name = dict([(th.ident, th.name) for th in threading.enumerate()])
    code = []
    for threadId, stack in sys._current_frames().items():
        code.append("\n# Thread: %s(%d)" % (id2name.get(threadId,""), threadId))
        for filename, lineno, name, line in traceback.extract_stack(stack):
            code.append('File: "%s", line %d, in %s' % (filename, lineno, name))
            if line:
                code.append("  %s" % (line.strip()))
    #print "\n".join(code)

def log(s):
    print("[%s] %s" % (datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'), s))


#RESET_PROGRAM = '''def resetProg():
#  sleep(0.0)
#end
#'''
Reset="def resetProg():"+"\n"+"sleep(0.0)"+"\n"+"end"+"\n"
RESET_PROGRAM = b = bytes(Reset, 'utf-8')

#RESET_PROGRAM = ''
    
class URConnection(object):
    TIMEOUT = 1.0
    
    DISCONNECTED = 0
    CONNECTED = 1
    READY_TO_PROGRAM = 2
    EXECUTING = 3
    
    def __init__(self, hostname, port, program):
        self.__thread = None
        self.__sock = None
        self.robot_state = self.DISCONNECTED
        self.hostname = hostname
        self.port = port
        self.program = program
        self.last_state = None

    def connect(self):
        if self.__sock:
            self.disconnect()
        self.__buf = b''
        self.robot_state = self.CONNECTED
        self.__sock = socket.create_connection((self.hostname, self.port))
        self.__keep_running = True
        self.__thread = threading.Thread(name="URConnection", target=self.__run)
        self.__thread.daemon = True
        self.__thread.start()

    def send_program(self):
        global prevent_programming
        if prevent_programming:
            #rospy.loginfo("Programming is currently prevented")
            return
        assert self.robot_state in [self.READY_TO_PROGRAM, self.EXECUTING]
        #rospy.loginfo("Programming the robot at %s" % self.hostname)
        self.__sock.sendall(self.program)
        self.robot_state = self.EXECUTING

    def send_reset_program(self):
        self.__sock.sendall(RESET_PROGRAM)
        self.robot_state = self.READY_TO_PROGRAM
        
    def disconnect(self):
        if self.__thread:
            self.__keep_running = False
            self.__thread.join()
            self.__thread = None
        if self.__sock:
            self.__sock.close()
            self.__sock = None
        self.last_state = None
        self.robot_state = self.DISCONNECTED

    def ready_to_program(self):
        return self.robot_state in [self.READY_TO_PROGRAM, self.EXECUTING]

    def __trigger_disconnected(self):
        log("Robot disconnected")
        self.robot_state = self.DISCONNECTED
    def __trigger_ready_to_program(self):
        #rospy.loginfo("Robot ready to program")
        pass
    def __trigger_halted(self):
        log("Halted")

    def __on_packet(self, buf):
        state = RobotState.unpack(buf)
        self.last_state = state
        #import deserialize; deserialize.pstate(self.last_state)

        #log("Packet.  Mode=%s" % state.robot_mode_data.robot_mode)

        if not state.robot_mode_data.real_robot_enabled:
            #rospy.logfatal("Real robot is no longer enabled.  Driver is fuxored")
            time.sleep(2)
            sys.exit(1)
        
        ###
        # IO-Support is EXPERIMENTAL
        # 
        # Notes: 
        # - Where are the flags coming from? Do we need flags? No, as 'prog' does not use them and other scripts are not running!
        # - analog_input2 and analog_input3 are within ToolData
        # - What to do with the different analog_input/output_range/domain?
        # - Shall we have appropriate ur_msgs definitions in order to reflect MasterboardData, ToolData,...?
        ###
        
        # Use information from the robot state packet to publish IOStates 
        """        
        msg = IOStates()
        #gets digital in states
        for i in range(0, 10):
            msg.digital_in_states.append(DigitalIn(i, (state.masterboard_data.digital_input_bits & (1<<i))>>i))
        #gets digital out states
        for i in range(0, 10):
            msg.digital_out_states.append(DigitalOut(i, (state.masterboard_data.digital_output_bits & (1<<i))>>i))
        #gets analog_in[0] state
        inp = state.masterboard_data.analog_input0 / MULT_analog_robotstate
        msg.analog_in_states.append(Analog(0, inp))
        #gets analog_in[1] state
        inp = state.masterboard_data.analog_input1 / MULT_analog_robotstate
        msg.analog_in_states.append(Analog(1, inp))      
        #gets analog_out[0] state
        inp = state.masterboard_data.analog_output0 / MULT_analog_robotstate
        msg.analog_out_states.append(Analog(0, inp))     
        #gets analog_out[1] state
        inp = state.masterboard_data.analog_output1 / MULT_analog_robotstate
        msg.analog_out_states.append(Analog(1, inp))     
        #print "Publish IO-Data from robot state data"
        pub_io_states.publish(msg)
        """

        # Updates the state machine that determines whether we can program the robot.
        can_execute = (state.robot_mode_data.robot_mode in [RobotMode.READY, RobotMode.RUNNING])
        if self.robot_state == self.CONNECTED:
            if can_execute:
                self.__trigger_ready_to_program()
                self.robot_state = self.READY_TO_PROGRAM
        elif self.robot_state == self.READY_TO_PROGRAM:
            if not can_execute:
                self.robot_state = self.CONNECTED
        elif self.robot_state == self.EXECUTING:
            if not can_execute:
                self.__trigger_halted()
                self.robot_state = self.CONNECTED

        # Report on any unknown packet types that were received
        if len(state.unknown_ptypes) > 0:
            state.unknown_ptypes.sort()
            s_unknown_ptypes = [str(ptype) for ptype in state.unknown_ptypes]
            self.throttle_warn_unknown(1.0, "Ignoring unknown pkt type(s): %s. "
                          "Please report." % ", ".join(s_unknown_ptypes))

    def throttle_warn_unknown(self, period, msg):
        self.__dict__.setdefault('_last_hit', 0.0)
        # this only works for a single caller
        #if (self._last_hit + period) <= rospy.get_time():
        #    self._last_hit = rospy.get_time()
        #    rospy.logwarn(msg)

    def __run(self):
        while self.__keep_running:
            r, _, _ = select.select([self.__sock], [], [], self.TIMEOUT)
            if r:
                more = self.__sock.recv(4096)
                if more:
                    self.__buf = self.__buf + more

                    #unpack_from requires a buffer of at least 48 bytes
                    while len(self.__buf) >= 48:
                        # Attempts to extract a packet
                        packet_length, ptype = struct.unpack_from("!IB", self.__buf)
                        #print("PacketLength: ", packet_length, "; BufferSize: ", len(self.__buf))
                        if len(self.__buf) >= packet_length:
                            packet, self.__buf = self.__buf[:packet_length], self.__buf[packet_length:]
                            self.__on_packet(packet)
                        else:
                            break

                else:
                    self.__trigger_disconnected()
                    self.__keep_running = False
                    
            else:
                self.__trigger_disconnected()
                self.__keep_running = False
                
                
class URConnectionRT(object):
    TIMEOUT = 1.0
    
    DISCONNECTED = 0
    CONNECTED = 1
    
    def __init__(self, hostname, port):
        self.__thread = None
        self.__sock = None
        self.robot_state = self.DISCONNECTED
        self.hostname = hostname
        self.port = port
        self.last_stateRT = None

    def connect(self):
        if self.__sock:
            self.disconnect()
        self.__buf = b""
        self.robot_state = self.CONNECTED
        self.__sock = socket.create_connection((self.hostname, self.port))
        self.__keep_running = True
        self.__thread = threading.Thread(name="URConnectionRT", target=self.__run)
        self.__thread.daemon = True
        self.__thread.start()
        
    def disconnect(self):
        if self.__thread:
            self.__keep_running = False
            self.__thread.join()
            self.__thread = None
        if self.__sock:
            self.__sock.close()
            self.__sock = None
        self.last_state = None
        self.robot_state = self.DISCONNECTED

    def __trigger_disconnected(self):
        log("Robot disconnected")
        self.robot_state = self.DISCONNECTED

    def __on_packet(self, buf):
        global last_joint_states, last_joint_states_lock,update,last_joint_state_time, pub_state
        #now = rospy.get_rostime()
        stateRT = RobotStateRT.unpack(buf)
        self.last_stateRT = stateRT
        
        #strt=RRN.NewStructure('robot.universalrobotics.JointData')
        #strt.ID=update
        update+=1
        
        #msg.header.stamp = now
        #msg.header.frame_id = "From real-time state data"
        #msg.name = joint_names
        #joint_values=[]
        #msg.position = [0.0] * 6
        #for i, q in enumerate(stateRT.q_actual):
        #    joint_values[i] = q #+ joint_offsets.get(joint_names[i], 0.0)
        
        #msg.effort = [0]*6
        #pub_joint_states.publish(msg)
        with last_joint_states_lock:
            state=RRN.NewStructure("com.robotraconteur.robotics.easy.EasyRobotState")
            state.seqno=update
            state.mode=1
            state.joint_position=np.asarray(stateRT.q_actual)
            state.joint_velocity=np.asarray(stateRT.qd_actual)
            state.joint_effort=np.zeros(6)
            state.position_command=np.asarray(stateRT.q_target)
            state.velocity_command=np.asarray(stateRT.qd_target)
            pub_state=state
            last_joint_states = strt
            
            #last_effort=stateRT.
            last_joint_state_time=time.time()
        """
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = now
        wrench_msg.wrench.force.x = stateRT.tcp_force[0]
        wrench_msg.wrench.force.y = stateRT.tcp_force[1]
        wrench_msg.wrench.force.z = stateRT.tcp_force[2]
        wrench_msg.wrench.torque.x = stateRT.tcp_force[3]
        wrench_msg.wrench.torque.y = stateRT.tcp_force[4]
        wrench_msg.wrench.torque.z = stateRT.tcp_force[5]
        pub_wrench.publish(wrench_msg)
        """

    def __run(self):
        while self.__keep_running:
            r, _, _ = select.select([self.__sock], [], [], self.TIMEOUT)
            if r:
                more = self.__sock.recv(4096)
                if more:
                    self.__buf = self.__buf + more
                    
                    #unpack_from requires a buffer of at least 48 bytes
                    while len(self.__buf) >= 48:
                        # Attempts to extract a packet
                        packet_length = struct.unpack_from("!i", self.__buf)[0]
                        #print("PacketLength: ", packet_length, "; BufferSize: ", len(self.__buf))
                        if len(self.__buf) >= packet_length:
                            packet, self.__buf = self.__buf[:packet_length], self.__buf[packet_length:]
                            self.__on_packet(packet)
                        else:
                            break
                else:
                    self.__trigger_disconnected()
                    self.__keep_running = False
                    
            else:
                self.__trigger_disconnected()
                self.__keep_running = False
                
                

class CommanderTCPHandler(socketserver.BaseRequestHandler):

    def recv_more(self):
        global last_joint_states, last_joint_states_lock,last_joint_state_time
        while True:
            r, _, _ = select.select([self.request], [], [], 0.2)
            if r:
                more = self.request.recv(4096)
                if not more:
                    raise EOF("EOF on recv")
                return more
            else:
                now = time.time()
                if last_joint_states and \
                        last_joint_state_time < now - 5:
                    print("Timeout Error")
                    #rospy.logerr("Stopped hearing from robot (last heard %.3f sec ago).  Disconnected" % \
                                    # (now - last_joint_states.header.stamp).to_sec())
                    raise EOF()

    def handle(self):
        self.__socket_lock = threading.Lock()
        setConnectedRobot(self)
        print("Handling a request")
        try:
            buf = self.recv_more()
            if not buf: return

            while True:
                #print "Buf:", [ord(b) for b in buf]

                # Unpacks the message type
                mtype = struct.unpack_from("!i", buf, 0)[0]
                buf = buf[4:]
                #print "Message type:", mtype

                if mtype == MSG_OUT:
                    # Unpacks string message, terminated by tilde
                    i = buf.find("~")
                    while i < 0:
                        buf = buf + self.recv_more()
                        i = buf.find("~")
                        if len(buf) > 2000:
                            raise Exception("Probably forgot to terminate a string: %s..." % buf[:150])
                    s, buf = buf[:i], buf[i+1:]
                    log("Out: %s" % s)
                
                elif mtype == MSG_QUIT:
                    print("Quitting")
                    raise EOF("Received quit")
                elif mtype == MSG_WAYPOINT_FINISHED:
                    while len(buf) < 4:
                        buf = buf + self.recv_more()
                    waypoint_id = struct.unpack_from("!i", buf, 0)[0]
                    buf = buf[4:]
                    print("Waypoint finished (not handled)")
                else:
                    raise Exception("Unknown message type: %i" % mtype)

                if not buf:
                    buf = buf + self.recv_more()
        except EOF:
            print("Connection closed (command):")
            setConnectedRobot(None)

    def __send_message(self, data):
        """
        Send a message to the robot.
        The message is given as a list of integers that will be packed
        as 4 bytes each in network byte order (big endian).
        A lock is acquired before sending the message to prevent race conditions.
        :param data: list of int, the data to send
        """
        buf = struct.pack("!%ii" % len(data), *data)
        with self.__socket_lock:
            self.request.send(buf)

    def send_quit(self):
        self.__send_message([MSG_QUIT])

    def send_servoj(self, waypoint_id, q_actual, t):
        assert(len(q_actual) == 6)
        q_robot = [0.0] * 6
        for i, q in enumerate(q_actual):
            q_robot[i] = q - joint_offsets.get(joint_names[i], 0.0)
        params = [MSG_SERVOJ, waypoint_id] + \
                 [MULT_jointstate * qq for qq in q_robot] + \
                 [MULT_time * t]
        self.__send_message(params)

    #Experimental set_payload implementation
    def send_payload(self,payload):
        self.__send_message([MSG_SET_PAYLOAD, payload * MULT_payload])

    #Experimental set_digital_output implementation
    def set_digital_out(self, pinnum, value):
        self.__send_message([MSG_SET_DIGITAL_OUT, pinnum, value])
        time.sleep(IO_SLEEP_TIME)

    def set_analog_out(self, pinnum, value):
        self.__send_message([MSG_SET_ANALOG_OUT, pinnum, value * MULT_analog])
        time.sleep(IO_SLEEP_TIME)

    def set_tool_voltage(self, value):
        self.__send_message([MSG_SET_TOOL_VOLTAGE, value, 0])
        time.sleep(IO_SLEEP_TIME)

    def set_flag(self, pin, val):
        self.__send_message([MSG_SET_FLAG, pin, val])
        #set_flag will fail if called too closely together--added delay
        time.sleep(IO_SLEEP_TIME)

    def send_stopj(self):
        self.__send_message([MSG_STOPJ])

    def set_waypoint_finished_cb(self, cb):
        self.waypoint_finished_cb = cb

    # Returns the last JointState message sent out
    def get_joint_states(self):
        global last_joint_states, last_joint_states_lock
        return last_joint_states
    

class TCPServer(socketserver.TCPServer):
    allow_reuse_address = True  # Allows the program to restart gracefully on crash
    timeout = 5

    
def setConnectedRobot(r):
    global connected_robot, connected_robot_lock
    with connected_robot_lock:
        connected_robot = r
        connected_robot_cond.notify()

def getConnectedRobot(wait=False, timeout=-1):
    started = time.time()
    with connected_robot_lock:
        if wait:
            while not connected_robot:
                if timeout >= 0 and time.time() > started + timeout:
                    break
                connected_robot_cond.wait(0.2)
        return connected_robot
        
        
        
        
class UR_Joint_Listener(object):
    #RATE = 0.02
    def __init__(self, robot, goal_time_tolerance=None):
        self.goal_time_tolerance = goal_time_tolerance or 0.0
        self.joint_goal_tolerances = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
        self.following_lock = threading.Lock()
        self.T0 = time.time()
        self.robot = robot
        self.trajectory_running=False
        self._current_trajectory=None
        #self.server = actionlib.ActionServer("follow_joint_trajectory",
        #                                    FollowJointTrajectoryAction,
        #                                   self.on_goal, self.on_cancel, auto_start=False)
        self.MODE_HALT=0
        self.MODE_POSITION = 3#;#1
        self.MODE_VELOCITY = 4#;#2
        self.MODE_TORQUE = 1#;#3
        self.MODE_TRAJECTORY = 2#;#4
        #self.goal_handle = None
        self.joint_command = None
        
        self.last_point_sent = True
        self._streaming=False
        
        #self._ep=0
        self._mode=self.MODE_HALT

        #self.update_timer = rospy.Timer(rospy.Duration(self.RATE), self._update)
        
    def StartRobot(self):
        with self.following_lock:
            if (self._streaming):
                raise Exception("Already streaming")
            #self._ep=RR.ServerEndpoint.GetCurrentEndpoint()
            self._streaming=True
            t=threading.Thread(target=self._send_thread)
            t.start()
            
    def StopRobot(self):
        if (not self._streaming):
            raise Exception("Not streaming")
        with self.following_lock:
            
            self._streaming=False
            
    def _send_thread(self):
        global state_pub
        try:
            while self._streaming:
                if (not self._streaming): return
                with self.following_lock:
                    self.easy_robot_state.OutValue=state_pub
                
        except:
            #Exception will be thrown when the port is closed
            #just ignore it
            if (self._streaming):

                traceback.print_exc()
            pass
    """
    def _ReadJoints(self):
        global last_joint_states_lock,last_joint_states
        with last_joint_states_lock:
            self.current_joints=last_joint_states
    """           

    def set_robot(self, robot):
        # Cancels any goals in progress
       # if self.goal_handle:
        #    self.goal_handle.set_canceled()
        #    self.goal_handle = None
        self.traj = None
        self.robot = robot
        if self.robot:
            self.init_traj_from_robot()

    # Sets the trajectory to remain stationary at the current position
    # of the robot.
    
    
    
    def init_traj_from_robot(self):
        if not self.robot: raise Exception("No robot connected")
        # Busy wait (avoids another mutex)
        state = self.robot.get_joint_states()
        while not state:
            time.sleep(0.1)
            state = self.robot.get_joint_states()
        self.traj_t0 = time.time()
        self.joint_command=state.joint_values
        #self.traj = JointTrajectory()
        #self.traj.joint_names = joint_names
        #self.traj.points = [JointTrajectoryPoint(
        #    positions = state.position,
        #    velocities = [0] * 6,
        #    accelerations = [0] * 6,
        #   time_from_start = rospy.Duration(0.0))]
    @property
    def command_mode(self):
        
        return self._mode

    @command_mode.setter
    def command_mode(self,value):
        print("Changing mode to: "+str(value))
        self._mode=value
        
    def execute_trajectory(self,trajectory):
        self.trajectory_running=True
        self._current_trajectory=trajectory
        return trajectory_generator(self)

        
    def jog_joint(self,joint_positions,joint_velocity):
        
        # Checks that the robot is connected
        if not self.robot:
            
            return
        
        # Orders the joints of the trajectory according to joint_names
        #reorder_traj_joints(goal_handle.get_goal().trajectory, joint_names)
                
        with self.following_lock:
            # Inserts the current setpoint at the head of the trajectory
            now = time.time()
            
            try:
                self.robot.send_servoj(999, joint_positions, 4 * self.RATE)
            except socket.error:
                pass
                
            position_in_tol = within_tolerance(state.position, last_point.positions, self.joint_goal_tolerances)
        print("Finished command and got to position=%r" %(position_in_tol))
            #point0 = sample_traj(self.traj, now - self.traj_t0)
            #point0.time_from_start = rospy.Duration(0.0)
            #goal_handle.get_goal().trajectory.points.insert(0, point0)
            #self.traj_t0 = now

            # Replaces the goal
            #self.goal_handle = goal_handle
            #self.traj = goal_handle.get_goal().trajectory
            #self.goal_handle.set_accepted()

    """def on_cancel(self, goal_handle):
        log("on_cancel")
        if goal_handle == self.goal_handle:
            with self.following_lock:
                # Uses the next little bit of trajectory to slow to a stop
                STOP_DURATION = 0.5
                now = time.time()
                point0 = sample_traj(self.traj, now - self.traj_t0)
                point0.time_from_start = rospy.Duration(0.0)
                point1 = sample_traj(self.traj, now - self.traj_t0 + STOP_DURATION)
                point1.velocities = [0] * 6
                point1.accelerations = [0] * 6
                point1.time_from_start = rospy.Duration(STOP_DURATION)
                self.traj_t0 = now
                self.traj = JointTrajectory()
                self.traj.joint_names = joint_names
                self.traj.points = [point0, point1]
                
                self.goal_handle.set_canceled()
                self.goal_handle = None
        else:
            goal_handle.set_canceled()
    """
    last_now = time.time()
    """
    def _update(self, event):
        if self.robot and self.traj:
            now = time.time()
            if (now - self.traj_t0) <= self.traj.points[-1].time_from_start.to_sec():
                self.last_point_sent = False #sending intermediate points
                setpoint = sample_traj(self.traj, now - self.traj_t0)
                try:
                    self.robot.send_servoj(999, setpoint.positions, 4 * self.RATE)
                except socket.error:
                    pass
                    
            elif not self.last_point_sent:
                # All intermediate points sent, sending last point to make sure we
                # reach the goal.
                # This should solve an issue where the robot does not reach the final
                # position and errors out due to not reaching the goal point.
                last_point = self.traj.points[-1]
                state = self.robot.get_joint_states()
                position_in_tol = within_tolerance(state.position, last_point.positions, self.joint_goal_tolerances)
                # Performing this check to try and catch our error condition.  We will always
                # send the last point just in case.
                if not position_in_tol:
                    rospy.logwarn("Trajectory time exceeded and current robot state not at goal, last point required")
                    rospy.logwarn("Current trajectory time: %s, last point time: %s" % \
                                (now - self.traj_t0, self.traj.points[-1].time_from_start.to_sec()))
                    rospy.logwarn("Desired: %s\nactual: %s\nvelocity: %s" % \
                                          (last_point.positions, state.position, state.velocity))
                setpoint = sample_traj(self.traj, self.traj.points[-1].time_from_start.to_sec())

                try:
                    self.robot.send_servoj(999, setpoint.positions, 4 * self.RATE)
                    self.last_point_sent = True
                except socket.error:
                    pass
                    
            else:  # Off the end
                if self.goal_handle:
                    last_point = self.traj.points[-1]
                    state = self.robot.get_joint_states()
                    position_in_tol = within_tolerance(state.position, last_point.positions, [0.1]*6)
                    velocity_in_tol = within_tolerance(state.velocity, last_point.velocities, [0.05]*6)
                    if position_in_tol and velocity_in_tol:
                        # The arm reached the goal (and isn't moving).  Succeeding
                        self.goal_handle.set_succeeded()
                        self.goal_handle = None
                    #elif now - (self.traj_t0 + last_point.time_from_start.to_sec()) > self.goal_time_tolerance.to_sec():
                    #    # Took too long to reach the goal.  Aborting
                    #    rospy.logwarn("Took too long to reach the goal.\nDesired: %s\nactual: %s\nvelocity: %s" % \
                    #                      (last_point.positions, state.position, state.velocity))
                    #    self.goal_handle.set_aborted(text="Took too long to reach the goal")
                    #    self.goal_handle = None
                    
        """
        
class trajectory_generator(object):
    def __init__(self,robot_object):
        self._j=0
        self._closed=False
        self.robot_object=robot_object
        self._aborted=False
        self.duration_from_start=0
        #self._goal = FollowJointTrajectoryGoal()
        #joint_names=[]
        #for i in self.robot_object._current_trajectory.joint_names:
        #    joint_names.append(str(i))
        #self._goal.trajectory.joint_names=joint_names
        #self._goal_time_tolerance = rospy.Time(0.1)
        #self._goal.goal_time_tolerance = self._goal_time_tolerance
        

    def Next(self): #add joints next
        
        trajectory_status=RRN.NewStructure("com.robotraconteur.robotics.trajectory.TrajectoryStatus")
        if self._aborted:
            self.robot_object.trajectory_running=False
            self.robot_object._current_trajectory=None
            trajectory_status.status= -1
            raise OperationAbortedException()
        #check if number of items = joint number and error
        elif self._closed:
            
            self.robot_object.trajectory_running=False
            self.robot_object._current_trajectory=None
            trajectory_status.status=3
            raise StopIterationException()
        elif self._j>=(len(self.robot_object._current_trajectory.waypoints)):
            trajectory_status.status=3
            #self._goal.trajectory.header.stamp = rospy.Time.now()
            #print(self._goal)
            #result=self.robot_object.trajectory_client.send_goal(self._goal)
            
            #self.robot_object.trajectory_client.wait_for_result(rospy.Duration(30.0))
            #print(self.robot_object.trajectory_client.get_result())
            #TODO: use global variable update as seqno, but messy with threading
            trajectory_status.seqno=self._j
            trajectory_status.current_waypoint=self._j
            self.duration_from_start=(self.robot_object.RATE)*self._j
            trajectory_status.trajectory_time=self.duration_from_start
            print("sending and finishing")
            self._j=0
            raise StopIterationException()
        else:
            trajectory_status.status=2
            print("continuing")
        waypoint=self.robot_object._current_trajectory.waypoints[self._j]
        print(self._j)
        print(waypoint.joint_position)

        #print(len(self.robot_object._current_trajectory.waypoints)-1)
        #point = JointTrajectoryPoint()
        #point.positions = list(waypoint.joint_position)
        #point.time_from_start = rospy.Duration(waypoint.time_from_start)
        
        #self._goal.trajectory.points.append(point)
        self.robot_object.jog_joint(waypoint.joint_position,waypoint.joint_velocity)
        time.sleep(1)
        #if (self._j>=8):
        #    raise StopIterationException()
        
        #a = copy.copy(v)
        #for i in xrange(len(a)):
        #    a[i]+=self._j
        
        trajectory_status.seqno=self._j
        trajectory_status.current_waypoint=self._j
        self.duration_from_start=(self.robot_object.RATE)*self._j
        trajectory_status.trajectory_time=self.duration_from_start
        
        self._j+=1
        
        return trajectory_status
        
    def Abort(self):
        self._aborted=True
        
    def Close(self):
        self._closed=True
                    
def joinAll(threads):
    while any(t.isAlive() for t in threads):
        for t in threads:
            t.join(0.2)
            
def within_tolerance(a_vec, b_vec, tol_vec):
    for a, b, tol in zip(a_vec, b_vec, tol_vec):
        if abs(a - b) > tol:
            return False
    return True
                    
def get_my_ip(robot_ip, port):
    s = socket.create_connection((robot_ip, port))
    tmp = s.getsockname()[0]
    s.close()
    return tmp
                    
def main():
    
    
    
    
    
    global joint_names
    joint_names = [name for name in JOINT_NAMES]
    reverse_port = DEFAULT_REVERSE_PORT
    # Parses command line arguments
    """
    parser = optparse.OptionParser(usage="usage: %prog robot_hostname [reverse_port]")
    (options, args) = parser.parse_args(rospy.myargv()[1:])
    if len(args) < 1:
        parser.error("You must specify the robot hostname")
    elif len(args) == 1:
        robot_hostname = args[0]
        
    elif len(args) == 2:
        robot_hostname = args[0]
        reverse_port = int(args[1])
        if not (0 <= reverse_port <= 65535):
                parser.error("You entered an invalid port number")
    else:
        parser.error("Wrong number of parameters")
    """
    # Reads the calibrated joint offsets from the URDF
    #global joint_offsets
    #joint_offsets = load_joint_offsets(joint_names)
    """
    if len(joint_offsets) > 0:
        rospy.loginfo("Loaded calibration offsets from urdf: %s" % joint_offsets)
    else:
        rospy.loginfo("No calibration offsets loaded from urdf")
    """
    # Reads the maximum velocity
    # The max_velocity parameter is only used for debugging in the ur_driver. It's not related to actual velocity limits
    global max_velocity
    #max_velocity = rospy.get_param("~max_velocity", MAX_VELOCITY) # [rad/s] 
    #rospy.loginfo("Max velocity accepted by ur_driver: %s [rad/s]" % max_velocity)
    
    # Reads the minimum payload
    global min_payload
    #min_payload = rospy.get_param("~min_payload", MIN_PAYLOAD)
    # Reads the maximum payload
    global max_payload
    #max_payload = rospy.get_param("~max_payload", MAX_PAYLOAD)
    #rospy.loginfo("Bounds for Payload: [%s, %s]" % (min_payload, max_payload))
    

    # Sets up the server for the robot to connect to
    server = TCPServer(("", reverse_port), CommanderTCPHandler)
    thread_commander = threading.Thread(name="CommanderHandler", target=server.serve_forever)
    thread_commander.daemon = True
    thread_commander.start()

    
    #with open(roslib.packages.get_pkg_dir('ur_driver') + '/prog') as fin:
    pather=Path()
    path=pather.absolute()
    #robot_hostname="ur-2013216004"
    robot_hostname="128.113.224.7"
    with open(str(path)+'/prog.txt') as fin:
        programstring = fin.read() % {"driver_hostname": get_my_ip(robot_hostname, PORT), "driver_reverseport": reverse_port}
    program=bytes(programstring,'utf-8')
    nodename="URConnection"
    with RR.ServerNodeSetup(nodename,2355):
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.geometry")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.uuid")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.datetime")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.identifier")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.sensordata")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.resource")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.device")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.units")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.robotics.joints")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.robotics.trajectory")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.datatype")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.signal")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.param")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.robotics.tool")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.robotics.payload")
        RRN.RegisterServiceTypeFromFile("com.robotraconteur.robotics.robot")
        connection = URConnection(robot_hostname, PORT, program)
        connection.connect()
        connection.send_reset_program()
        
        connectionRT = URConnectionRT(robot_hostname, RT_PORT)
        connectionRT.connect()
        
        service_provider=None
       
        
        try:
            
            # Checks for disconnect
            if getConnectedRobot(wait=False):
                time.sleep(0.2)
                print("Got connected robot?")
                if prevent_programming:
                    print("Programming now prevented")
                    connection.send_reset_program()
            else:
                print("Disconnected.  Reconnecting")               

                  

                #rospy.loginfo("Programming the robot")
                while True:
                    # Sends the program to the robot
                    while not connection.ready_to_program():
                        print("Waiting to program")
                        time.sleep(1.0)
                    
                    connection.send_program()

                    r = getConnectedRobot(wait=True, timeout=1.0)
                    if r:
                        break
                #rospy.loginfo("Robot connected")

                #provider for service calls

                   
                    
                action_server = UR_Joint_Listener(r, 1.0)
                
                
                RRN.RegisterService("Universal_Robot",
                                      "com.robotraconteur.robotics.robot.Robot",
                                                  action_server)
    
                #Register the service type and the service
                #RRN.RegisterServiceTypeFromFile("universal_robotics")
                #RRN.RegisterService("Universal_Robot","robot.universalrobotics.Universal_Robot",action_server)
                #action_server.start()

        except KeyboardInterrupt:
            try:
                r = getConnectedRobot(wait=False)
                #rospy.signal_shutdown("KeyboardInterrupt")
                if r: r.send_quit()
            except:
                pass
            raise
    

if __name__ == '__main__': main()