
# Copyright 2015-2020 - RoboDK Inc. - https://robodk.com/
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ----------------------------------------------------
# This file is a POST PROCESSOR for Robot Offline Programming to generate programs 
# for a Universal Robot with RoboDK
#
# To edit/test this POST PROCESSOR script file:
# Select "Program"->"Add/Edit Post Processor", then select your post or create a new one.
# You can edit this file using any text editor or Python editor. Using a Python editor allows to quickly evaluate a sample program at the end of this file.
# Python should be automatically installed with RoboDK
#
# You can also edit the POST PROCESSOR manually:
#    1- Open the *.py file with Python IDLE (right click -> Edit with IDLE)
#    2- Make the necessary changes
#    3- Run the file to open Python Shell: Run -> Run module (F5 by default)
#    4- The "test_post()" function is called automatically
# Alternatively, you can edit this file using a text editor and run it with Python
#
# To use a POST PROCESSOR file you must place the *.py file in "C:/RoboDK/Posts/"
# To select one POST PROCESSOR for your robot in RoboDK you must follow these steps:
#    1- Open the robot panel (double click a robot)
#    2- Select "Parameters"
#    3- Select "Unlock advanced options"
#    4- Select your post as the file name in the "Robot brand" box
#
# To delete an existing POST PROCESSOR script, simply delete this file (.py file)
#
# ----------------------------------------------------
# More information about RoboDK Post Processors and Offline Programming here:
#     https://robodk.com/help#PostProcessor
#     https://robodk.com/doc/en/PythonAPI/postprocessor.html
# ----------------------------------------------------

DEFAULT_HEADER_SCRIPT =  """"""

def get_safe_name(progname):
    """Get a safe program name"""
    for c in r'-[]/\;,><&*:%=+@!#^|?^':
        progname = progname.replace(c,'')
    if len(progname) <= 0:
        progname = 'Program'
    if progname[0].isdigit():
        progname = 'P' + progname    
    return progname

# ----------------------------------------------------
# Import RoboDK tools
from robodk import *

# ----------------------------------------------------
import socket
import struct
# UR information for real time control and monitoring
# Byte shifts for the real time packet:
UR_GET_RUNTIME_MODE = 132*8-4

RUNTIME_CANCELLED = 0
RUNTIME_READY = 1
RUNTIME_BUSY = 2

RUNTIME_MODE_MSG = []
RUNTIME_MODE_MSG.append("Operation cancelled") #0
RUNTIME_MODE_MSG.append("Ready") #1
RUNTIME_MODE_MSG.append("Running") #2 # Running or Jogging

ROBOT_PROGRAM_ERROR = -1
ROBOT_NOT_CONNECTED = 0
ROBOT_OK = 1

def Pose_2_TxyzRxyzTEST2(H):
    """Retrieve the position (mm) and Euler angles (rad) as an array [x,y,z,rx,ry,rz] given a pose.
    It returns the values that correspond to the following operation:
    H = transl(x,y,z)*rotx(rx)*roty(ry)*rotz(rz).

    :param H: pose
    :type H: :class:`.Mat`

    .. seealso:: :class:`.Mat`, :func:`~robodk.TxyzRxyz_2_Pose`, :func:`~robodk.Pose_2_TxyzRxyz`, :func:`~robodk.Pose_2_ABB`, :func:`~robodk.Pose_2_Adept`, :func:`~robodk.Pose_2_Comau`, :func:`~robodk.Pose_2_Fanuc`, :func:`~robodk.Pose_2_KUKA`, :func:`~robodk.Pose_2_Motoman`, :func:`~robodk.Pose_2_Nachi`, :func:`~robodk.Pose_2_Staubli`, :func:`~robodk.Pose_2_UR`, :func:`~robodk.quaternion_2_pose`
    """
    x = H[0, 3]
    y = H[1, 3]
    z = H[2, 3]
    a = H[0, 0]
    b = H[0, 1]
    c = H[0, 2]
    d = H[1, 2]
    e = H[2, 2]
    f = H[2, 0]
    g = H[2, 1]

    """
    a = r*math.pi/180.0
    b = p*math.pi/180.0
    c = w*math.pi/180.0
    ca = math.cos(a)
    sa = math.sin(a)
    cb = math.cos(b)
    sb = math.sin(b)
    cc = math.cos(c)
    sc = math.sin(c)
    return Mat([[cb*ca,ca*sc*sb-cc*sa,sc*sa+cc*ca*sb,x],[cb*sa,cc*ca+sc*sb*sa,cc*sb*sa-ca*sc,y],[-sb,cb*sc,cc*cb,z],[0.0,0.0,0.0,1.0]])
    """

    rz1= atan2(H[1,0],H[0,0])
    ry1 = atan2(-H[2,0],sqrt(H[2,1]**2 + H[2,2]**2))
    rx1 = atan2(H[2,1], H[2,2])


    return [x, y, z, rx1, ry1, rz1]


def pose_2_str(pose):
    """Prints a pose target"""
    [x,y,z,r,p,w] = Pose_2_TxyzRxyzTEST2(pose)
    return ('coordinate={%.3f, %.3f, %.3f, %.3f, %.3f, %.3f}' % (x,y,z,r*180.0/pi,p*180.0/pi,w*180.0/pi))

def joint_2_str(joint):
    """Print a joint target"""
    return ('{joint={%.6f, %.6f, %.6f, %.6f, %.6f, %.6f}}' % (joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]))

def circle_radius(p0,p1,p2):
    a = norm(subs3(p0,p1))
    b = norm(subs3(p1,p2))
    c = norm(subs3(p2,p0))
    radius = a*b*c/sqrt(pow(a*a+b*b+c*c,2)-2*(pow(a,4)+pow(b,4)+pow(c,4)))            
    return radius
    
#def distance_p1_p02(p0,p1,p2):
#    v01 = subs3(p1, p0)
#    v02 = subs3(p2, p0)
#    return dot(v02,v01)/dot(v02,v02)
        
# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object"""   
    
    # default speed for linear moves in %
    SPEED_PRCENT      = 50     
    
    # default acceleration for lineaer moves in %
    ACCEL_PRCENT      = 50    
    
    # default speed for joint moves in %
    SPEED_DEG_PRCENT      = 50     
    
    # default acceleration for joint moves in %
    ACCEL_DEG_PRCENT      = 50    

    # default blend radius in mm (corners smoothing)
    ROUNDING = 1
    
    # 5000    # Maximum number of lines per program. If the number of lines is exceeded, the program will be executed step by step by RoboDK
    MAX_LINES_X_PROG = 1e9  
    
    # Maximum speeds and accelerations allowed by the controller
    MAX_SPEED_MMS = 2000
    MAX_SPEED_DEGS = 180
    MAX_ACCEL_MMSS = 10000
    MAX_ACCEL_DEGSS = 2300
    
    #--------------------------------
    PROG_EXT = '.lua'        # set the program extension
    REF_FRAME      = eye(4) # default reference frame (the robot reference frame)
    LAST_POS_ABS = None # last XYZ position
    TOOL_FRAME = eye(4)
    
    # Remember last direction
    LAST_DIR = None
    
    # other variables
    ROBOT_POST = 'unset'
    ROBOT_NAME = 'generic'
    PROG_FILES = []
    MAIN_PROGNAME = 'unknown'
    
    nPROGS = 0
    PROG = []
    PROG_LIST = []
    VARS = []
    VARS_LIST = []
    SUBPROG = []
    FRAME_LIST = []
    TOOL_LIST = []
    TAB = ''
    LOG = ''    

    JOINT_TARGET = []
    POSE_TARGET = []

    FRAME_COUNT = 0
    TOOL_COUNT = 0

    NUM_TARGETJ = 0
    NUM_TARGETP = 0

    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        
        for k,v in kwargs.items():
            if k == 'lines_x_prog':
                self.MAX_LINES_X_PROG = v
        
    def ProgStart(self, progname):
        progname = FilterName(progname)
        self.nPROGS = self.nPROGS + 1
         
        
    def ProgFinish(self, progname):
        progname = FilterName(progname)     
                
    def ProgSave(self, folder, progname, ask_user = False, show_result = False):
        progname = FilterName(progname)
        progname = progname + self.PROG_EXT
        if ask_user or not DirExists(folder):
            filesave = getSaveFile(folder, progname, 'Save program as...')
            if filesave is not None:
                filesave = filesave.name
            else:
                return
        else:
            filesave = folder + '/' + progname

        self.prog_2_list()        

        fid = open(filesave, "w")
        # Create main program call:
        #fid.write('def %s():\n' % self.MAIN_PROGNAME)

        # Add global parameters:
        #fid.write('  # Global parameters:\n')
        for line in self.VARS_LIST[0]:
            fid.write(line+ '\n')            
        #fid.write('  \n')
        #fid.write('  ')

        # Add a custom header if desired:
        fid.write(DEFAULT_HEADER_SCRIPT)        
        #fid.write('  \n')

        #Add the joint target
        for line in self.JOINT_TARGET:
            fid.write(line + '\n')

        #Add the pose target
        for line in self.POSE_TARGET:
            fid.write(line + '\n')

        # Add the suprograms that are being used in RoboDK
        for line in self.SUBPROG:
            fid.write(line + '\n')            
        #fid.write('  \n')

        # Add the main code:
        #fid.write('  # Main program:\n')
        for prog in self.PROG_LIST:
            for line in prog:
                fid.write(line + '\n')

        #fid.write('end\n\n')
        #fid.write('%s()\n' % self.MAIN_PROGNAME)
            
        fid.close()
    
        print('SAVED: %s\n' % filesave) # tell RoboDK the path of the saved file
        self.PROG_FILES = filesave
        
        
        #------------------------------------------------------------------------------------------            
        
        # open file with default application
        if show_result:
            if type(show_result) is str:
                # Open file with provided application
                import subprocess
                p = subprocess.Popen([show_result, filesave])
            elif type(show_result) is list:
                import subprocess
                p = subprocess.Popen(show_result + [filesave])   
            else:
                # open file with default application
                os.startfile(filesave)
            if len(self.LOG) > 0:
                mbox('Program generation LOG:\n\n' + self.LOG)
    
        #if len(self.PROG_LIST) > 1:
        #    mbox("Warning! The program " + progname + " is too long and directly running it on the robot controller might be slow. It is better to run it form RoboDK.")


    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        """Send a program to the robot using the provided parameters. This method is executed right after ProgSave if we selected the option "Send Program to Robot".
        The connection parameters must be provided in the robot connection menu of RoboDK"""
        #UploadFTP(self.PROG_FILES, robot_ip, remote_path, ftp_user, ftp_pass)
        #return        

        
    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""

        #add joint target
        target = self.addtargetjoint(joints)

        self.addline('MoveJ(%s)' % target)
        
    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""

        #add pose target
        target = self.addtargetpose(pose)

        self.addline('Move(%s)' % target)
        
        
    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""      
        targetM = self.addtargetpose(pose1)
        targetE = self.addtargetpose(pose2)
        
        self.addline('Arc3(%s,%s)' % (targetM,targetE))
        
    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        self.REF_FRAME = pose
        if pose == eye(4):
            self.FRAME_ID = 0
        elif frame_id < 0:
            if frame_name in self.FRAME_LIST:
                id_list = self.FRAME_LIST.index(frame_name)
                self.FRAME_ID = self.FRAME_LIST[id_list,1]
            else:
                self.FRAME_ID = self.FRAME_COUNT + 10
                new_frame = [frame_name,self.FRAME_ID]
                self.FRAME_LIST.append(new_frame)
                self.FRAME_COUNT += 1
        else:
            self.FRAME_ID = frame_id

        
    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        self.TOOL_FRAME = pose
        if tool_id < 0:
            if tool_name in self.TOOL_LIST:
                id_list = self.TOOL_LIST.index(tool_name)
                self.TOOL_ID = self.TOOL_LIST[id_list,1]
            else:
                self.TOOL_ID = self.TOOL_COUNT + 10
                self.TOOL_LIST.append(tool_name)
                self.TOOL_COUNT += 1
        else:
            self.TOOL_ID = tool_id
        
    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms <= 0:
            self.addline('halt() # reimplement this function to force stop')
        else:
            self.addline('Wait(%d)' % round(time_ms))
        
    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.SPEED_PRCENT = max(1,min(speed_mms/self.MAX_SPEED_MMS*100,100))
        self.addline('SpeedS(%i)' % self.SPEED_PRCENT)
        
    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""
        self.ACCEL_PRCENT = max(1,min(accel_mmss/self.MAX_ACCEL_MMSS*100,100))    
        self.addline('AccelS(%i)' % self.ACCEL_PRCENT)
        
    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.SPEED_DEG_PRCENT = max(1,min(speed_degs/self.MAX_SPEED_DEGS*100,100))
        self.addline('Speed(%i)' % self.SPEED_DEG_PRCENT)
    
    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.ACCEL_DEG_PRCENT = max(1,min(accel_degss/self.MAX_ACCEL_DEGSS*100,100))    
        self.addline('Accel(%i)' % self.ACCEL_DEG_PRCENT)
        
    def setZoneData(self, zone_percentage):
        """Changes the zone data approach (makes the movement more smooth)"""
        if zone_percentage < 0:
            zone_percentage = 0            
        self.ROUNDING = min(100,zone_percentage)

        self.addline('CP(%i)' % self.ROUNDING)
        
    def setDO(self, io_var, io_value):
        """Set a Digital Output"""
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'ON'
            else:
                io_value = 'OFF'
        else:
            if io_value.lower() == 'true':
                io_value = 'ON'
            else:
                io_value = 'OFF'        

        newline = 'DO(' + str(io_var) + ',' + io_value +',"SYNC=1")'            
        self.addline(newline)
        
    def setAO(self, io_var, io_value):
        """Set an Analog Output"""

        newline = 'AO(' + str(io_var) + ',' + str(io_value) +',"SYNC=1")'             
        self.addline(newline)
        
    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for an input io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        newline = '-- WaitDI not implemented'
        self.addline(newline)
        #------------------------------------------------
        
    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        #self.make_last_move_accurate()
        
        if is_function_call:
            code = FilterName(code)
            """
            if code.lower() == "usemovel":
                self.USE_MOVEP = False
                return
            elif code.lower() == "usemovep":
                self.USE_MOVEP = True
                return
            """
            if not code.endswith(')'):
                code = code + '()'
            self.addline(code)
        else:
            if not '\n' in code:
                self.addline(code)
            else:
                for line in code.split('\n'):
                    self.addline(line)
        
    def RunMessage(self, message, iscomment = False):
        """Show a message on the controller screen"""
        if iscomment:
            self.addline('-- ' + message)
        else:
            self.addline('popup("%s","Message",False,False,blocking=False)' % message)
        
# ------------------ private ----------------------=            
    def prog_2_list(self):
        if len(self.PROG) > 1:
            self.PROG_LIST.append(self.PROG)
            self.PROG = []
            self.VARS_LIST.append(self.VARS)
            self.VARS = []
        
    def addline(self, newline):
        """Add a program line"""
        if self.nPROGS <= 1:
            if len(self.PROG) > self.MAX_LINES_X_PROG:
                self.prog_2_list()
                
            self.PROG.append(self.TAB + newline)
        else:
            self.SUBPROG.append(self.TAB + newline)

    def addtargetjoint(self, joint):
        """Add a target"""
        target_name = 'TargetJ' + str(self.NUM_TARGETJ)
        new_target = target_name + '=' + joint_2_str(joint)
        #local StarPoint = {joint={0,0,0,0,0,0}} 
        self.JOINT_TARGET.append(new_target)
        self.NUM_TARGETJ += 1
        return target_name

    def addtargetpose(self, pose):
        """Add a target"""
        target_name = 'TargetP' + str(self.NUM_TARGETP)
        new_target = target_name + '={armOrientation={-1, -1, -1, -1}, ' + pose_2_str(pose) + ', user=%s, load=0}' % (self.FRAME_ID)
        #P1={armOrientation={-1, -1, -1, -1}, coordinate={193.834396, 449.261292, 411.144806, -179.079605, 1.843200, -6.970400}, tool=0, user=0, load=0}
        self.POSE_TARGET.append(new_target)
        self.NUM_TARGETP += 1
        return target_name
        
    def addlog(self, newline):
        """Add a log message"""
        self.LOG = self.LOG + newline + '\n'

# -------------------------------------------------
# ------------ For testing purposes ---------------   
""" def Pose(xyzrpw):
    [x,y,z,r,p,w] = xyzrpw
    a = r*math.pi/180
    b = p*math.pi/180
    c = w*math.pi/180
    ca = math.cos(a)
    sa = math.sin(a)
    cb = math.cos(b)
    sb = math.sin(b)
    cc = math.cos(c)
    sc = math.sin(c)
    return Mat([[cb*ca, ca*sc*sb - cc*sa, sc*sa + cc*ca*sb, x],[cb*sa, cc*ca + sc*sb*sa, cc*sb*sa - ca*sc, y],[-sb, cb*sc, cc*cb, z],[0,0,0,1]])

def test_post():
    Test the post with a basic program

    robot = RobotPost('Universal Robotics', 'Generic UR robot')

    robot.ProgStart("Program")
    robot.RunMessage("Program generated by RoboDK", True)
    robot.setFrame(Pose([807.766544, -963.699898, 41.478944, 0, 0, 0]))
    robot.setTool(Pose([62.5, -108.253175, 100, -60, 90, 0]))
    robot.setSpeed(100) # set speed to 100 mm/s
    robot.setAcceleration(3000) # set speed to 3000 mm/ss    
    robot.MoveJ(Pose([200, 200, 500, 180, 0, 180]), [-46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752] )
    robot.MoveL(Pose([200, 250, 348.734575, 180, 0, -150]), [-41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418] )
    robot.MoveL(Pose([200, 200, 262.132034, 180, 0, -150]), [-43.73892, -3.91728, -35.77935, 58.57566, 54.11615, -253.81122] )
    robot.RunMessage("Setting air valve 1 on")
    robot.RunCode("TCP_On", True)
    robot.Pause(1000)
    robot.MoveL(Pose([200, 250, 348.734575, 180, 0, -150]), [-41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418] )
    robot.MoveL(Pose([250, 300, 278.023897, 180, 0, -150]), [-37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677] )
    robot.MoveL(Pose([250, 250, 191.421356, 180, 0, -150]), [-39.75778, -1.04537, -40.37883, 52.09118, 54.15317, -246.94403] )
    robot.RunMessage("Setting air valve off")
    robot.RunCode("TCP_Off", True)
    robot.Pause(1000)
    robot.MoveL(Pose([250, 300, 278.023897, 180, 0, -150]), [-37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677] )
    robot.MoveL(Pose([250, 200, 278.023897, 180, 0, -150]), [-41.85389, -1.95619, -34.89154, 57.43912, 52.34162, -253.73403] )
    robot.MoveL(Pose([250, 150, 191.421356, 180, 0, -150]), [-43.82111, 3.29703, -40.29493, 56.02402, 56.61169, -249.23532] )
    robot.ProgFinish("Program")
    # robot.ProgSave(".","Program",True)
    for line in robot.PROG:
        print(line)
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)

    input("Press Enter to close...")

if __name__ == "__main__":
    Function to call when the module is executed by itself: test
    test_post()
 """