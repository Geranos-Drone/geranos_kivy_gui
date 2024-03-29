#!/usr/bin/python
##!/home/tim/anaconda3/bin/python

#runs on python 3.7
#run: 'pip3 install kivy'


from scipy.spatial.transform import Rotation

import numpy as np

import kivy
from kivy.app import App
from kivy.config import Config
from kivy.core.window import Window
from kivy.uix.label import Label
from kivy.uix.slider import Slider
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.textinput import TextInput
from kivy.uix.image import Image

import rospy
import roslaunch
import rospkg

# services
from std_srvs.srv import Trigger, SetBool, Empty
from sensor_fusion_comm.srv import InitScale

# messages
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from omav_hovery_msgs.msg import UAVStatus
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TransformStamped


# print in yellow
def print_warn(text):
    fg = lambda text, color: "\33[38;5;" + str(color) + "m" + str(text) + "\33[0m"
    print(fg(text, 11))


class Container(BoxLayout):

    def __init__(self, **kwargs):
        super(Container, self).__init__(**kwargs)
        self.uav_state_sub = rospy.Subscriber("uavstate", UAVStatus, callback=self.UAVStatusCallback, queue_size=1)
        self.dynamixel_state_sub = rospy.Subscriber("dynamixel_state/state_info", String, self.updateDynamixelState)
        self.odometry_sub = rospy.Subscriber("transformed_odometry", Odometry, callback=self.OdometryCallback)
        self.motor_speed_sub = rospy.Subscriber("command/motor_speed", Actuators, callback=self.MotorSpeedCallback, queue_size=1)
        self.control_node_sub = rospy.Subscriber("impedance_module/control_mode", String, self.updateControlMode)
        self.pole_trajectory_sub = rospy.Subscriber("geranos_planner/mode_info", String, self.updateTrajMode)
        self.state_machine_sub = rospy.Subscriber("state_machine/state_info", String, self.updateState)
        self.msf_checker_sub = rospy.Subscriber("state_machine/state_info", String, self.updateState)
        self.imu_sub = rospy.Subscriber("/vectornav/IMU", Imu, self.imuCallback)
        self.vicon_sub = rospy.Subscriber("geranos_boreas/vrpn_client/estimated_transform", TransformStamped, self.viconCallback)
        self.pole_mount_vicon_sub = rospy.Subscriber("geranos_pole_mount/vrpn_client/estimated_transform", TransformStamped, self.PoleMountViconCallback)
        self.pole_white_vicon_sub = rospy.Subscriber("geranos_pole_white/vrpn_client/estimated_transform", TransformStamped, self.PoleWhiteViconCallback)
        self.pole_grey_vicon_sub = rospy.Subscriber("geranos_pole_grey/vrpn_client/estimated_transform", TransformStamped, self.PoleGreyViconCallback)

        self.publish_wp_service = rospy.ServiceProxy('publish_wp', Empty)


        #ROS Publisher
        self.waypoint_pub = rospy.Publisher("gui/waypoint", PoseStamped, queue_size = 10)

        self.script_path = 'not available'
        try:
            self.script_path = rospy.get_param("~script_path")
            self.button_runScript.configure(state=NORMAL)
            print("Loaded param: {}".format(self.script_path))
        except:
            print_warn("Did not load any script.")

        # Variables
        self.POLEMODE = 0
        self.PUBLISHWP = 0
        self.FINEMODE = 0
        self.OdometryRecieved = 0
        self.ImuRecieved = 0
        self.ViconRecieved = 0
        self.ControlRecieved = 0
        self.x_value = 0
        self.y_value = 0
        self.z_value = 0
        self.yaw_value = 0


    #-------------------------------Buttons--------------------------------------------------------

    #INIT_MSF Button
    def init(self):
        init_service = rospy.ServiceProxy('pose_sensor/pose_sensor/initialize_msf_scale', InitScale)
        try:
            init_service(1.0)
            self.ids['console'].text = "Console:  MSF initialized"
            self.ids['init'].background_color = 0, 170/255, 0, 1.0

            #activate disabled Buttons
            self.ids['reset_wp'].disabled = False
            self.ids['publish_wp'].disabled = False
            self.ids['start'].disabled = False
            self.ids['takeoff'].disabled = False
            self.ids['GoTo'].disabled = False
            self.ids['backToPos'].disabled = False
            self.ids['lower'].disabled = False
            self.ids['lift_pole'].disabled = False
            self.ids['land'].disabled = False
        except Exception as exc:
            self.ids['console'].text = "Console:  MSF Error"
            print('Error: ', exc)

    #Switch Button
    def switch(self):
        try:
            switch_service = rospy.ServiceProxy('impedance_module/switch_control_params', Empty)
            switch_service()
            self.ids['console'].text = "Console:  request sent to switch params"
            print("request sent to switch params")
        except Exception as e:
            self.ids['console'].text = "Console:  Switch Error"
            print("Error: ", e)

    #Reset Integrators Button
    def start(self):
        self.ids['console'].text = "Console:  Resetting Integrators"
        print("Resetting Integrators.")
        reset_integrator_service = rospy.ServiceProxy('impedance_module/reset_integrator', Empty)
        try:
            reset_integrator_service()
            print("Integrators reset.")
        except rospy.ServiceException as exc:
            print_warn("Was not able to reset integrators, error: {}".format(exc))
            self.ids['console'].text = "Console:  " + "Was not able to reset integrators, error: {}".format(exc)

    #Take-Off and Land Button
    def takeoff(self):
        if(self.PUBLISHWP == 1):
            self.PUBLISHWP = 0
            self.publish_wp_service()
            self.ids['publish_wp'].background_color = 120/255, 120/255, 120/255, 1
            print("Publish Waypoints disabled")
        
        print("take off")
        reset_integrator_service = rospy.ServiceProxy('impedance_module/reset_integrator', Empty)
        takeoff_service = rospy.ServiceProxy('take_off', Empty)
        try:
            reset_integrator_service()
            self.ids['console'].text = "Console:  Integrators reset"
        except rospy.ServiceException as exc:
            print_warn("Was not able to reset integrators, error: {}".format(exc))
            self.ids['console'].text = "Console:  Was not able to reset integrators, error: {}".format(exc)
        try:
            takeoff_service()
            self.ids['console'].text = "Console:  Taking Off"
            print("Taking off.")
        except rospy.ServiceException as exc:
            print_warn("Not able to take off, error: %s"%exc)
            self.ids['console'].text = "Console:  Not able to take off, error: %s"%exc
            

    #Land Button
    def land(self):
        if(self.PUBLISHWP == 1):
            self.PUBLISHWP = 0
            self.publish_wp_service()
            self.ids['publish_wp'].background_color = 120/255, 120/255, 120/255, 1
            print("Publish Waypoints disabled")
        land_service = rospy.ServiceProxy('land', Empty)
        land_service()
        self.ids['console'].text = "Console:  Landing"
        print("Landing.")


    #Grab and Realease Pole Button
    def GrabPole(self):
        try:
            print('Polemode' + str(self.POLEMODE))
            if(self.POLEMODE == 0):
                self.ids['GrabPole'].text = 'Drop Pole'  # remove later
                grab_pole_service = rospy.ServiceProxy('grab_pole', Empty)
                print("Request sent to Grab Pole")
                grab_pole_service()
                self.ids['console'].text = "Console:  Request sent to Grab Pole"
                self.POLEMODE = 1
            else: 
                self.ids['GrabPole'].text = 'Grab Pole'  # remove later
                DropPole_service = rospy.ServiceProxy('/geranos/release_pole', Empty)
                DropPole_service()
                print("request sent detach Pole")
                self.ids['console'].text = "Console:  Request sent to detach Pole"
                self.POLEMODE = 0
        except Exception as e:
            self.ids['console'].text = "Console:  Dynamixel error: " + str(e)
            self.POLEMODE = 0
            self.ids['GrabPole'].text = 'Grab Pole'  # remove later

    #Go To Pole Button
    def GoTo(self):
        if(self.PUBLISHWP == 1):
            self.PUBLISHWP = 0
            self.publish_wp_service()
            self.ids['publish_wp'].background_color = 120/255, 120/255, 120/255, 1
            print("Publish Waypoints disabled")
        go_to_pole_service = rospy.ServiceProxy('go_to_pole_service', Empty)
        print("Request Sending to Go to Pole")
        self.ids['console'].text = "Console:  Request sent to go to Pole"
        go_to_pole_service()

    #Trajectory Reset Button
    def reset(self):
        reset_service = rospy.ServiceProxy("reset_trajectory_service", Empty)
        self.ids['console'].text = "Console:  Resetting Trajectories"
        print("Resetting Trajectories")
        reset_service()
        self.ids['GoTo'].disabled = False
        self.ids['lower'].disabled = False
        self.ids['lift_pole'].disabled = False

    #Back to Position Hold Button
    def backToPos(self):
        position_hold_service = rospy.ServiceProxy('back_to_position_hold', Empty)
        position_hold_service()
        self.ids['console'].text = "Console:  Going back to position hold"
        print("Going back to position hold.")

    #Lower to Pole Button
    def lower(self):
        if(self.PUBLISHWP == 1):
            self.PUBLISHWP = 0
            self.publish_wp_service()
            self.ids['publish_wp'].background_color = 120/255, 120/255, 120/255, 1
            print("Publish Waypoints disabled")
        grab_pole_service = rospy.ServiceProxy('grab_pole_service', Empty)
        print("Request Sending to Lower Drone to Grab Pole")
        grab_pole_service()
        self.ids['console'].text = "Console:  Lowering to Grab Position"

    #Reset Waypoints Button
    def reset_wp(self):
        try:
            self.ids['x'].value = self.pose_x
            self.ids['y'].value = self.pose_y
            self.ids['z'].value = self.pose_z
            self.ids['yaw'].value = float(self.yaw)

            self.x_value = self.pose_x
            self.y_value = self.pose_y
            self.z_value = self.pose_z
            self.yaw_value = float(self.yaw)

            print("Reset WP")
            self.ids['console'].text = "Console:  Waypoints Reset"
        except Exception as e:
            self.ids['console'].text = "Console:  " + str(e)

    #Publish Waypoints Button
    def publish_wp(self):
        print("Publish WP")
        if(self.PUBLISHWP == 0):
            self.reset_wp()
            
            self.ids['publish_wp'].background_color = 0, 170/255, 0, 1.0
            try:
                self.publish_wp_service()
                self.ids['console'].text = "Console:  Publish Waypoints enabled"
                self.PUBLISHWP = 1
            except Exception as e:
                print_warn(e)
                self.ids['console'].text =  "Console:  " + str(e)
        else:
            self.ids['publish_wp'].background_color = 120/255, 120/255, 120/255, 1
            self.ids['console'].text = "Console:  Publish Waypoints disabled"
            try:
                self.publish_wp_service()
                self.ids['console'].text = "Console:  Publish Waypoints disabled"
                self.PUBLISHWP = 0
            except Exception as e:
                print_warn(e)
                self.ids['console'].text =  "Console:  " + str(e)

    #Fine Mode Button
    def fine_mode(self):
        if (self.FINEMODE == 0):
            self.ids['fine_mode'].background_color = 0, 170/255, 0, 1.0
            self.FINEMODE = 1
            self.ids['console'].text = "Console:  Fine Mode enabled"
        else:
            self.ids['fine_mode'].background_color = 120/255, 120/255, 120/255, 1
            self.FINEMODE = 0
            self.ids['console'].text = "Console:  Fine Mode disabled"


    #Lift Pole Button
    def lift_pole(self):
        if(self.PUBLISHWP == 1):
            self.PUBLISHWP = 0
            self.publish_wp_service()
            self.ids['publish_wp'].background_color = 120/255, 120/255, 120/255, 1
            print("Publish Waypoints disabled")
        liftPole_service = rospy.ServiceProxy("lift_pole_service", Empty)
        liftPole_service()
        print("lifting pole")
        self.ids['console'].text = "Console:  Lifting Pole"

    #-------------------------------ROS-----------------------------------
        
    def UAVStatusCallback(self, msg):
        voltage = msg.motors[0].voltage
        percentage = ((voltage-21)/25.2)*100
        if (percentage < 0):
            percentage = 0
            self.ids['console'].text = "Console:  Exchange batteries!"
        elif (percentage > 100):
            percentage = 100
            self.ids['console'].text = "Console:  Battery voltage too high!!!"

        self.ids['battery'].text = "Battery: " + str(voltage) + " V"

        if (voltage < 21.5):
            self.ids['battery'].color = 205/255, 34/255, 34/255, 1
        elif (voltage < 22):
            self.ids['battery'].color = 242/255, 226/255, 51/255, 1
        else:
            self.ids['battery'].color = 1, 1, 1, 1


    def updateDynamixelState(self, msg):
        if msg.data == "Ready":
            self.ids['GrabPole'].text = 'Grab Pole'
            self.ids['GrabPole'].disabled = False
        elif msg.data == "Centering":
            self.ids['GrabPole'].text = 'Centering'
            self.ids['GrabPole'].disabled = True 
        elif msg.data == "Gripping":
            self.ids['GrabPole'].text = 'Gripping'
            self.ids['GrabPole'].disabled = True 
        elif msg.data == "Gripped":
            self.ids['GrabPole'].text = 'Release Pole'
            self.ids['GrabPole'].disabled = False 
        elif msg.data == "Releasing":
            self.ids['GrabPole'].text = 'Releasing'
            self.ids['GrabPole'].disabled = True 
        elif msg.data == "Service unavailable":
            self.ids['GrabPole'].text = 'Grab Pole'

    def OdometryCallback(self, msg):
        if(self.OdometryRecieved == 0):
            self.ids['msf_checker'].source = 'Checkbox_checked.png'
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.pose_z = msg.pose.pose.position.z
        orientation = msg.pose.pose.orientation
        rot = Rotation.from_quat((orientation.x, orientation.y, orientation.z, orientation.w))
        rot_euler = rot.as_euler('xyz', degrees=True)
        self.yaw = rot_euler[2]

    def imuCallback(self, msg):
        if(self.ImuRecieved == 0):
            self.ids['imu_checker'].source = 'Checkbox_checked.png'

    def viconCallback(self, msg):
        if(self.ViconRecieved == 0):
            self.ids['vicon_checker'].source = 'Checkbox_checked.png'

    def MotorSpeedCallback(self, msg):
        if ((self.ControlRecieved == 0) & (msg.angular_velocities[0] > 1.0)):
            self.ids['control_checker'].source = 'Checkbox_checked.png'
            self.ControlRecieved = 1

    def PoleMountViconCallback(self, msg):
            self.ids['pole_mount'].source = 'Checkbox_checked.png'

    def PoleWhiteViconCallback(self, msg):
            self.ids['pole_white'].source = 'Checkbox_checked.png'

    def PoleGreyViconCallback(self, msg):
            self.ids['pole_grey'].source = 'Checkbox_checked.png'
            
    def updateControlMode(self, msg):
        self.ids['modeControl'].text = "Mode = "+msg.data

    def updateTrajMode(self, msg):
        self.ids['modeTraj'].text = "Mode =\n"+msg.data
        mode = self.ids['modeTraj'].text
        if(mode == "PLACE_WHITE"):
            self.ids['GoTo'].text = "Go To Mount"
            self.ids['lower'].text = "Lower to\nplace"
            self.ids['lift_pole'].text = "Fly up"
        elif (mode == "GET_GREY"):
            self.ids['GoTo'].text = "Go To Grey"
            self.ids['lower'].text = "Lower to\nPole"
            self.ids['lift_pole'].text = "Lift Pole"
        elif (mode == "PLACE_GREY"):
            self.ids['GoTo'].text = "Go To Mount"
            self.ids['lower'].text = "Lower to\nplace"
            self.ids['lift_pole'].text = "Fly up"
        elif (mode == "DONE"):
            self.ids['GoTo'].disabled = True
            self.ids['lower'].disabled = True
            self.ids['lift_pole'].disabled = True
        else:
            self.ids['GoTo'].text = "Go To White"
            self.ids['lower'].text = "Lower to\nPole"
            self.ids['lift_pole'].text = "Lift Pole"

    def updateState(self, msg):
        colour = self.colourSwitcher(msg.data)
        self.ids['StatusMessage'].text = "Control Status =\n"+msg.data
        self.ids['StatusMessage'].color = colour

    #colours for different Control States
    def colourSwitcher(self, text):
        switcher = {
            "Inactive": (205/205, 34/255, 34/255, 1), #red
            "RemoteControl": (1, 1, 1, 1),
            "RemoteControlReadyForOdometry": (120/255, 120/255, 120/255, 1),
            "HaveOdometry": (120/255, 120/255, 120/255, 1),
            "PoseHold": (120/255, 120/255, 120/255, 1),
            "RcTeleOp": (120/255, 120/255, 120/255, 1)
        }
        return switcher.get(text, (1, 1, 1, 1))

    #--------------------------Sliders--------------------------------------
    
    def silder_x(self):
        if(self.PUBLISHWP == 1):
            if (np.abs(self.x_value - self.ids['x'].value) <= 0.3) & (np.abs(self.y_value - self.ids['y'].value) <= 0.3) & (np.abs(self.z_value - self.ids['z'].value) <= 0.3) & (np.abs(self.yaw_value - self.ids['yaw'].value) <= 30.0):
                try:
                    msg = PoseStamped()
                    msg.pose.position.x = float(self.ids['x'].value)
                    msg.pose.position.y = float(self.ids['y'].value)
                    msg.pose.position.z = float(self.ids['z'].value)
                    orientation = msg.pose.orientation
                    rot = Rotation.from_euler('xyz', (0.0, 0.0, float(self.ids['yaw'].value)), degrees=True)
                    orientation.x, orientation.y, orientation.z, orientation.w = rot.as_quat()
                    msg.header.frame_id = "Waypoints"
                    msg.header.stamp = rospy.Time(0)
                    self.waypoint_pub.publish(msg)
                    print("published waypoints")
                    self.x_value = self.ids['x'].value
                    self.y_value = self.ids['y'].value
                    self.z_value = self.ids['z'].value
                    self.yaw_value = self.ids['yaw'].value
                except Exception as e: 
                    print_warn(e)
                    self.ids['console'].text =  "Console:  " + str(e)
            else:
                self.ids['console'].text = "Console:  Input Step too big"
                self.ids['x'].value = self.x_value
                self.ids['y'].value = self.y_value
                self.ids['z'].value = self.z_value
                self.ids['yaw'].value = self.yaw_value
                self.silder_x()
        else:
            print("Please enable Publish Waypoints")

    def minus_x(self):
        if (self.FINEMODE == 0):
            self.ids['x'].value = self.ids['x'].value - 0.1
        else:
            self.ids['x'].value = self.ids['x'].value - 0.01

    def plus_x(self):
        if (self.FINEMODE == 0):
            self.ids['x'].value = self.ids['x'].value + 0.1
        else:
            self.ids['x'].value = self.ids['x'].value + 0.01

    def minus_y(self):
        if (self.FINEMODE == 0):
            self.ids['y'].value = self.ids['y'].value - 0.1
        else:
            self.ids['y'].value = self.ids['y'].value - 0.01

    def plus_y(self):
        if (self.FINEMODE == 0):
            self.ids['y'].value = self.ids['y'].value + 0.1
        else:
            self.ids['y'].value = self.ids['y'].value + 0.01

    def minus_z(self):
        if (self.FINEMODE == 0):
            self.ids['z'].value = self.ids['z'].value - 0.1
        else:
            self.ids['z'].value = self.ids['z'].value - 0.01

    def plus_z(self):
        if (self.FINEMODE == 0):
            self.ids['z'].value = self.ids['z'].value + 0.1
        else:
            self.ids['z'].value = self.ids['z'].value + 0.01

    def minus_yaw(self):
        if (self.FINEMODE == 0):
            self.ids['yaw'].value = self.ids['yaw'].value - 5
        else:
            self.ids['yaw'].value = self.ids['yaw'].value - 1

    def plus_yaw(self):
        if (self.FINEMODE == 0):
            self.ids['yaw'].value = self.ids['yaw'].value + 5
        else:
            self.ids['yaw'].value = self.ids['yaw'].value + 1


    #----------------------------Text Inputs-----------------------------------

    def text_x(self):
        try:
            if((float(self.ids['text_x'].text) <= 5.0) & (float(self.ids['text_x'].text) >= -5.0)):
                self.ids['x'].value = float(self.ids['text_x'].text)
                self.silder_x()
                if (self.PUBLISHWP == 1):
                    self.ids['console'].text = "Console:  Waypoints published"
                else:
                    self.ids['console'].text = "Console:  Please enable Publish Waypoints"
            else:
                self.ids['console'].text = "Console:  Please enter number in scope"
        except Exception as e:
            self.ids['console'].text = "Console:  Please only enter Numbers"


    def text_y(self):
        try:
            if((float(self.ids['text_y'].text) <= 5.0) & (float(self.ids['text_y'].text) >= -5.0)):
                self.ids['y'].value = float(self.ids['text_y'].text)
                self.silder_x()
                if (self.PUBLISHWP == 1):
                    self.ids['console'].text = "Console:  Waypoints published"
                else:
                    self.ids['console'].text = "Console:  Please enable Publish Waypoints"
            else:
                self.ids['console'].text = "Console:  Please enter number in scope"
        except Exception as e:
            self.ids['console'].text = "Console:  Please only enter Numbers"

    def text_z(self):
        try:
            if((float(self.ids['text_z'].text) <= 5.0) & (float(self.ids['text_z'].text) >= -5.0)):
                self.ids['z'].value = float(self.ids['text_z'].text)
                self.silder_x()
                if (self.PUBLISHWP == 1):
                    self.ids['console'].text = "Console:  Waypoints published"
                else:
                    self.ids['console'].text = "Console:  Please enable Publish Waypoints"
            else:
                self.ids['console'].text = "Console:  Please enter number in scope"
        except Exception as e:
            self.ids['console'].text = "Console:  Please only enter numbers"

    def text_yaw(self):
        try:
            if((float(self.ids['text_yaw'].text) <= 250.0) & (float(self.ids['text_yaw'].text) >= -250.0)):
                self.ids['yaw'].value = float(self.ids['text_yaw'].text)
                self.silder_x()
                if (self.PUBLISHWP == 1):
                    self.ids['console'].text = "Console:  Waypoints published"
                else:
                    self.ids['console'].text = "Console:  Please enable Publish Waypoints"
            else:
                self.ids['console'].text = "Console:  Please enter number in scope"
        except Exception as e:
            self.ids['console'].text = "Console:  Please only enter Numbers"

class GeranosApp(App):
    def build(self):
        self.icon = 'Geranos_GUI.png'
        self.title = 'Geranos'

        return Container()

class Geranos_smallApp(App):
    def build(self):
        self.icon = 'Geranos_GUI.png'
        self.title = 'Geranos'

        return Container()

if __name__ == "__main__":

    rospy.init_node('omav_gui')

    try:
        window_size = rospy.get_param('~window_size')
    except Exception as e:
        window_size="small"   
    print(window_size)   

    if(window_size == "small"):
        Window.size = (1900, 400)      
        Config.set('graphics','window_state','visible') #normal
        Config.write()
        app = Geranos_smallApp().run()
    else:
        Config.set('graphics','window_state','maximized') #fullscreen
        Config.write()
        app = GeranosApp().run()
