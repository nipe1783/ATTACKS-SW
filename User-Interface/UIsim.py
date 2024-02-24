import sys
import matplotlib.pyplot as plt
from PyQt5.QtCore import QUrl, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, QButtonGroup
from PyQt5.QtMultimedia import QCamera, QCameraInfo #, QCameraViewfinder
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import socket
import threading
import time, math
from UIros import BatteryStatusSubscriber, VehicleAttitudeSubscriber, VehicleLocalPositionSubscriber, RGV2TruthDataSubscriber, RGV1TruthDataSubscriber, RGV2CoarseLocalizationSubscriber, ClockSubscriber, MissionPhaseSubscriber
from px4_msgs.msg import VehicleAttitude,BatteryStatus,VehicleGlobalPosition, VehicleLocalPosition  # Make sure this matches the actual message type
from std_msgs.msg import Float64MultiArray, String
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy
import time
import math

truth_values = []
estimate_values = []

class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        FigureCanvas.__init__(self, self.fig)
        self.setParent(parent)
        #self.init_plot()


    def init_plot(self):
        self.axes.set_title('UAS and RGV Plotting (ENU Frame)')
        self.axes.set_xlabel('East')
        self.axes.set_ylabel('North')

    def plot_rgv2_point(self, x,y, color):
        if math.isnan(x) or math.isnan(y):
            print("Skipping NaN point")
            return
        #self.axes.clear()
        self.axes.plot(x, y, color)
        self.axes.set_title('UAS and RGV Plotting (ENU Frame)')
        self.axes.set_xlabel('East [m]')
        self.axes.set_ylabel('North [m]')
        #self.axes.relim()
        self.axes.autoscale_view()
        self.draw()

    def plot_rgv1_point(self, x,y, color):
        if math.isnan(x) or math.isnan(y):
            print("Skipping NaN point")
            return
        #self.axes.clear()
        self.axes.plot(x, y, color)
        self.axes.set_title('UAS and RGV Plotting (ENU Frame)')
        self.axes.set_xlabel('East [m]')
        self.axes.set_ylabel('North [m]')
        #self.axes.relim()
        self.axes.autoscale_view()
        self.draw()

    def plot_rgv_points(self, dot_history):
        
        def interpolate_color(start_color, end_color, factor):
            """Interpolates between two colors with a given factor."""
            return tuple(start_color[i] + (end_color[i] - start_color[i]) * factor for i in range(3))

        self.axes.clear()
        self.fig.set_facecolor('white')
        self.axes.set_facecolor('black')
        self.axes.set_title('UAS and RGV Plotting (ENU Frame)')
        self.axes.set_xlabel('East [m]')
        self.axes.set_ylabel('North [m]')
        # self.axes.tick_params(colors='white')
        self.axes.set_xlim(-25, 25)
        self.axes.set_ylim(-25, 25)

        # Define start and end colors for "1 true" (e.g., light to dark red)
        start_color_red = (1, 0.5, 0.5)
        end_color_red = (0.8, .2, .2)
        num_points = len(dot_history["1 true"])
        for i, point in enumerate(dot_history["1 true"]):
            color = interpolate_color(start_color_red, end_color_red, i / (num_points - 1) if num_points > 1 else 0)
            self.axes.plot(point[0], point[1], marker='o', color=color, linestyle='None')

        # Define start and end colors for "2 true" (e.g., light to dark green)
        start_color_green = (1, 1, 1)
        end_color_green = (0.7, 0.7, 0.7)
        num_points = len(dot_history["2 true"])
        for i, point in enumerate(dot_history["2 true"]):
            color = interpolate_color(start_color_green, end_color_green, i / (num_points - 1) if num_points > 1 else 0)
            self.axes.plot(point[0], point[1], marker='o', color=color, linestyle='None')

        # Define start and end colors for "2 estimate" (e.g., light to dark gray)
        start_color_gray = (0, 0.9, 0.9)
        end_color_gray = (0, 0.8, 0.8)
        num_points = len(dot_history["2 estimate"])
        for i, point in enumerate(dot_history["2 estimate"]):
            color = interpolate_color(start_color_gray, end_color_gray, i / (num_points - 1) if num_points > 1 else 0)
            self.axes.plot(point[0], point[1], marker='.', color=color, linestyle='None')

        self.axes.autoscale_view()
        self.draw()


    def plot_uas_point(self, x,y):
        if math.isnan(x) or math.isnan(y):
            print("Skipping NaN point")
            return
        #self.axes.clear()
        self.axes.plot(x, y, 'b.')
        #self.axes.relim()
        self.axes.autoscale_view()
        self.draw()


class App(QMainWindow):
    def __init__(self):
        super().__init__()
        self.title = 'ATTACKS User Interface'
        self.left = 100
        self.top = 100
        self.width = 800
        self.height = 600
        self.initUI()
        self.dot_history = {
            "1 true": [],
            "1 estimate": [],
            "2 true": [],
            "2 estimate": [],
        }
        #self.plotCanvas = PlotCanvas(self, width=5, height=4)
        #self.points = []

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        # Create the main layout
        mainLayout = QVBoxLayout()

        # Top half layout
        topLayout = QHBoxLayout()

        # Plot area
        self.plotCanvas = PlotCanvas(self, width=5, height=4)
        topLayout.addWidget(self.plotCanvas)

        # Camera previews
        cameraLayout = QVBoxLayout()
        #self.cameraViewfinder = QCameraViewfinder()
        camera2Label = QLabel('Camera 2 Preview')
        camera2Label.setStyleSheet("background-color: gray")
        #cameraLayout.addWidget(self.cameraViewfinder)  # Add webcam viewfinder here
        cameraLayout.addWidget(camera2Label)
        topLayout.addLayout(cameraLayout)

        mainLayout.addLayout(topLayout)

        #self.initCamera()

        # Bottom half layout for UAS state and RGV Estimate
        self.uasStateLabel = QLabel('UAS State: x=?, y=?, z=?, u=?, v=?, q=?, φ=?, θ=?, ψ=?')
        self.uasStateLabel.setStyleSheet("border: 1px solid black; padding: 5px;")
        mainLayout.addWidget(self.uasStateLabel)

        self.rgvEstimateLabel = QLabel('Current RGV Estimate: x=?, y=?, z=?')
        self.rgvEstimateLabel.setStyleSheet("border: 1px solid black; padding: 5px;")
        mainLayout.addWidget(self.rgvEstimateLabel)

        self.missionPhaseLabel = QLabel('Mission Phase: Unknown')
        self.missionPhaseLabel.setStyleSheet("border: 1px solid black; padding: 5px;")
        mainLayout.addWidget(self.missionPhaseLabel)

        controlModeLayout = QHBoxLayout()
        self.controlModeLabel = QLabel('Control Mode:')
        controlModeLayout.addWidget(self.controlModeLabel)

        self.autonomousButton = QPushButton('Autonomous')
        self.autonomousButton.setCheckable(True)
        self.manualOverrideButton = QPushButton('Manual Override')
        self.manualOverrideButton.setCheckable(True)

        self.controlModeButtonGroup = QButtonGroup()
        self.controlModeButtonGroup.addButton(self.autonomousButton)
        self.controlModeButtonGroup.addButton(self.manualOverrideButton)

        controlModeLayout.addWidget(self.autonomousButton)
        controlModeLayout.addWidget(self.manualOverrideButton)

        mainLayout.addLayout(controlModeLayout)

        #Battery, Storage, Flight Time
        self.batteryStorageLabel = QLabel('Battery Level: ---%')
        self.batteryStorageLabel.setStyleSheet("border: 1px solid black; padding: 5px;")
        mainLayout.addWidget(self.batteryStorageLabel)

        self.memoryStorageLabel = QLabel('Storage Level: -- GB / -- GB ~ -- %')
        self.memoryStorageLabel.setStyleSheet("border: 1px solid black; padding: 5px;")
        mainLayout.addWidget(self.memoryStorageLabel)

        
        self.flightTime = QLabel('Fight Time: MM:SS')
        self.flightTime.setStyleSheet("border: 1px solid black; padding: 5px;")
        mainLayout.addWidget(self.flightTime)

    

        # Set the main layout
        centralWidget = QWidget()
        centralWidget.setLayout(mainLayout)
        self.setCentralWidget(centralWidget)

        self.show()
    #def initCamera(self):
    #    self.camera = QCamera(QCameraInfo.defaultCamera())
    #    self.camera.setViewfinder(self.cameraViewfinder)
    #    self.camera.start()

    def updateUASState(self, x, y, z, u, v, q, phi, theta, psi):
        self.uasStateLabel.setText(f'UAS State: x={x}, y={y}, z={z}, u={u}, v={v}, q={q}, φ={phi}, θ={theta}, ψ={psi}')
        #self.plotCanvas.plot_uas_point(x,y)  

    def updateRGVEstimate(self, x, y, z, type):
        self.rgvEstimateLabel.setText(f'Current RGV Estimate: x={x}, y={y}, z={z}')
        #self.points.append((x, y))  # Add new point to the list
        self.dot_history[type].append((x, y))

        if(len(self.dot_history[type]) > 10):
            self.dot_history[type].pop(0)

        # if type == "1 true":

        #     self.plotCanvas.plot_rgv1_point(x,y, 'ro')
        # elif type == "1 estimate":
        #      self.plotCanvas.plot_rgv2_point(x,y, "k.")  
        # elif type == "2 true":
        #      self.plotCanvas.plot_rgv2_point(x,y,"go") 
        # elif type == "2 estimate":
        #      self.plotCanvas.plot_rgv2_point(x,y,"k.")

        self.plotCanvas.plot_rgv_points(self.dot_history) 
         

    def updateMissionPhase(self, phase):
        self.missionPhaseLabel.setText(f'Mission Phase: {phase}')

    def updateBatteryLevel(self, batteryLevel):
        self.batteryStorageLabel.setText(f'Battery Level: {batteryLevel} %')

    def updateMemoryLevel(self, gb_used, percent_used):
        self.memoryStorageLabel.setText(f'Memory Usage: {gb_used} GB / -- GB ~ {percent_used} %')

    def updateFlightTime(self, flightMinutes, flightSeconds):
        self.flightTime.setText(f'Flight Time: {flightMinutes}:{flightSeconds}')

def NED2ENU(N,E,D):
    x_enu = E
    y_enu = N
    z_enu = -D
    return x_enu, y_enu, z_enu
    
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1) * (180/math.pi)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2) * (180/math.pi)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4) * (180/math.pi)

    return roll_x, pitch_y, yaw_z
     
    #print("roll: " + str(roll_x) + "\npitch: " + str(pitch_y) + "\nyaw: " + str(yaw_z))

def update_state(uas_pos_node ,attitude_node, ex):
    rclpy.spin_once(attitude_node)
    rclpy.spin_once(uas_pos_node)

    w = attitude_node.quat[0]
    x = attitude_node.quat[1]
    y = attitude_node.quat[2]
    z = attitude_node.quat[3]

    [roll, pitch, yaw]= euler_from_quaternion(x,y,z,w)

    [E, N, U] = NED2ENU(x,y,z)
    [E_dot, N_dot, U_dot] = NED2ENU(uas_pos_node.x_vel, uas_pos_node.y_vel, uas_pos_node.z_vel)

    ex.updateUASState(E, N, U, E_dot, N_dot, U_dot, roll, pitch, yaw)

    QTimer.singleShot(200, lambda: update_state(uas_pos_node, attitude_node, ex))

def update_battery_level(battery_node, ex):
    rclpy.spin_once(battery_node)
    ex.updateBatteryLevel(battery_node.battery_remaining)
    QTimer.singleShot(200, lambda: update_battery_level(battery_node, ex))  # Schedule the next update after 2 seconds

def update_rgv_estimate(rgv_node, ex):
    rclpy.spin_once(rgv_node)
    print("RGV Estimate: ", rgv_node.rgv_x, rgv_node.rgv_y, rgv_node.rgv_z, rgv_node.rgv_type)
    ex.updateRGVEstimate(rgv_node.rgv_x, rgv_node.rgv_y, rgv_node.rgv_z, rgv_node.rgv_type)
    QTimer.singleShot(200, lambda: update_rgv_estimate(rgv_node, ex))

def update_flight_time(clock_node, ex):
    rclpy.spin_once(clock_node)
    
    minutes, seconds = divmod(clock_node.time_sec, 60)
    ex.updateFlightTime(minutes, seconds)

    QTimer.singleShot(200, lambda: update_flight_time(clock_node, ex))

def update_mission_phase(phase_node,ex):
    rclpy.spin_once(phase_node)
    ex.updateMissionPhase(phase_node.mission_phase)
    QTimer.singleShot(200, lambda: update_mission_phase(phase_node, ex))





def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    ex = App()
    ex.show()

    battery_node = BatteryStatusSubscriber()
    attitude_node = VehicleAttitudeSubscriber()
    uas_pos_node = VehicleLocalPositionSubscriber()
    rgv1_state_node = RGV1TruthDataSubscriber()
    rgv2_state_node = RGV2TruthDataSubscriber()
    rgv2_estimate_node = RGV2CoarseLocalizationSubscriber()
    clock_node = ClockSubscriber()
    phase_node = MissionPhaseSubscriber()
    update_state(uas_pos_node, attitude_node, ex)
    update_battery_level(battery_node, ex)
    update_rgv_estimate(rgv1_state_node, ex)
    update_rgv_estimate(rgv2_state_node, ex)
    update_rgv_estimate(rgv2_estimate_node, ex)
    update_flight_time(clock_node, ex)
    update_mission_phase(phase_node, ex)

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()