import sys
import matplotlib.pyplot as plt
from PyQt5.QtCore import QUrl, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, QButtonGroup, QSizePolicy, QSpacerItem
from PyQt5.QtMultimedia import QCamera, QCameraInfo #, QCameraViewfinder
from PyQt5.QtGui import QImage, QPixmap
import cv2
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
import socket
import threading
import time, math
from UIros import BatteryStatusSubscriber, VehicleAttitudeSubscriber, VehicleLocalPositionSubscriber, RGV2TruthDataSubscriber, RGV1TruthDataSubscriber, RGV2CoarseLocalizationSubscriber, ClockSubscriber, MissionPhaseSubscriber, ModeSubscriber, ImageSubscriber, RGV1CoarseLocalizationSubscriber
from px4_msgs.msg import VehicleAttitude,BatteryStatus,VehicleGlobalPosition, VehicleLocalPosition, VehicleStatus  # Make sure this matches the actual message type
from std_msgs.msg import Float64MultiArray, String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy
import time
import math
import ros2topic

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


    

    def plot_points(self, dot_history):
        
        def interpolate_color(start_color, end_color, factor):
            """Interpolates between two colors with a given factor."""
            return tuple(start_color[i] + (end_color[i] - start_color[i]) * factor for i in range(3))

        self.axes.clear()
        self.fig.set_facecolor('white')
        self.axes.set_facecolor('grey')
        self.axes.set_title('UAS and RGV Plotting (ENU Frame)')
        self.axes.set_xlabel('East [m]')
        self.axes.set_ylabel('North [m]')
        # self.axes.tick_params(colors='white')
        self.axes.set_xlim(-17, 15)
        self.axes.set_ylim(-17, 15)

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

         # Define start and end colors for "1 estimate" (e.g., light to dark gray)
        start_color_yellow = (1, 1, 0)
        end_color_yellow = (0.8, 0.8, 0)
        num_points = len(dot_history["1 estimate"])
        for i, point in enumerate(dot_history["1 estimate"]):
            color = interpolate_color(start_color_yellow, end_color_yellow, i / (num_points - 1) if num_points > 1 else 0)
            self.axes.plot(point[0], point[1], marker='.', color=color, linestyle='None')

         # Define start and end colors for "uas" (e.g., light to dark blue)
        start_color_blue = (0.8, 0.8, 1)  # light blue
        end_color_blue = (0, 0, 0.6)      # dark blue
        num_points = len(dot_history["uas"])
        for i, point in enumerate(dot_history["uas"]):
            color = interpolate_color(start_color_blue, end_color_blue, i / (num_points - 1) if num_points > 1 else 0)
            self.axes.plot(point[0], point[1], marker='x', color=color, linestyle='None')

        legend_elements = [
            Line2D([0], [0], marker='o', label='RGV1 Truth', markersize=7, markerfacecolor=end_color_red),
            Line2D([0], [0], marker='o', label='RGV2 Truth', markersize=7, markerfacecolor=end_color_green),
            Line2D([0], [0], marker='.', label='RGV2 Estimate', markersize=10, markerfacecolor=end_color_gray),
            Line2D([0], [0], marker='.', label='RGV1 Estimate', markersize=10, markerfacecolor=end_color_yellow),
            Line2D([0], [0], marker='x', label='UAS', markersize=7, markerfacecolor=end_color_blue)
        ]
        self.axes.legend(handles=legend_elements)

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
            "uas": [],
        }
        

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
        
        
       
        self.camera2Label = QLabel('Camera 2 Preview')
        self.camera2Label.setStyleSheet("background-color: gray")
        self.camera2Label.setFixedSize(800, 400)
        topLayout.addWidget(self.camera2Label)
       
       

        mainLayout.addLayout(topLayout)

        #self.initCamera()

        # Bottom half layout for UAS state and RGV Estimate
        self.uasStateLabel = QLabel('UAS State: x=?, y=?, z=?, u=?, v=?, q=?, φ=?, θ=?, ψ=?')
        self.uasStateLabel.setStyleSheet("border: 1px solid black; padding: 5px;")
        mainLayout.addWidget(self.uasStateLabel)

        self.rgv1EstimateLabel = QLabel('Current RGV1 Estimate: x=?, y=?, z=?')
        self.rgv1EstimateLabel.setStyleSheet("border: 1px solid black; padding: 5px;")
        mainLayout.addWidget(self.rgv1EstimateLabel)

        self.rgv2EstimateLabel = QLabel('Current RGV2 Estimate: x=?, y=?, z=?')
        self.rgv2EstimateLabel.setStyleSheet("border: 1px solid black; padding: 5px;")
        mainLayout.addWidget(self.rgv2EstimateLabel)


        self.missionPhaseLabel = QLabel('Mission Phase: Unknown')
        self.missionPhaseLabel.setStyleSheet("border: 1px solid black; padding: 5px;")
        mainLayout.addWidget(self.missionPhaseLabel)

        self.controlModeLabel = QLabel('Control Mode: Unknown')
        self.controlModeLabel.setStyleSheet("border: 1px solid black; padding: 5px;")
        mainLayout.addWidget(self.controlModeLabel)


        self.batteryStorageLabel = QLabel('Battery Level: ---%')
        self.batteryStorageLabel.setStyleSheet("border: 1px solid black; padding: 5px;")
        mainLayout.addWidget(self.batteryStorageLabel)

      
        
        self.flightTime = QLabel('Fight Time: MM:SS')
        self.flightTime.setStyleSheet("border: 1px solid black; padding: 5px;")
        mainLayout.addWidget(self.flightTime)

    

        # Set the main layout
        centralWidget = QWidget()
        centralWidget.setLayout(mainLayout)
        self.setCentralWidget(centralWidget)

        self.show()

    def updateUASState(self, x, y, z, u, v, q, phi, theta, psi):
        self.uasStateLabel.setText(f'UAS State: x={x}, y={y}, z={z}, u={u}, v={v}, q={q}, φ={phi}, θ={theta}, ψ={psi}')  
        self.dot_history["uas"].append((x,y))

        if(len(self.dot_history["uas"]) > 10):
            self.dot_history["uas"].pop(0)
        
        self.plotCanvas.plot_points(self.dot_history)

    def updateRGVEstimate(self, x, y, z, type):
        if type == "1 estimate":
            self.rgv1EstimateLabel.setText(f'Current RGV1 Estimate: x={x}, y={y}, z={z}')
        
        if type == "2 estimate":
            self.rgv2EstimateLabel.setText(f'Current RGV2 Estimate: x={x}, y={y}, z={z}')

        if x != 0 and y != 0:        
            self.dot_history[type].append((x, y))

            if(len(self.dot_history[type]) > 10):
                self.dot_history[type].pop(0)

            self.plotCanvas.plot_points(self.dot_history) 
         

    def updateMissionPhase(self, phase):
        self.missionPhaseLabel.setText(f'Mission Phase: {phase}')

    def updateControlMode(self, mode):
        self.controlModeLabel.setText(f'Control Mode: {mode}')

    def updateBatteryLevel(self, batteryLevel):
        self.batteryStorageLabel.setText(f'Battery Level: {batteryLevel} %')

    

    def updateFlightTime(self, flightMinutes, flightSeconds):
        self.flightTime.setText(f'Flight Time: {flightMinutes}:{flightSeconds}')

    def updateCameraLabel(self, cv_image):
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        q_image = QImage(rgb_image.data, rgb_image.shape[1], rgb_image.shape[0], QImage.Format_RGB888)
        q_pixmap = QPixmap.fromImage(q_image)
        self.camera2Label.setPixmap(q_pixmap)
        self.camera2Label.setScaledContents(True)

def NED2ENU(N,E,D):
    x_enu = E
    y_enu = N
    z_enu = -D
    return x_enu, y_enu, z_enu
    
def euler_from_quaternion(x, y, z, w):
   
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

    #[E, N, U] = NED2ENU(uas_pos_node.x_pos, uas_pos_node.y_pos, uas_pos_node.z_pos)
    [E_dot, N_dot, U_dot] = NED2ENU(uas_pos_node.x_vel, uas_pos_node.y_vel, uas_pos_node.z_vel)

    ex.updateUASState(uas_pos_node.x_pos, uas_pos_node.y_pos, uas_pos_node.z_pos, E_dot, N_dot, U_dot, roll, pitch, yaw)
    #ex.updateUASState(E, N, U, E_dot, N_dot, U_dot, roll, pitch, yaw)

    QTimer.singleShot(500, lambda: update_state(uas_pos_node, attitude_node, ex))

def update_battery_level(battery_node, ex):
    rclpy.spin_once(battery_node)
    ex.updateBatteryLevel(battery_node.battery_remaining)
    QTimer.singleShot(500, lambda: update_battery_level(battery_node, ex))  # Schedule the next update after 2 seconds

def update_rgv_estimate(rgv_node, phase_node, ex):
    rclpy.spin_once(phase_node)
    phase = phase_node.mission_phase

    # if phase == 'coarse' or phase == 'fine' or phase == 'jointTrailing':
    rclpy.spin_once(rgv_node, timeout_sec=0.1)
    ex.updateRGVEstimate(rgv_node.rgv_x, rgv_node.rgv_y, rgv_node.rgv_z, rgv_node.rgv_type)

    QTimer.singleShot(500, lambda: update_rgv_estimate(rgv_node, phase_node, ex))

def update_flight_time(clock_node, ex):
    rclpy.spin_once(clock_node)
    
    minutes, seconds = divmod(clock_node.time_sec, 60)
    ex.updateFlightTime(minutes, seconds)

    QTimer.singleShot(500, lambda: update_flight_time(clock_node, ex))

def update_mission_phase(phase_node,ex):
    rclpy.spin_once(phase_node)
    ex.updateMissionPhase(phase_node.mission_phase)
    QTimer.singleShot(500, lambda: update_mission_phase(phase_node, ex))

def update_control_mode(control_node, ex):
    rclpy.spin_once(control_node)
    ex.updateControlMode(control_node.flight_mode)
    QTimer.singleShot(500, lambda: update_control_mode(control_node, ex))

def update_image(image_node, ex):
    rclpy.spin_once(image_node)
    ex.updateCameraLabel(image_node.image_data)
    QTimer.singleShot(33.33, lambda: update_image(image_node, ex))
    


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
    rgv1_estimate_node = RGV1CoarseLocalizationSubscriber()


 
    clock_node = ClockSubscriber()
    phase_node = MissionPhaseSubscriber()
    control_node = ModeSubscriber()
    image_node = ImageSubscriber()

    update_state(uas_pos_node, attitude_node, ex)
    update_battery_level(battery_node, ex)
    update_rgv_estimate(rgv1_state_node, phase_node, ex)
    update_rgv_estimate(rgv2_state_node, phase_node, ex)

    
    update_rgv_estimate(rgv2_estimate_node, phase_node, ex)

    update_rgv_estimate(rgv1_estimate_node, phase_node, ex)
    

    update_flight_time(clock_node, ex)
    update_mission_phase(phase_node, ex)
    update_control_mode(control_node, ex)
    update_image(image_node,ex)

   

    sys.exit(app.exec_())

    

if __name__ == '__main__':
    main()