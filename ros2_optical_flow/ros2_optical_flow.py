# Copyright (c) 2023 Aditya Kamath
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import rclpy
import numpy as np
from typing import Optional
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from rclpy.timer import Timer
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Twist, Point, Quaternion, Vector3, TransformStamped, Transform
import serial
from rclpy.node import Node
from example_interfaces.msg import Int64
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf_transformations import quaternion_from_euler

# hard-coded values for PAA5100 and PMW3901 (to be verified for PMW3901)
FOV_DEG = 42.0
RES_PIX = 35

class OpticalFlowPublisher(Node):
    def __init__(self, node_name='optical_flow_ros'):
        super().__init__(node_name)
        self._odom_pub = self.create_publisher(Odometry, "example/odom", 10) # type: ignore
        self._tf_broadcaster = TransformBroadcaster(self) #Optional[TransformBroadcaster] = None
        self._timer = self.create_timer(0.1, self.publish_odom) # type: ignore
        #self.publish_odom

        # declare parameters and default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('timer_period', 0.01),
                ('sensor_timeout', 1.0),
                ('parent_frame', 'odom'),
                ('child_frame', 'base_link'),
                ('x_init', 0.0),
                ('y_init', 0.0),
                ('z_height', 0.025),
                ('board', 'pmw3901'),
                ('scaler', 5),
                ('spi_nr', 0),
                ('spi_slot', 'front'),
                ('rotation', 0),
                ('publish_tf', True),
            ]
        )
        
        self._pos_x = self.get_parameter('x_init').value
        self._pos_y = self.get_parameter('y_init').value
        self._pos_z = self.get_parameter('z_height').value
        #self._angle = 0.0
        self._scaler = self.get_parameter('scaler').value
        self._dt = self.get_parameter('timer_period').value
        self._sensor = None
        
        self.get_logger().info('Initialized')

    def publish_odom(self): 
        ser = serial.Serial('/dev/ttyACM0',9600, timeout=1)
        ser.reset_input_buffer()
        while True: 
            read_serial=ser.readline()
            sensor_data = read_serial.split()

            #delta_x_int, delta_y_int = self.new_method(sensor_data)
            [pos_x, pos_y, v_x, v_y, angle, angledot] = self.new_method(sensor_data)

            # dx = delta_x_int
            # dy = delta_y_int   
     
            # fov = np.radians(FOV_DEG)
            # cf = self._pos_z*2*np.tan(fov/2)/(RES_PIX*self._scaler)

            # dist_x, dist_y = 0.0, 0.0
            # if self.get_parameter('board').value == 'paa5100':
            #     # Convert data from sensor frame to ROS frame for PAA5100
            #     # ROS frame: front/back = +x/-x, left/right = +y/-y
            #     # Sensor frame: front/back = -y/+y, left/right = +x/-x
            #         dist_x = -1*cf*dy
            #         dist_y = cf*dx
            # elif self.get_parameter('board').value == 'pmw3901':
            #     # ROS and Sensor frames are assumed to align for PMW3901 based on https://docs.px4.io/main/en/sensor/pmw3901.html#mounting-orientation
            #     dist_x = cf*dx
            #     dist_y = cf*dy
            
            #self._pos_x += dist_x
            #self._pos_y += dist_y

            self._pos_x = pos_x
            self._pos_y = pos_y
            q = quaternion_from_euler(0, 0, -angle)    
            print(q)
            odom_msg = Odometry(
                header = Header(
                    stamp = self.get_clock().now().to_msg(),
                    frame_id = self.get_parameter('parent_frame').value
                ),
                child_frame_id = self.get_parameter('child_frame').value,
                pose = PoseWithCovariance(
                    pose = Pose(position = Point(x=self._pos_x, y=self._pos_y, z=self._pos_z), orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3]))
                ),
                twist = TwistWithCovariance(
                    #twist = Twist(linear = Vector3(x=dist_x/self._dt, y=dist_y/self._dt, z=0.0))
                    twist = Twist(linear = Vector3(x=v_x, y=v_y, z=0.0), angular = Vector3(x = 0.0, y = 0.0, z = angledot))
                ),
            )
            self._odom_pub.publish(odom_msg)

            if self.get_parameter('publish_tf').value is True:
                tf_msg = TransformStamped(
                    header = odom_msg.header,
                    child_frame_id = odom_msg.child_frame_id,
                    transform = Transform(translation = Vector3(x=odom_msg.pose.pose.position.x,
                                                                y=odom_msg.pose.pose.position.y,
                                                                z=odom_msg.pose.pose.position.z),
                                         rotation = Quaternion(x = odom_msg.pose.pose.orientation.x, y = odom_msg.pose.pose.orientation.y, z = odom_msg.pose.pose.orientation.z, w = odom_msg.pose.pose.orientation.w)),
                )
                self._tf_broadcaster.sendTransform(tf_msg)

    def new_method(self, sensor_data):
        
        # if len(sensor_data) > 1:	
        #     delta_x = sensor_data[0].decode()
        #     delta_y = sensor_data[1].decode()
        #     if (len(delta_x) == 1):
        #         delta_x_int = int(delta_x)
        #     elif (delta_x[1:].isnumeric() == 1):
        #         delta_x_int = int(delta_x)
        #     else:
        #         delta_x_int = 0
        #     if (len(delta_x) == 1):
        #         delta_y_int = int(delta_y)
        #     elif (delta_x[1:].isnumeric() == 1):
        #         delta_y_int = int(delta_y)
        #     else:
        #         delta_y_int = 0	
        # else:
        #       delta_x_int = 0
        #       delta_y_int = 0
        # return delta_x_int,delta_y_int
    
        if len(sensor_data) > 5:	
            pos_x_st = sensor_data[0].decode()
            pos_y_st = sensor_data[1].decode()
            v_x_st = sensor_data[2].decode()
            v_y_st =sensor_data[3].decode()
            angle_st =sensor_data[4].decode()
            angledot_st =sensor_data[5].decode()
            print(angle_st)
            if (len(pos_x_st) == 4):
                pos_x = float(pos_x_st)
            elif (pos_x_st[4:].isnumeric() == 1):
                pos_x = float(pos_x_st)
            else:
                pos_x = self._pos_x

            if (len(pos_y_st) == 4):
                pos_y = float(pos_y_st)
            elif (pos_y_st[4:].isnumeric() == 1):
                pos_y = float(pos_y_st)
            else:
                pos_y = self._pos_y

            if (len(v_x_st) == 4):
                v_x = float(v_x_st)
            else:
                v_x = float(v_x_st)

            if (len(v_y_st) == 4):
                v_y = float(v_y_st)
            else:
                v_y = float(v_y_st)

            if (len(angle_st) == 4):
                 angle = float(angle_st)
            elif (angle_st[3:].isnumeric() == 1):
                 angle = float(angle_st)
            else:
                 angle = 0.0

            if (len(angledot_st) == 4):
                 angledot = float(angledot_st)
            else:
                 angledot = float(angledot_st)
            
        else:
              pos_x = self._pos_x
              pos_y = self._pos_y
              v_x = 0.0
              v_y = 0.0
              angle = 0.0
              angledot = 0.0
        print(angle)
        return pos_x, pos_y, v_x, v_y, angle, angledot
    


def main(args=None):
        rclpy.init(args=args)
        node = OpticalFlowPublisher()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
        main()
