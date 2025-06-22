#!/usr/bin/env python
# ROS2 version of imu.py
# Reads the RTIMU sensor and publishes pose and raw sensor data.

# topics
# publishes
# /pose, Vector3 .z is compass heading in degrees corrected by magnetometer calibration
# /accel raw sensor values in g
# /gyro raw sensor values in deg/s
# /compass raw sensor values in uT

import rclpy
from rclpy.node import Node
import RTIMU
import os.path
from  geometry_msgs.msg import Vector3
import time
import math

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.get_logger().info('Initializing IMU node...')

        SETTINGS_FILE = "RTIMULib"

        abs_path = os.path.abspath(SETTINGS_FILE + ".ini")
        self.get_logger().info(f"Using IMU settings file: {abs_path}")
        if not os.path.exists(SETTINGS_FILE + ".ini"):
            self.get_logger().info("Settings file does not exist, will be created")

        s = RTIMU.Settings(SETTINGS_FILE)
        self.imu = RTIMU.RTIMU(s)

        self.get_logger().info(f"IMU Name: {self.imu.IMUName()}")

        if not self.imu.IMUInit():
            self.get_logger().error("************* IMU Init Failed, shutting down. *************")
            # In a class-based node, we can't just quit().
            # We'll prevent the timer from being created and let the main function exit.
            self.destroy_node()
            rclpy.shutdown()
            return
        else:
            self.get_logger().info("IMU Init Succeeded")

        # Set fusion parameters
        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)

        # Publishers
        self.pub_accel = self.create_publisher(Vector3, 'accel', 10)
        self.pub_gyro = self.create_publisher(Vector3, 'gyro', 10)
        self.pub_compass = self.create_publisher(Vector3, 'compass', 10)
        self.pub_pose = self.create_publisher(Vector3, 'pose', 10)

        # Main loop timer
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz

    def timer_callback(self):
        if self.imu.IMURead():
            data = self.imu.getIMUData()

            # Sample of data returned:
            # {'accelValid': True, 'fusionQPoseValid': True, 'timestamp': ...,
            #  'compassValid': True, 'compass': (-24.0, 20.3, 2.3), # uT
            #  'accel': (-0.4, 0.8, 0.3), # g
            #  'gyroValid': True, 'gyro': (0.006, 0.014, -0.004), # rad/s
            #  'fusionQPose': (0.04, 0.29, -0.5, -0.8),
            #  'fusionPoseValid': True, 'fusionPose': (1.2, 0.4, -2.7)} # rad

            fusionPose = data.get("fusionPose", (0.0, 0.0, 0.0))
            gyro = data.get('gyro', (0.0, 0.0, 0.0))
            accel = data.get('accel', (0.0, 0.0, 0.0))
            compass = data.get('compass', (0.0, 0.0, 0.0))

            # Convert radians to degrees for pose and gyro
            rollDeg = math.degrees(fusionPose[0])
            pitchDeg = math.degrees(fusionPose[1])
            yawDeg = math.degrees(fusionPose[2])

            gx_dps = math.degrees(gyro[0])
            gy_dps = math.degrees(gyro[1])
            gz_dps = math.degrees(gyro[2])

            self.get_logger().debug(
                f"imu: pose rpy=({rollDeg:.2f},{pitchDeg:.2f},{yawDeg:.2f}), "
                f"gyro xyz=({gx_dps:.1f},{gy_dps:.1f},{gz_dps:.1f})deg/s, "
                f"acc xyz=({accel[0]:.2f},{accel[1]:.2f},{accel[2]:.2f})g, "
                f"mag xyz=({compass[0]:.1f},{compass[1]:.1f},{compass[2]:.1f})uT"
            )

            # Publish compass data (uT)
            self.pub_compass.publish(Vector3(x=compass[0], y=compass[1], z=compass[2]))
            # Publish gyro data (deg/s)
            self.pub_gyro.publish(Vector3(x=gx_dps, y=gy_dps, z=gz_dps))
            # Publish accel data (g)
            self.pub_accel.publish(Vector3(x=accel[0], y=accel[1], z=accel[2]))
            # Publish pose data (roll, pitch, yaw in degrees)
            self.pub_pose.publish(Vector3(x=rollDeg, y=pitchDeg, z=yawDeg))
        else:
            self.get_logger().error('Could not read IMU')

def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuNode()

    if rclpy.ok():
        try:
            rclpy.spin(imu_node)
        except KeyboardInterrupt:
            print("Keyboard interrupt, stopping IMU node.")
        finally:
            # The RTIMU library doesn't seem to have a specific de-init function.
            # Destroying the node and shutting down rclpy is sufficient.
            imu_node.destroy_node()
            # rclpy.shutdown() is not called here to avoid the "context already shutdown" error.

if __name__ == '__main__':
    main()
