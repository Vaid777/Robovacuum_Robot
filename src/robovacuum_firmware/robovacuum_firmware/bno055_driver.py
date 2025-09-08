#!/usr/bin/env python3
import rclpy.time
import smbus
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

# BNO055 Register Addresses
BNO055_CHIP_ID_ADDR         = 0x00
BNO055_OPR_MODE_ADDR        = 0x3D
BNO055_PWR_MODE_ADDR        = 0x3E
BNO055_SYS_TRIGGER_ADDR     = 0x3F

# Operation Modes
BNO055_OPERATION_MODE_CONFIG    = 0x00
BNO055_OPERATION_MODE_NDOF      = 0x0C  # 9DOF fusion mode

# Data registers for fusion data
BNO055_QUATERNION_DATA_W_LSB_ADDR   = 0x20
BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0x28
BNO055_GYRO_DATA_X_LSB_ADDR         = 0x14

# Unit selection register
BNO055_UNIT_SEL_ADDR = 0x3B

# I2C Address (can be 0x28 or 0x29 depending on COM3 pin)
BNO055_ADDRESS = 0x28  # Default address when COM3 is LOW

class BNO055_Driver(Node):
    def __init__(self):
        super().__init__("bno055_driver")
        
        # I2C Interface
        self.is_connected_ = False
        self.init_i2c()
        
        # ROS 2 Interface
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "imu_link"  # Changed from base_footprint
        
        # Set covariance matrices (BNO055 provides fused data with better accuracy)
        self.imu_msg_.orientation_covariance[0] = 0.01
        self.imu_msg_.orientation_covariance[4] = 0.01
        self.imu_msg_.orientation_covariance[8] = 0.01
        
        self.imu_msg_.angular_velocity_covariance[0] = 0.01
        self.imu_msg_.angular_velocity_covariance[4] = 0.01
        self.imu_msg_.angular_velocity_covariance[8] = 0.01
        
        self.imu_msg_.linear_acceleration_covariance[0] = 0.01
        self.imu_msg_.linear_acceleration_covariance[4] = 0.01
        self.imu_msg_.linear_acceleration_covariance[8] = 0.01
        
        self.frequency_ = 0.02  # 50Hz (BNO055 can handle up to 100Hz)
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)
        
        self.get_logger().info("BNO055 IMU Driver initialized")

    def timerCallback(self):
        try:
            if not self.is_connected_:
                self.init_i2c()
                return
            
            # Read quaternion data (4 values: w, x, y, z)
            quat_w = self.read_16bit_data(BNO055_QUATERNION_DATA_W_LSB_ADDR)
            quat_x = self.read_16bit_data(BNO055_QUATERNION_DATA_W_LSB_ADDR + 2)
            quat_y = self.read_16bit_data(BNO055_QUATERNION_DATA_W_LSB_ADDR + 4)
            quat_z = self.read_16bit_data(BNO055_QUATERNION_DATA_W_LSB_ADDR + 6)
            
            # Read linear acceleration data (m/s²)
            accel_x = self.read_16bit_data(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR)
            accel_y = self.read_16bit_data(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR + 2)
            accel_z = self.read_16bit_data(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR + 4)
            
            # Read gyroscope data (rad/s)
            gyro_x = self.read_16bit_data(BNO055_GYRO_DATA_X_LSB_ADDR)
            gyro_y = self.read_16bit_data(BNO055_GYRO_DATA_X_LSB_ADDR + 2)
            gyro_z = self.read_16bit_data(BNO055_GYRO_DATA_X_LSB_ADDR + 4)
            
            # Convert quaternion values (BNO055 uses 1 LSB = 1/16384)
            self.imu_msg_.orientation.w = quat_w / 16384.0
            self.imu_msg_.orientation.x = quat_x / 16384.0
            self.imu_msg_.orientation.y = quat_y / 16384.0
            self.imu_msg_.orientation.z = quat_z / 16384.0
            
            # Convert acceleration values (BNO055: 1 LSB = 0.01 m/s² for linear acceleration)
            self.imu_msg_.linear_acceleration.x = accel_x / 100.0
            self.imu_msg_.linear_acceleration.y = accel_y / 100.0
            self.imu_msg_.linear_acceleration.z = accel_z / 100.0
            
            # Convert gyroscope values (BNO055: 1 LSB = 1/16 dps, convert to rad/s)
            dps_to_rads = 3.14159265359 / 180.0 / 16.0
            self.imu_msg_.angular_velocity.x = gyro_x * dps_to_rads
            self.imu_msg_.angular_velocity.y = gyro_y * dps_to_rads
            self.imu_msg_.angular_velocity.z = gyro_z * dps_to_rads
            
            # Update timestamp and publish
            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub_.publish(self.imu_msg_)
            
        except OSError as e:
            self.get_logger().error(f"I2C communication error: {e}")
            self.is_connected_ = False

    def init_i2c(self):
        try:
            self.bus_ = smbus.SMBus(1)  # I2C bus 1
            
            # Check chip ID to verify connection
            chip_id = self.bus_.read_byte_data(BNO055_ADDRESS, BNO055_CHIP_ID_ADDR)
            if chip_id != 0xA0:  # BNO055 chip ID should be 0xA0
                self.get_logger().error(f"Wrong chip ID: {hex(chip_id)}, expected 0xA0")
                self.is_connected_ = False
                return
            
            # Switch to config mode
            self.bus_.write_byte_data(BNO055_ADDRESS, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG)
            rclpy.time.sleep(0.025)  # Wait 25ms for mode switch
            
            # Reset system
            self.bus_.write_byte_data(BNO055_ADDRESS, BNO055_SYS_TRIGGER_ADDR, 0x20)
            rclpy.time.sleep(0.65)   # Wait 650ms for reset
            
            # Set power mode to normal
            self.bus_.write_byte_data(BNO055_ADDRESS, BNO055_PWR_MODE_ADDR, 0x00)
            rclpy.time.sleep(0.010)
            
            # Set unit selection (acceleration: m/s², angular rate: rad/s, euler: degrees)
            self.bus_.write_byte_data(BNO055_ADDRESS, BNO055_UNIT_SEL_ADDR, 0x01)
            rclpy.time.sleep(0.010)
            
            # Switch to NDOF mode (9-axis fusion)
            self.bus_.write_byte_data(BNO055_ADDRESS, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF)
            rclpy.time.sleep(0.020)
            
            self.is_connected_ = True
            self.get_logger().info("BNO055 initialized successfully in NDOF mode")
            
        except OSError as e:
            self.get_logger().error(f"Failed to initialize BNO055: {e}")
            self.is_connected_ = False

    def read_16bit_data(self, addr):
        """Read 16-bit signed data from BNO055 (little-endian format)"""
        try:
            # Read low and high bytes
            low = self.bus_.read_byte_data(BNO055_ADDRESS, addr)
            high = self.bus_.read_byte_data(BNO055_ADDRESS, addr + 1)
            
            # Combine bytes (little-endian)
            value = (high << 8) | low
            
            # Convert to signed 16-bit value
            if value > 32767:
                value = value - 65536
            
            return value
        except OSError:
            raise

def main():
    rclpy.init()
    bno055_driver = BNO055_Driver()
    try:
        rclpy.spin(bno055_driver)
    except KeyboardInterrupt:
        pass
    finally:
        bno055_driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
