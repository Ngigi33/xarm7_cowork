from xarm import version
from xarm.wrapper import XArmAPI
from xarm_msgs.msg import JointNamesAndAngles
from datetime import datetime
import rclpy
from rclpy.node import Node
import time
import numpy as np


class JointInfoPublisher(Node):
    def __init__(self, robot):
        super().__init__('joint_info_publisher')
        self.freq = 60.0    # Publish frequency
        self._arm = robot
       # Create publisher
        self.joint_info_publisher = self.create_publisher(JointNamesAndAngles, '/joint_info', 10)
        
        timer_period = 1/self.freq  # seconds
        self._timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f'Joint Publisher Node started - Publishing at {self.freq}Hz')

    # Robot init
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)

 # Register error/warn changed callback
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # Register state changed callback
    def _state_changed_callback(self, data):
        if not self._ignore_exit_state and data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._ignore_exit_state:
                return True
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    def timer_callback(self):
        if not self._arm.connected:
            return

        joint_info_msg = JointNamesAndAngles()
        joint_info_msg.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        # joint_info_msg.names = [f'joint_{i+1}' for i in range(self._arm.num_joints)]
        joint_info_msg.names = []
        joint_info_msg.positions = [angle for angle in self._arm.angles]
        # joint_info_msg.positions = joint_info_msg.positions.append((float)(self._arm.get_gripper_position()[1]))
        # print(type(self._arm.get_gripper_position()[1]))
        # print(type(joint_info_msg.po)
        joint_info_msg.velocities = []
        joint_info_msg.tcp_position = []

        self.joint_info_publisher.publish(joint_info_msg)



def main(args=None):
    rclpy.init(args=args)
    JointInfoPublisher.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))

    # Initialize robot
    arm = XArmAPI('172.16.40.20', baud_checkset=False)
    rclpy.spin(JointInfoPublisher(arm))
    JointInfoPublisher.get_logger().info('Shutting down Joint State Publisher Node')
    JointInfoPublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()