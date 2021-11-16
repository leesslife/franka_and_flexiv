"""Flexiv robot ethernet communication python wrapper api.

Author: Junfeng Ding
"""

import time

import numpy as np

from control_api.base import Logging
from control_api import lib_cxx

from control_api.error_config import code as error_code

LOG_LEVEL = "info"


class ModeMap:
    idle = "CTRL_MODE_IDLE"
    plan = "CTRL_MODE_PLAN_EXECUTION"
    primitive = "CTRL_MODE_PRIMITIVE_EXECUTION"
    torque = "CTRL_MODE_CARTESIAN_SLAVEMODE"
    online = "CTRL_MODE_CARTESIAN_ONLINEMOVE"
    joint = "CTRL_MODE_JOINT_POSITION"
    impedance = "CTRL_MODE_CARTESIAN_IMPEDANCE"


class FlexivApi:
    """Flexiv Robot Control Class.

    This class provides python wrapper for Flexiv robot control in different modes.
    Features include:
        - torque control mode
        - online move mode
        - plan execution mode
        - primitive excution mode
        - get robot status information
        - force torque record
    """

    logger_name = "FlexivApi"

    def __init__(self, robot_ip_address, pc_ip_address):
        """initialize.

        Args:
            robot_ip_address: robot_ip address string
            pc_ip_address: pc_ip address string

        Returns:
            None

        Raises:
            RuntimeError: error occurred when ip_address is None.
        """
        self.error = error_code.ok
        self.logger = Logging.init_lib_logger(self.logger_name, LOG_LEVEL)
        self.robot = lib_cxx.FlexivRdkWrapper.RobotClientHandler()
        ret = self.robot.init(robot_ip_address, pc_ip_address)
        if ret != lib_cxx.FvrParamsWrapper.FvrStg.FVR_OKg:
            self.logger.info("Init robot failed.")
            self.error = error_code.robot_lost
        else:
            time.sleep(0.6)
            self.extCtrlMode = lib_cxx.FlexivRdkWrapper.ControlMode
            error = self.enable()
            if error:
                self.logger.error("Robot init failed. Error: {}".format(error))
                raise RuntimeError("Robot init failed. Error: {}".format(error))
            self.logger.info(" init success")

    def enable(self, max_time=10):
        """Enable robot after emergency button is released."""
        self.robot.enable()
        tic = time.time()
        while not self.is_operational():
            if time.time() - tic > max_time:
                return error_code.robot_enable_failed
            time.sleep(0.01)
        return error_code.ok

    def _get_robot_status(self):
        robot_status_data = lib_cxx.FlexivRdkWrapper.RobotStates()
        self.robot.getRobotStates(robot_status_data)
        return robot_status_data

    def _get_system_status(self):
        system_status_data = lib_cxx.FlexivRdkWrapper.SystemStatus()
        self.robot.getSystemStatus(system_status_data)
        #print(system_status_data)
        return system_status_data

    def mode_mapper(self, mode):   # 控制模式与对应的ModeMap字段中的内容
        assert mode in ModeMap.__dict__.keys(), "unknown mode name: %s" % mode
        return getattr(self.extCtrlMode, getattr(ModeMap, mode))

    def get_control_mode(self):
        return self.robot.getControlMode()

    def set_control_mode(self, mode):
        control_mode = self.mode_mapper(mode)
        self.robot.setControlMode(control_mode)

    def switch_mode(self, mode, sleep_time=0.01):
        """switch to different control modes.

        Args:
            mode: 'torque', 'online', 'plan', 'idle', 'line', 'joint', 'primitive', 'impedance'
            sleep_time: sleep time to control mode switch time

        Returns:
            None

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        if self.get_control_mode() == self.mode_mapper(mode):
            return

        while self.get_control_mode() != self.mode_mapper("idle"):
            self.set_control_mode("idle")
            time.sleep(sleep_time)

        while self.get_control_mode() != self.mode_mapper(mode):
            self.set_control_mode(mode)
            time.sleep(sleep_time)

        self.logger.info("set mode: {}".format(str(self.get_control_mode())))

    def get_emergency_state(self):
        """get robot is emergency stopped or not. The emergency state means the
        E-Stop is pressed. instead of being soft fault.

        Returns:
            True indicates robot is not stopped, False indicates robot is emergency stopped.

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return not self._get_system_status().m_emergencyStop

    def clear_fault(self):
        self.robot.clearFault()

    def is_fault(self):
        """Check if robot is in FAULT state."""
        return self.robot.isFault()

    def is_stopped(self):
        """Check if robot is stopped."""
        return self.robot.isStopped()

    def is_connected(self):
        """return if connected.

        Returns: True/False
        """
        return self.robot.isConnected()

    def is_operational(self):
        """Check if robot is operational."""
        return self.robot.isOperational()

    def target_reached(self):
        """Only work when using move related api in EthernetConnection."""
        return np.array(self._get_system_status().m_reachedTarget)

    def get_tcp_pose(self):
        """get current robot's tool pose in world frame.

        Returns:
            7-dim list consisting of (x,y,z,rw,rx,ry,rz)

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().m_tcpPose)

    def get_tcp_vel(self):
        """get current robot's tool velocity in world frame.

        Returns:
            7-dim list consisting of (vx,vy,vz,vrw,vrx,vry,vrz)

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().m_tcpVel)

    def get_tcp_acc(self):
        """get current robot's tool acceleration in world frame.

        Returns:
            7-dim list consisting of (ax,ay,az,arw,arx,ary,arz)

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().m_tcpAcc)

    def get_tcp_force(self):
        """get current robot's tool force torque wrench.

        Returns:
            6-dim list consisting of (fx,fy,fz,wx,wy,wz)

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().m_tcpWrench[:6])

    def get_camera_pose(self, camera_id: int = None):
        """get current wrist camera pose in world frame.

        Returns:
            7-dim list consisting of (x,y,z,rw,rx,ry,rz)

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        # TODO: (chongzhao) check ethernet connection api for camera pose
        if camera_id is None:
            return np.array(self._get_robot_status().m_camPose)
        else:
            return np.array(self._get_robot_status().m_camPose[camera_id])

    def get_flange_pose(self):
        """get current flange pose in world frame.

        Returns:
            7-dim list consisting of (x,y,z,rw,rx,ry,rz)

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().m_flangePose)

    def get_end_link_pose(self):
        """get current end link pose in world frame.

        Returns:
            7-dim list consisting of (x,y,z,rw,rx,ry,rz)

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().m_endLinkPose)

    def get_joint_pos(self):
        """get current joint value.

        Returns:
            7-dim numpy array of 7 joint position

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().m_linkPosition)

    def get_joint_vel(self):
        """get current joint velocity.

        Returns:
            7-dim numpy array of 7 joint velocity

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return np.array(self._get_robot_status().m_linkVelocity)

    def get_joint_torque(self):
        """get measured link-side joint torque.

        Returns:
            7-dim numpy array of 7 link torques
        """
        return np.array(self._get_robot_status().m_linkTorque)

    def get_external_joint_torque(self):
        """get estimated EXTERNAL link-side joint torque.

        Returns:
            7-dim numpy array of 7 link torques
        """
        return np.array(self._get_robot_status().m_linkTorqueExt)

    def get_primitive_states(self):
        """get current primitive states.

        Returns:
            string of primitive states, including 'terminated', 'timePeriod', 'reachedTarget' and so on

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        return self._get_system_status().m_ptStates

    def get_plan_info(self, attribute="m_ptName"):
        """get current robot's running plan info.

        Returns:
            name string of running node in plan

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        plan_info = lib_cxx.FlexivRdkWrapper.PlanInfo()
        self.robot.getPlanInfo(plan_info)
        return plan_info

    def get_plan_name_list(self):
        plan_list = []
        self.robot.getPlanNameList(plan_list)
        return plan_list

    def write_io(self, port, value):
        """Set io value.

        Args:
            port: 0~15
            value: True/False
        """
        assert port >= 0 and port <= 15
        self.robot.writeDigitalOutput(port, value)

    def execute_primitive(self, cmd):
        """Execute primitive.

        Args:
            cmd: primitive command string, e.x. "ptName(inputParam1=xxx, inputParam2=xxx, ...)"
        """
        self.switch_mode("primitive")
        self.logger.info("Execute primitive: {}".format(cmd))
        lib_cxx.FvrUtilsWrapper.checkError(self.robot.executePrimitive(cmd))

    def stop(self):
        """Stop current motion and switch mode to idle."""
        self.robot.stop()
        while self.get_control_mode() != self.mode_mapper("idle"):
            time.sleep(0.005)

    def send_tcp_pose(self, tcp, wrench=np.array([25.0, 25.0, 10.0, 20.0, 20.0, 20.0])):
        """make robot move towards target pose in torque control mode,
        combining with sleep time makes robot move smmothly.

        Args:
            tcp: 7-dim list or numpy array, target pose (x,y,z,rw,rx,ry,rz) in world frame
            wrench: 6-dim list or numpy array, max moving force (fx,fy,fz,wx,wy,wz)

        Returns:
            None

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        self.switch_mode("torque")
        pose = np.array(tcp)
        self.robot.sendTcpPose(pose, wrench)

    def send_online_pose(self, tcp, max_v=0.05, max_a=0.1, max_w=0.2, max_dw=1):
        """make robot move towards target pose in online mode without reach
        check.

        Args:
            tcp: 7-dim list or numpy array, target pose (x,y,z,rw,rx,ry,rz) in world frame
            max_v: double, max linear velocity
            max_a: double, max linear acceleration
            max_w: double, max angular velocity
            max_dw: double, max angular acceleration

        Returns:
            None

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        self.switch_mode("online")
        tcp = np.array(tcp)
        self.robot.sendTcpPose(tcp, [0, 0, 0, 0, 0, 0], max_v, max_a, max_w, max_dw)

    def move_online(
        self,
        tcp,
        max_v=0.05,
        max_a=0.2,
        max_w=1.5,
        max_dw=2.0,
        trans_epsilon=0.001,
        rot_epsilon=0.5,
    ):
        """move in online mode until reaching the target.

        Args:
            tcp: 7-dim list or numpy array, target pose (x,y,z,rw,rx,ry,rz) in world frame
            max_v: double, max linear velocity
            max_a: double, max linear acceleration
            max_w: double, max angular velocity
            max_dw: double, max angular acceleration
            trans_epsilon: unit: meter, translation threshold to judge whether reach the target x,y,z
            rot_epsilon: unit: degree, rotation threshold to judge whether reach the target rotation degree

        Returns:
            None

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        self.switch_mode("online")
        while not self.is_reached(tcp, trans_epsilon, rot_epsilon):
            self.robot.sendTcpPose(tcp, [0, 0, 0, 0, 0, 0], max_v, max_a, max_w, max_dw)
            if self.is_fault():
                self.clear_fault()
                time.sleep(0.5)
                if self.is_fault():
                    self.logger.error("FAULT in moveOnline")
                    return
            time.sleep(0.1)
        self.logger.info("Reached")

    def execute_plan_by_name(
        self,
        name,
    ):
        """execute plan by name, make sure control mode is switched into plan.

        Args:
            name: string of plan name

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        self.switch_mode("plan")
        lib_cxx.FvrUtilsWrapper.checkError(self.robot.executePlanByName(name))

    def execute_plan_by_index(self, index):
        """execute plan by index, make sure control mode is switched into plan.

        Args:
            index: int, index of plan

        Returns:
            None

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        self.switch_mode("plan")
        lib_cxx.FvrUtilsWrapper.checkError(self.robot.executePlanByIndex(index))
