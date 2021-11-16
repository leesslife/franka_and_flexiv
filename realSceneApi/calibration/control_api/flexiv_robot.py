"""Flexiv Robot SDK.

Author: Junfeng Ding
"""
import sys 
sys.path.append("./")
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

from control_api import lib_cxx
from control_api.pose import compute_angle_two_quat, compute_rot_z, pose_to_degree

from control_api.flexiv_api import FlexivApi, error_code


class ForceServer(threading.Thread):
    """Force Seg Record Class.

    This class spawns a thread for recording fixed length of force
    torques.
    """

    def __init__(self, lock, robot, force_seg, length):
        threading.Thread.__init__(self)
        self.force_seg = force_seg
        self.lock = lock
        self.length = length
        self.robot = robot
        self.count = 0

    def run(self):
        while True:
            robot_status_data = lib_cxx.FlexivRdkWrapper.RobotStates()
            self.robot.getRobotStates(robot_status_data)
            force = robot_status_data.m_tcpWrench[:6]
            time.sleep(0.5)
            self.lock.acquire()
            if len(self.force_seg) < self.length:
                force.append(time.time())
                self.force_seg.append(force)
            else:
                self.force_seg.pop(0)
                force.append(time.time())
                self.force_seg.append(force)
            self.count += 1
            time.sleep(0.001)
            self.lock.release()


class ForceClient(threading.Thread):
    """Force Seg Fetch Class.

    This class spawns a thread for fetching fixed length of force
    torques.
    """

    def __init__(self, lock, force_seg, length):
        threading.Thread.__init__(self)
        self.force_seg = force_seg
        self.lock = lock
        self.length = length

    def run(self):
        while True:
            self.lock.acquire()
            self.lock.release()


class FlexivRobot(FlexivApi):
    """Flexiv Robot Control Class.

    This class provides python wrapper for Flexiv robot control in different modes.
    Features include:
        - torque control mode
        - online move mode
        - plan execution mode
        - get robot status information
        - force torque record
    """

    logger_name = "FlexivRobot"

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
        super().__init__(robot_ip_address, pc_ip_address)

    def clear_fault(self, max_try_time=5):
        current_state = self.is_fault()
        tic = time.time()
        while self.is_fault:
            super().clear_fault()
            if time.time() - tic > max_try_time:
                break
        new_state = self.is_fault()
        if current_state and new_state:
            raise RuntimeError("[Error] clear fault failed")
        elif current_state and not new_state:
            self.logger.info("Successfully cleared fault. You are cool now.")

    def get_plan_info(self, attribute="m_ptName"):
        """get current robot's running plan info.

        Returns:
            name string of running node in plan

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        plan_info = super().get_plan_info()
        if attribute is None:
            return plan_info
        return str(getattr(plan_info, attribute))

    def is_reached(self, target, trans_epsilon=0.0002, rot_epsilon=0.5):
        """check if at target position.

        Args:
            target: 7-dim list or numpy array of 7D target pose (x,y,z,rw,rx,ry,rz)
            trans_epsilon: unit: meter, translation threshold to judge whether reach the target x,y,z
            rot_epsilon: unit: degree, rotation threshold to judge whether reach the target rotation degree

        Returns:
            True indicates reaching target pose, False indicates not.

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        target = np.array(target)
        current_pose = self.get_tcp_pose()
        if (
            compute_angle_two_quat(current_pose[3:], target[3:]) < rot_epsilon
            and sum(abs(current_pose[:3] - target[:3]) > trans_epsilon) == 0
        ):
            return True
        else:
            return False

    def is_moving(self, epsilon=1e-2):
        """check whether robot is moving or not.

        Args:
            epsilon: unit: degree/s, threshold to judge whether robot is moving

        Returns:
            True indicates robot is moving, False indicates robot is not running.

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        vel = abs(self.get_joint_vel())
        if sum(vel > epsilon) == 0:
            return False
        else:
            return True

    def move_torque(
        self,
        tcp,
        wrench=np.array([25.0, 25.0, 10.0, 20.0, 20.0, 20.0]),
        trans_epsilon=0.001,
        rot_epsilon=0.5,
    ):
        """move in torque control mode until reaching the target.

        Args:
            tcp: 7-dim list or numpy array, target pose (x,y,z,rw,rx,ry,rz) in world frame
            wrench: 6-dim list or numpy array, max moving force (fx,fy,fz,,wx,wy,wz)
            trans_epsilon: unit: meter, translation threshold to judge whether reach the target x,y,z
            rot_epsilon: unit: degree, rotation threshold to judge whether reach the target rotation degree

        Returns:
            None

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        self.switch_mode("torque")
        while not self.is_reached(tcp, trans_epsilon, rot_epsilon):
            self.robot.sendTcpPose(tcp, wrench)
            if self.is_fault():
                self.clear_fault()
                time.sleep(0.01)
                if self.is_fault():
                    self.logger.error("FAULT in move torque")
                    return
            time.sleep(0.1)
        self.logger.info("Reached")

    def move_tool_rotation(self, rot):
        """compute transformed pose after rotation in tool frame.

        Args:
            rot : 3-dim list or numpy array, (drx, dry, drz) relative to current tool frame

        Returns:
            7-dim list consisting of (x,y,z,rw,rx,ry,rz), transformed pose in world frame

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        tool_pose = self.get_tcp_pose()
        r1 = R.from_quat([tool_pose[4], tool_pose[5], tool_pose[6], tool_pose[3]])
        r2 = R.from_rotvec(np.array(rot))
        r = r1 * r2
        r = r.as_quat()

        tool_pose[3] = r[3]
        tool_pose[4] = r[0]
        tool_pose[5] = r[1]
        tool_pose[6] = r[2]

        return tool_pose

    def move_tool_trans(self, trans):
        """compute transformed pose after translation in tool frame.

        Args:
            trans : 3-dim list or numpy array, (dx, dy, dz) relative to current tool frame

        Returns:
            7-dim list consisting of (x,y,z,rw,rx,ry,rz), transformed pose in world frame

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        tool_pose = self.get_tcp_pose()
        r = R.from_quat([tool_pose[4], tool_pose[5], tool_pose[6], tool_pose[3]])
        delta = r.apply(trans)
        return np.array(
            [
                tool_pose[0] + delta[0],
                tool_pose[1] + delta[1],
                tool_pose[2] + delta[2],
                tool_pose[3],
                tool_pose[4],
                tool_pose[5],
                tool_pose[6],
            ]
        )

    def move_relative_trans(self, pose, trans):
        """compute translated pose relative to given pose in given pose's
        frame.

        Args:
            pose: 7-dim list consisting of (x,y,z,rw,rx,ry,rz), given pose in world frame
            trans : 3-dim list or numpy array, (dx, dy, dz) relative to current tool frame

        Returns:
            7-dim list consisting of (x,y,z,rw,rx,ry,rz), transformed pose in world frame

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        tool_pose = pose.copy()
        r = R.from_quat([tool_pose[4], tool_pose[5], tool_pose[6], tool_pose[3]])
        delta = r.apply(trans)
        return np.array(
            [
                tool_pose[0] + delta[0],
                tool_pose[1] + delta[1],
                tool_pose[2] + delta[2],
                tool_pose[3],
                tool_pose[4],
                tool_pose[5],
                tool_pose[6],
            ]
        )

    def move_relative_rotation(self, pose, rot):
        """compute translated pose relative to given pose in given pose's
        frame.

        Args:
            pose: 7-dim list consisting of (x,y,z,rw,rx,ry,rz), given pose in world frame
            rot : 3-dim list or numpy array, (rx, ry, rz) relative to current tool frame

        Returns:
            7-dim list consisting of (x,y,z,rw,rx,ry,rz), transformed pose in world frame

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        tool_pose = pose.copy()
        r1 = R.from_quat([tool_pose[4], tool_pose[5], tool_pose[6], tool_pose[3]])
        r2 = R.from_rotvec(np.array(rot))
        r = r1 * r2
        r = r.as_quat()

        tool_pose[3] = r[3]
        tool_pose[4] = r[0]
        tool_pose[5] = r[1]
        tool_pose[6] = r[2]

        return tool_pose

    def move_slide(
        self,
        target,
        force=-3,
        v=0.002,
        zv=0.0005,
        rv=0.5 / 180 * np.pi,
        trans_epsilon=0.0006,
        rot_epsilon=0.6,
        max_force=100,
        max_slide_step=100,
    ):
        """Move with constant z force is in world frame.

        Args:
            target: 7 dim (x,y,z,rw,rx,ry,rz) target position in world frame
            force: constant z force
            v: x,y velocity
            zv: z velocity
            trans_epsilon: threshold to judege whether reach target position
            rot_epsilon: threshold to judege whether reach target rotation
            max_force: max protect force
            max_slide_step: max slide step

        Returns:
            None
        """
        self.switch_mode("online")
        delay = 0.05
        slide_step = 0

        current_pose = self.get_tcp_pose()
        while (
            compute_angle_two_quat(self.get_tcp_pose()[3:], target[3:]) > rot_epsilon
            or sum(abs(self.get_tcp_pose()[:2] - target[:2]) > trans_epsilon) > 0
        ):
            current_force = self.get_tcp_force()
            while abs(current_force[2]) > max_force:
                current_force = self.get_tcp_force()
                self.logger.warning(
                    "Received force is so big: {}".format(current_force[2])
                )

            delta_z = (force - current_force[2]) * zv * delay
            dir_x = 1 if current_pose[0] < target[0] else -1
            dir_y = 1 if current_pose[1] < target[1] else -1
            r2 = R.from_quat(
                [
                    current_pose[4],
                    current_pose[5],
                    current_pose[6],
                    current_pose[3],
                ]
            )
            r_initial = R.from_quat(
                [
                    target[4],
                    target[5],
                    target[6],
                    target[3],
                ]
            )
            r = r2 * r_initial.inv()
            rz = r.as_euler("yzx", degrees=True)[1]
            dir_rz = 1 if rz < 0 else -1
            current_pose[0] += dir_x * v * delay
            current_pose[1] += dir_y * v * delay
            current_pose[2] += delta_z
            current_pose = compute_rot_z(current_pose, dir_rz * rv * delay)
            self.logger.info("move pose: {}".format(current_pose))
            self.send_online_pose(current_pose, 2, 4, 0.2, 1)
            time.sleep(delay)
            slide_step += 1
            if slide_step > max_slide_step:
                self.logger.info("out of slide_step")
                break
    
    def move_jnt_pos(
        self,
        target,
        max_jnt_vel=[6, 6, 7, 7, 14, 14, 14],
        max_jnt_acc=[3.60, 3.60, 4.20, 4.20, 8.40, 8.40, 8.40]
    ):
        """
        this function:
            for move to target pos by joint value
         max_jnt_vel: maximum joint velocity for each joint, 7d array or list
         max_jnt_acc: maximum joint acceleration for each joint, 7d array or list
         target: target joint value list, 7d array or list

        """
        self.switch_mode("primitive")
        if(len(target)!=7):
            print("the joint num is not 7,please setup right joint value")
            return -1
    
        
        command=("MoveJ(   maxJntVel={} {} {} {} {} {} {}, \
                           maxJntAcc={} {} {} {} {} {} {}, \
                           target=   {} {} {} {} {} {} {})"
                                        .format(
                                         max_jnt_vel[0],
                                         max_jnt_vel[1],
                                         max_jnt_vel[2],
                                         max_jnt_vel[3],
                                         max_jnt_vel[4],
                                         max_jnt_vel[5],
                                         max_jnt_vel[6],
                                         max_jnt_acc[0],
                                         max_jnt_acc[1],
                                         max_jnt_acc[2],
                                         max_jnt_acc[3],
                                         max_jnt_acc[4],
                                         max_jnt_acc[5],
                                         max_jnt_acc[6],
                                         target[0]*180/np.pi,
                                         target[1]*180/np.pi,
                                         target[2]*180/np.pi,
                                         target[3]*180/np.pi,
                                         target[4]*180/np.pi,
                                         target[5]*180/np.pi,
                                         target[6]*180/np.pi)
                )
        self.execute_primitive(command)
        time.sleep(0.1)
        state = self.get_primitive_states()
        while len(state) != 4 or state[2][-1] != "1":          
            try:         
                state = self.get_primitive_states()
            except Exception as e:
                print("state read i error,read state again!")
                state = self.get_primitive_states()
            time.sleep(0.1)
            #print(state)
            
        
    def move_ptp(
        self,
        target,
        waypoints=None,
        prefer_jnt_pos=[0.0, -40.0, 0.0, 90.0, 0.0, 40.0, 0.0],
        max_jnt_vel=[6, 6, 7, 7, 14, 14, 14],
        max_jnt_acc=[3.60, 3.60, 4.20, 4.20, 8.40, 8.40, 8.40],
        target_tol_level=1,
    ):
        """
        Args:
            max_jnt_vel: maximum joint velocity for each joint, 7d array or list
            max_jnt_acc: maximum joint acceleration for each joint, 7d array or list
            target: target tcp coordinate, 7d array or list
            waypoints: array list of waypoints between initial and target poses, e.g. [[0,0,0,0,0,0]]
            free_orientation: Free the orientation constraint
            prefer_jnt_pos: preferred target joint configuration. The robot is compliance with Cartesian constraint while reaching this configuration as close as possible.
            target_tol_level: tolerance level of target pose accuracy ranged from 1 to 10, where 1 is the smallest tolerence

        Returns: None
        """
        self.switch_mode("primitive")
        target = pose_to_degree(target, True)
        waypoints_str = "waypoints = "
        if waypoints is not None:
            for i in range(len(waypoints)):
                waypoint = pose_to_degree(waypoints[i], True)
                waypoints_str += (
                    "waypoints = {} {} {} {} {} {} WORLD WORLD_ORIGIN,".format(
                        waypoint[0],
                        waypoint[1],
                        waypoint[2],
                        waypoint[3],
                        waypoint[4],
                        waypoint[5],
                    )
                )
        command = (
            "MovePTP(maxJntVel= {} {} {} {} {} {} {}, \
                                maxJntAcc= {} {} {} {} {} {} {}, \
                                target= {} {} {} {} {} {} WORLD WORLD_ORIGIN, \
                                preferJntPos= {} {} {} {} {} {} {}, targetTolLevel= {},".format(
                max_jnt_vel[0],
                max_jnt_vel[1],
                max_jnt_vel[2],
                max_jnt_vel[3],
                max_jnt_vel[4],
                max_jnt_vel[5],
                max_jnt_vel[6],
                max_jnt_acc[0],
                max_jnt_acc[1],
                max_jnt_acc[2],
                max_jnt_acc[3],
                max_jnt_acc[4],
                max_jnt_acc[5],
                max_jnt_acc[6],
                target[0],
                target[1],
                target[2],
                target[3],
                target[4],
                target[5],
                prefer_jnt_pos[0],
                prefer_jnt_pos[1],
                prefer_jnt_pos[2],
                prefer_jnt_pos[3],
                prefer_jnt_pos[4],
                prefer_jnt_pos[5],
                prefer_jnt_pos[6],
                target_tol_level,
            )
            + waypoints_str
            + ")"
        )
        self.execute_primitive(command)
        time.sleep(0.1)
        state = self.get_primitive_states()
        while len(state) != 4 or state[2][-1] != "1":
            time.sleep(0.01)
            try:         
                state = self.get_primitive_states()
            except Exception as e:
                print("state read i error,read state again!")
                state = self.get_primitive_states()
            #print(state)

    def cali_force_sensor(self, data_collection_time=0.2):
        """calibrate force sensor.

        Args:
            data_collection_time: time period of the force offset estimation, unit: s

        Returns: None
        """
        self.switch_mode("primitive")
        self.execute_primitive(
            "CaliForceSensor(dataCollectionTime={})".format(data_collection_time)
        )
        time.sleep(data_collection_time + 2)

    def contact(
        self,
        contact_vel=0.02,
        coord="world",
        moving_dir=[0, 0, -1],
        max_contact_force=5,
    ):
        """move until contact.

        Args:
            contact_vel: robot Cartesian moving velocity to contact environment, unit: m/s
            coord: coordinate of contact moving direction, "world" or "tcp"
            moving_dir: moving direction in defined coordinate, array or list
            max_contact_force: absolute value of the maximum total force, threshold to terminate contact, unit: N

        Returns: None
        """
        self.switch_mode("primitive")
        self.execute_primitive(
            "Contact(contactVel= {}, coord={}, movingDir= {} {} {}, maxContactForce= {})".format(
                contact_vel,
                coord,
                moving_dir[0],
                moving_dir[1],
                moving_dir[2],
                max_contact_force,
            )
        )

    def align_contact(
        self,
        contact_dir=[0, 0, 1],
        alignment_dir=[1, 1, 0, 1, 1, 0],
        target_force=5,
        deadband_scale=0.5,
        freespace_vel=0.01,
        align_vel_scale=0.2,
        ori_align_mode=0,
    ):
        """align contact primitive.

        Args:
            contact_dir: contact axis in tool coordinate. It could be 0 or 1 for each axis. if set as 1, the robot would make force control on cooresponding axis
            alignment_dir: alignment direction in tool coordinate. It could be 0 or 1 for each axis. if set as 1, the robot would be force compliant in cooresponding axis. For example, (0,1,0,1,0,0) means compliant in linear y and rotation x axis
            target_force: target contact force int tcp coordinate that robot output on pre-set force control axis
            deadband_scale: robot alignment wrench deadband scale. 0 means no deadband. Robot would not adjust its position or orientation when external wrench torque is smaller than this deadband
            freespace_vel: velocity in free space
            align_vel_scale: scale of alignment speed to observed external wrench. Robot would adjust its pose faster when this valuse is bigger, but may become less stable. The default value is recommended.
            ori_align_mode: orientation alignment mode. 0 for position admittance mode, 1 for torque admittance mode. The former works better when external torque is small and varies slow， and the latter works better when external torque is large and varies fast

        Returns: None
        """
        self.switch_mode("primitive")
        self.execute_primitive(
            "Aligncontact(contactDir= {} {} {}, alignmentDir= {} {} {} {} {} {}, targetForce= {}, deadbandScale= {}, \
                freeSpaceVel= {}, alignVelScale= {}, oriAlignMode= {})".format(
                contact_dir[0],
                contact_dir[1],
                contact_dir[2],
                alignment_dir[0],
                alignment_dir[1],
                alignment_dir[2],
                alignment_dir[3],
                alignment_dir[4],
                alignment_dir[5],
                target_force,
                deadband_scale,
                freespace_vel,
                align_vel_scale,
                ori_align_mode,
            )
        )

    def start_force_record(self, length, model_path=""):
        """start force seg record.

        Args:
            length: int, force seg length
            model_path: string, insert detection model path

        Returns:
            None

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        lock = threading.Lock()
        self.force_seg = []
        server = ForceServer(lock, self.robot, self.force_seg, length)
        server.start()
        client = ForceClient(lock, self.force_seg, length)
        client.start()

    def wait_until_execution_begin(self):
        system_status = self._get_system_status()
        # wait until execution begin
        while not system_status.m_programRunning:
            self.robot.getSystemStatus(system_status)
            time.sleep(1)

    def wait_until_execution_stop(self):
        system_status = self._get_system_status()
        # wait until execution begin
        while system_status.m_programRunning:
            self.robot.getSystemStatus(system_status)
            time.sleep(1)

    def execute_plan_by_name(
        self,
        name,
        end_node_path=None,
        max_time=None,
        end_node_path_iter_num=1,
        block=True,
    ):
        """execute plan by name, make sure control mode is switched into plan.

        Args:
            name: string of plan name
            end_node_path: string, end plan execution after finishing node name
            max_time: float, max time for plan to execute

        Returns:
            error_code: int

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        super().execute_plan_by_name(name)
        system_status = self._get_system_status()
        self.logger.info("Plan started: %s", name)
        if not block:
            return error_code.ok
        tic = time.time()
        if end_node_path:
            check = False
        # wait until execution finished
        while system_status.m_programRunning:
            self.robot.getSystemStatus(system_status)
            if self.is_fault():
                self.logger.error(
                    "system fault. emergency stop = %s, joint limit triggered = %s, error message = %s",
                    system_status.m_emergencyStop,
                    system_status.m_jntLimitTriggered,
                    system_status.m_errorMsg,
                )
                self.clear_fault()
                return error_code.system_fault
            if end_node_path:
                current_node_path = self.get_plan_info(attribute="m_nodePath")
                self.logger.debug("Node: {}".format(str(current_node_path)))
                if current_node_path == end_node_path:
                    check = True
                elif check and current_node_path != end_node_path:
                    check = False
                    end_node_path_iter_num -= 1
                elif end_node_path_iter_num == 0 and check is False:
                    self.stop()
                    self.wait_until_execution_stop()
                    break

                if max_time and time.time() - tic > max_time:
                    self.logger.error(
                        "plan stoped, execution exceeds max time: %s", max_time
                    )
                    self.stop()
                    self.wait_until_execution_stop()
                    return error_code.exceeds_max_time
                time.sleep(0.01)
            else:
                time.sleep(1)

        if self.is_fault():
            self.logger.error(
                "system fault after run. emergency stop = %s, joint limit triggered = %s, error message = %s",
                system_status.m_emergencyStop,
                system_status.m_jntLimitTriggered,
                system_status.m_errorMsg,
            )
            self.clear_fault()
            return error_code.system_fault

        self.logger.info("Plan stopped safely")
        return error_code.ok

    def begin_plot(self):
        """turn on plot window.

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        plt.ion()
        plt.clf()

    def plot_force_seg(self, force_seg):
        """visualize 6-axis force seg in six windows.

        Args:
            force_seg:

        Returns:
            None

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        plt.clf()
        fx = [f[0] for f in force_seg]
        fy = [f[1] for f in force_seg]
        fz = [f[2] for f in force_seg]
        frx = [f[3] for f in force_seg]
        fry = [f[4] for f in force_seg]
        frz = [f[5] for f in force_seg]
        x = np.arange(len(force_seg))
        ax0 = plt.subplot(2, 3, 1)
        ax1 = plt.subplot(2, 3, 2)
        ax2 = plt.subplot(2, 3, 3)
        ax3 = plt.subplot(2, 3, 4)
        ax4 = plt.subplot(2, 3, 5)
        ax5 = plt.subplot(2, 3, 6)
        ax0.plot(x, fx)
        ax1.plot(x, fy)
        ax2.plot(x, fz)
        ax3.plot(x, frx)
        ax4.plot(x, fry)
        ax5.plot(x, frz)
        plt.pause(0.4)

    def end_plot(self):
        """close plot window.

        Returns:
            None

        Raises:
            RuntimeError: error occurred when mode is None.
        """
        plt.ioff()
        plt.show()
