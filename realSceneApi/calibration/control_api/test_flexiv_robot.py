#!/usr/bin/python3.6


import sys

sys.path.insert(0, ".")

from base import Logging  # noqa
from flexiv_robot import FlexivRobot
import numpy as np
logger = Logging.init_app_logger("debug")


def main():
    client = FlexivRobot("192.168.2.100", "192.168.2.110")
    print(client.get_tcp_pose())
    print(client.get_joint_pos())
    client.send_tcp_pose(np.array([6.73251908e-01,-1.12604752e-01,3.01008642e-01,1.51821217e-02,-2.93194270e-03,9.99880314e-01,-5.68694260e-04]))
    
    #client.switch_mode("idle")
    # client.switch_mode("online")
    # client.switch_mode("torque")
    # client.switch_mode(mode="plan")
    # client.clear_fault()
    # client.execute_plan_by_name("PLAN-Home")
    # client.execute_plan_by_name("TestEasyMove", max_time=1)
    # client.execute_plan_by_name("TestEasyMove", end_node_path="rooNode::move0")
    # client.execute_plan_by_name(
    #     "TestEasyMove", end_node_path="rooNode::move0", end_node_path_iter_num=1
    # )
    # client.execute_plan_by_name("PLAN-Home")

    # state = client.get_emergency_state()
    # logger.info("robot state: %s", state)

    # tcp_pose = client.get_tcp_pose()
    # logger.info("tcp pose: %s", tcp_pose)

    # tcp_force = client.get_tcp_force()
    # logger.info("tcp force: %s", tcp_force)

    # camera_pose_list = client.get_camera_pose()
    # camera_pose = client.get_camera_pose(camera_id=0)
    # logger.info("camera pose list: %s", str(camera_pose_list))
    # logger.info("camera pose of camera id = 0: %s", camera_pose)

    # plan_info = client.get_plan_info()
    # logger.info("current plan info: %s", str(plan_info))

    # plan_name_list = client.get_plan_name_list()
    # logger.info("plan name list: %s", str(plan_name_list))


if __name__ == "__main__":
    main()
