#!/usr/bin/python
"""
PandaHandler
take panda-startup script from armer-panda and ros-ified.
Takes in rosservice calls and makes lock/unlock brake and home gripper calls via https connection to desk API
"""
import requests
import threading
import signal
import sys
import json
import urllib3
from http import client

import rospy
import actionlib

from armer_panda.PandaConnector import PandaConnector
from franka_msgs.msg import FrankaState
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse

class PandaHandlerNode():

  def __init__(self):
    self.panda_connector = None

  def on_shutdown(self):
    """ handles cleanup on shutdown of node """
    if self.panda_connector is not None:
      
      while (self.panda_connector.calling):
        rospy.sleep(0.5)

      if not self.panda_connector.close_brakes():
        rospy.logerr("Unable to close brakes")
      else:
        rospy.loginfo("Brakes Closed")

      if not self.panda_connector.disable_fci():
        rospy.logerr("Unable to disable FCI")
      else:
        rospy.loginfo("FCI disabled")
      
      if self.panda_connector.is_control_token_active() and not self.panda_connector.release_control_token():
        rospy.logerr("Unable to release control authority, forced override may be required")
      else:
        rospy.loginfo("Control Authority Released")
      
      self.panda_connector.logout()

  def handle_acquire_control(self, req):
    """ROS Service to handle acquisition of control authority"""
    if self.panda_connector.can_control():
      return SetBoolResponse(True, "Node already has control authority")
    elif not self.panda_connector.acquire_control_token(force=req.data):
      return SetBoolResponse(False, "Unable to acquire control authority, robot will not respond to commands")
    else:
      rospy.loginfo("Acquired control authority")
      return SetBoolResponse(True, "Control authority acquired, enable open brakes and enable fci to start control")

  def handle_release_control(self, req):
    """ROS Service to handle acquisition of control authority"""
    if not self.panda_connector.can_control():
      return TriggerResponse(True, "Robot does not have control authority")
    if not self.panda_connector.control_token_req:
      return TriggerResponse(True, "Robot doesn't require control authority")
    elif not self.panda_connector.release_control_token():
      return TriggerResponse(False, "Unable to release control authority, override may be required")
    else:
      rospy.loginfo("Released control authority")
      return TriggerResponse(True, "Control authority released, robot will not respond to commands")

  def handle_enable_fci(self, req):
    """ROS Service to handle enabling of fci"""
    if not self.panda_connector.is_control_token_active():
      return TriggerResponse(False, "Node does not have control authority, cannot activate fci")
    elif not self.panda_connector.enable_fci():
      return TriggerResponse(False, "Unable to activate FCI")
    else:
      rospy.loginfo("FCI Activated")
      return TriggerResponse(True, "FCI Activated, ros communications enabled")

  def handle_disable_fci(self, req):
    """ROS Service to handle enabling of fci"""
    if not self.panda_connector.is_control_token_active():
      return TriggerResponse(False, "Node does not have control authority, cannot disable fci")
    elif not self.panda_connector.disable_fci():
      return TriggerResponse(False, "Unable to disable FCI")
    else:
      rospy.loginfo("FCI Disabled")
      return TriggerResponse(True, "FCI Disabled, ros control prohibited")

  def handle_open_brakes(self, req):
    """ROS Service to handle opening arm brakes"""
    if self.panda_connector.open_brakes():
      rospy.logwarn("WARNING: PANDA BRAKES OPENED, APPROACH ROBOT WITH CATION!")
      return TriggerResponse(True, "API Reports Brakes opened. WARNING: APPROACH ROBOT WITH CATION")
    else:
      return TriggerResponse(False, "Unable to open Brakes!")

  def handle_close_brakes(self, req):
    """ROS Service to handle closing arm brakes"""
    if self.panda_connector.close_brakes():
      rospy.loginfo("Brakes closed")
      return TriggerResponse(True, "API Reports Brakes closed")
    else:
      return TriggerResponse(False, "Unable to close Brakes!")

  def handle_home_gripper(self, req):
    if self.panda_connector.home_gripper():
      rospy.loginfo("Gripper homing sequence requested")
      return TriggerResponse(True, "API Reports Gripper homed")
    else:
      return TriggerResponse(False, "Unable to home Gripper")

  def main(self):
    """ Main function for the panda connector. Gets connection info and exposes services """
    rospy.init_node('panda_handler', anonymous=True)
    rospy.on_shutdown(self.on_shutdown)

    # get config from param server
    robot_ip = rospy.get_param('~robot_ip', '192.168.0.1')
    username = rospy.get_param('~username', 'franka')
    password = rospy.get_param('~password', 'MjMsMTY0LDIzMSwxNjIsMTM4LDEyMCwxOTQsNTUsMTk2LDI5LDE5Myw5MCwxNzAsMTgsODIsMTEyLDE5NCw3MywxNjMsMTA0LDEsNTksMjAxLDk1LDI0OCwyMjAsMTcsMTgyLDQxLDI0MywzOSwxMTc=')
    
    connection_retries = rospy.get_param('~connection_retries', 20)
    
    auto_acquire_control = rospy.get_param('~auto_acquire_control', False)
    force_auto_acquire = rospy.get_param('~force_auto_acquire', True)
    auto_open_brakes = rospy.get_param('~auto_open_brakes', False)
    auto_home_gripper = rospy.get_param('~auto_home_gripper', False)
    auto_enable_fci = rospy.get_param('~auto_enable_fci', False)

    # connect to panda
    self.panda_connector = PandaConnector(robot_ip, username, password)

    rospy.loginfo("Connecting to panda at {}".format(robot_ip))

    # if connection retries is < 0, loop and wait for connection indefinitely
    if connection_retries < 0:
      while not rospy.is_shutdown():
        if self.panda_connector.connect(retries=0):
          break
        rospy.logerr("Connection failed, retrying in 5 seconds")
        rospy.sleep(5)

    rospy.loginfo("Login successful")

    # check if the control token system is required (firmware >4.2.0)
    if self.panda_connector.control_token_req:

      rospy.loginfo("Panda arm requires single point of control, use control_authority/acquire and control_authority/release to control")
      # if auto acquire is true, attempt to start up
      if auto_acquire_control:
        
        # attempt a standard control authority request and assess
        if self.panda_connector.acquire_control_token(False):
          rospy.loginfo("Control authority acquired")
        elif force_auto_acquire:
          # if the above fails, someone else may have control, thus we may need to enfore control, inform system
          rospy.logwarn("Control authority not acquired! Attempting to force, please push physical button on the arm")
          if self.panda_connector.acquire_control_token(True):
            rospy.loginfo("Control authority acquired")
          else:
            rospy.logerr("Unable to auto acquire control")
            return
        else:
          rospy.logerr("Unable to auto acquire control")
          return

    else:
      rospy.loginfo("Panda arm does not require single point of control")

    if auto_open_brakes:
      if self.panda_connector.open_brakes():
        rospy.loginfo("Brakes opened: WARNING: APPROACH ROBOT WITH CATION!")
      else:
        rospy.logerr("Unable to auto open brakes, aborting")
        return
        
    if auto_home_gripper:
      if self.panda_connector.home_gripper():
        rospy.loginfo("Gripper homing complete")
      else:
        rospy.logerr("Unable to auto home gripper, aborting")
        return

    if auto_enable_fci:
      if self.panda_connector.enable_fci():
        rospy.loginfo("FCI Enabled, ready for ros control")
      else:
        rospy.logerr("Unable to auto open brakes, aborting")
        return
    
    rospy.loginfo("Enabling services")

    # setup services
    rospy.Service("control_authority/acquire", SetBool, self.handle_acquire_control)
    rospy.Service("control_authority/release", Trigger, self.handle_release_control)
    rospy.Service("fci/enable", Trigger, self.handle_enable_fci)
    rospy.Service("fci/disable", Trigger, self.handle_disable_fci)
    rospy.Service("brakes/open", Trigger, self.handle_open_brakes)
    rospy.Service("brakes/close", Trigger, self.handle_close_brakes)
    rospy.Service("home_gripper", Trigger, self.handle_home_gripper)

    # spin
    rospy.spin()