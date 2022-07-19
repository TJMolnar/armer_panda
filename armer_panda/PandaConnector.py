#!/usr/bin/env python3
import requests
import json
import time
import urllib3
from packaging import version

class PandaConnector:
  """ Panda Connector: Franka Desk API Handling for the Franka Panda arm. Connects to the web server on the panda arm 
  and allows for scripting of the functions that normally require an end-user on a gui:
    - Single-Point-Of-Control 
    - Opening/Closing Brakes
    - Homing Gripper
    - Enabling/Disabling FCI
  """
  def __init__(self, robot_ip, username, password):
    self.robot_ip = robot_ip
    self.username = username
    # 'password' here is actually the hash generated for the login request to the desk webpage. To find this:
    # 1. Go to the franka Desk login page on chrome/firefox (logout if you're logged in) and hit f12 to open console. 
    # 2. Go to the network tab and turn on 'persist-log'
    # 3. Log in to Desk with the username you want to use here
    # 4. Under logs, there should be a POST request to https://<ip>/admin/api/login, in the payload of that request will
    #    be the username and password hash to use here.
    self.password = password

    self.session = None
    self.connected = False

    # system version string
    self.version = None

    # control token information for single-point-of-control (spoc, v4.2.0 and up)
    self.control_token_req = False
    self.control_token = None
    self.control_token_id = None

  def connect(self, retries=20):
    """ Establish a tcp session and attempt to connect to the platform (and get system version for spoc compatibility)
    """
    urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
    self.session = requests.Session()
    # login using proviced credentials
    if not self.login(retries=retries):
      return False
    # get system version, determines if we need spoc
    if not self.read_system_version():
      return False
    return True

  def login(self, retries=20):
    """ Log in to web interface and establish connection """
    res, resp = self.__call('https://{}/admin/api/login'.format(self.robot_ip), json={'login': self.username, 'password': self.password}, retries=retries)
    if res:
      # add authorisation cookie to session
      self.session.cookies['authorization'] = resp.text
      self.connected = True
    return res

  def logout(self):
    """ Log out of web interface """
    if self.__call('https://{}/admin/api/logout'.format(self.robot_ip))[0]:
      self.connected = False
      return True
    return False

  def read_system_version(self):
    """ Get the system version of the franka, this lets us determine if we need to use the spoc functionality"""
    res, resp = self.__call('https://{}/admin/api/system-version'.format(self.robot_ip), reqtype='GET')
    if res:
      # parse system version string, if version >= 4.2.0, then we need to use spoc
      self.version = resp.text.strip('\"').split('\\n')[0]
      if version.parse(self.version) >= version.parse("4.2.0"):
        self.control_token_req = True
    return res

  def acquire_control_token(self, force=True):
    """ Acquire control authority for single-point-of-control. In order to open the brakes and activate fci, we need to
    hold the active control token for the franka (and add it to the header of our api calls), this can optionally be 
    forcibly acquired by making the appropriate request and then pressing the frontmost button on the end panda link 
    (see docs)"""
    if force:
      # get length of timeout for forcing control override
      res, resp = self.__call('https://{}/admin/api/safety'.format(self.robot_ip), reqtype='GET')
      if not res:
          return False
      timeout = json.loads(resp.text)['tokenForceTimeout']

      # make forced request
      res, resp = self.__call('https://{}/admin/api/control-token/request?force'.format(self.robot_ip), json={'requestedBy': self.username})
      if not res:
        return False
      
      # forced request returns a control token, however, it is not necessarily active yet
      self.control_token = json.loads(resp.text)['token']
      self.control_token_id = json.loads(resp.text)['id']
      self.session.headers.update({'X-Control-Token': self.control_token})
      
      # loop and wait to see if we get control, if no other user has control, this will succeed immediately. If another
      # user does have authority, this will wait the full timeout period to see if we get control, this will either be
      # from the other user releasing control, or the operator pushing the physical button on the arm
      control_acquired = False
      start = time.time()
      while time.time() < start + timeout:
        if self.is_control_token_active():
          control_acquired = True
          break
      return control_acquired

    else:
      # not forcing, make a standard control request and then see if we acquired it successfully
      res, resp = self.__call('https://{}/admin/api/control-token/request'.format(self.robot_ip), json={'requestedBy': self.username})
      if not res:
        return False
      
      # system always returns a control token, but ours might not be active yet
      self.control_token = json.loads(resp.text)['token']
      self.control_token_id = json.loads(resp.text)['id']
      self.session.headers.update({'X-Control-Token': self.control_token})
      
      return self.is_control_token_active()
  
  def can_control(self):
    if self.control_token_req and self.is_control_token_active():
      return True
    elif not self.control_token_req:
      return True
    return False

  def is_control_token_active(self):
    """Determines if control token we have is active"""
    # get active control token
    res, resp = self.__call('https://{}/admin/api/control-token'.format(self.robot_ip), reqtype='GET')
    if not res:
      return False
    # if noone has control, activeToken is null
    active_token = json.loads(resp.text)['activeToken']
    if active_token is None:
      return False
    
    # otherwise, if activeToken id is ours, we have control
    return active_token['id'] == self.control_token_id

  def release_control_token(self):
    """Releases control token, regardless of if ours is active"""
    if self.__call('https://{}/admin/api/control-token'.format(self.robot_ip), reqtype='DELETE', json={'token': self.control_token})[0]:
      self.control_token == None
      return True
    return False
  
  def enable_fci(self):
    """Enables the Franka Control Interface feature"""
    return self.__call('https://{}/admin/api/control-token/fci'.format(self.robot_ip), json={'token': self.control_token})[0]

  def disable_fci(self):
    """Disables the Franka Control Interface"""
    return self.__call('https://{}/admin/api/control-token/fci'.format(self.robot_ip), reqtype='DELETE', json={'token': self.control_token})[0]

  def open_brakes(self, retries=5):
    """Open the physcial brakes on the arm (takes ~10s)"""
    return self.__call('https://{}/desk/api/robot/open-brakes'.format(self.robot_ip))[0]

  def close_brakes(self):
    """Close the physcial brakes on the arm (WARNING: Avoid closing brakes while arm is moving except in emergency)"""
    return self.__call('https://{}/desk/api/robot/close-brakes'.format(self.robot_ip))[0]

  def home_gripper(self):
    """Perform homing for the franka gripper (required on each startup prior to use)"""
    return self.__call('https://{}/desk/api/gripper/homing'.format(self.robot_ip))[0]

  def __call(self, endpoint, reqtype='POST', json=None, retries=5):
    """Make an api call to the Franka Panda web server, optionally providing data, returns tuple of call success and 
    api response"""
    attempts = 0
    while attempts < retries + 1:
      try:
        resp = self.session.request(reqtype, endpoint, json=json, verify=False, timeout=10)
        if resp.status_code == 200:
          return True, resp
        else:
          attempts = attempts + 1
      except:
        attempts = attempts + 1
    return False, None