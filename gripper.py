from copy import deepcopy
import numpy as np
try:
    import rospy
    import actionlib
    from franka_msgs.msg import FrankaState
    from ros_panda_controller.msg import RobotModel
    from franka_gripper.msg import GraspAction, GraspGoal, GraspEpsilon, \
        HomingAction, HomingGoal, MoveAction, MoveGoal
    from sensor_msgs.msg import JointState
    ready_for_real_world = True
    missing_module = None
except ImportError as e:
    missing_module = e.name
    ready_for_real_world = False


class Gripper:

    def __init__(self, homing=True):

        # check if everything is ready for real-world training
        if not ready_for_real_world:
            raise ImportError("You can not run any real-world training. The following package is missed:\n %s" %
                              missing_module)

        # create the gripper action clients
        self.homing_action_client = actionlib.SimpleActionClient("/franka_gripper/homing", HomingAction)
        self.move_action_client = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
        self.grasp_action_client = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)

        # wait for the interfaces
        rospy.loginfo("Waiting for gripper action servers ...")
        self.homing_action_client.wait_for_server()
        self.move_action_client.wait_for_server()
        self.grasp_action_client.wait_for_server()
        rospy.loginfo("Found gripper action servers!")

        # setup a subscriber for information about the gripper fingers
        self._joint_positions = dict()
        self._joint_names = ('panda_finger_joint1', 'panda_finger_joint2')
        self._joint_velocity = dict()
        self._joint_effort = dict()
        self._joint_states_state_sub = rospy.Subscriber('/franka_gripper/joint_states', JointState,
                                                        self._joint_states_callback, queue_size=1, tcp_nodelay=True)

        # gripper_pose
        self._gripper_position = None
        self._gripper_orientation = None
        self._gripper_pose = None

        # gripper grasp indicator
        self._last_action = None
        self._grasped_obj = False
        self.gripper_open = True if homing else None

        # lets make a homing action first
        if homing:
            print("Homing. Please wait ...")
            self.homing()
            print("Finished Homing. The Gripper is ready to use!")

        # setup some default values
        self.def_force = 10     # 10 NM
        self.max_force = 30     # maximum allowed force -> Note that the gripper actually supports more!
        self.def_speed = 0.1
        self.max_speed = 0.2
        self.def_epsilon = 0.005     # 0.005 m

    # -------------- Callbacks --------------
    def _joint_states_callback(self, msg):
        """
        Callback for gripper finger states.

        Parameters
        ----------
        msg: JointState
            Message containing information about the gripper fingers.

        """

        for idx, name in enumerate(msg.name):
            if name in self._joint_names:
                self._joint_positions[name] = msg.position[idx]
                self._joint_velocity[name] = msg.velocity[idx]
                self._joint_effort[name] = msg.effort[idx]

    # -------------- Getters --------------
    def get_joint_positions(self):
        """
        Returns the gripper finger positions.

        Returns
        -------
        np.array
            2-dim array containing the positions of the gripper fingers.
        """

        return deepcopy(np.fromiter(self._joint_positions.values(), dtype=float))

    def get_joint_velocities(self):
        """
        Returns the gripper finger velocities.

        Returns
        -------
        np.array
            2-dim array containing the velocities of the gripper fingers.
        """
        return deepcopy(np.fromiter(self._joint_velocity.values(), dtype=float))

    def get_gripper_position(self):
        """
        Returns the end-effector (gripper) position.

        Returns
        -------
        np.array
            3-dim array containing the position of the end-effector with respect to the Panda robot base.
        """

        return deepcopy(self._gripper_position)

    def get_gripper_orientation(self):
        """
        Returns the end-effector (gripper) orientation. Currently not supported!

        Returns
        -------
        np.array
            4-dim array containing the orientation in quaternions (qx,qy,qz,qw) of the end-effector
            with respect to the Panda robot base.
        """

        return deepcopy(self._gripper_orientation)

    def get_gripper_pose(self):
        """
        Returns the pose of the end-effector (gripper). Currently not supported
        Returns
        -------
        np.array
            7-dim array (x,y,z,qx,qy,qz,qw) containing the pose of the end-effector.

        """
        return deepcopy(np.concatenate([self._gripper_position + self._gripper_orientation]))

    def check_if_object_grasped(self):
        """
        Checks whether the object has been grasped successfully or not.

        Returns
        -------
        bool
            True, if object has been grasped successfully.
        """

        if self._last_action is None:
            raise ValueError("You can not check for grasped object, before making an open or grasp action!")
        elif self._last_action == "open":
            return False
        elif self._last_action == "grasp":
            # check the result
            result = self.grasp_action_client.get_result()
            if result is None:
                self._grasped_obj = False
                return self._grasped_obj
            elif result.success:
                self._grasped_obj = True
                return self._grasped_obj
            else:
                self._grasped_obj = False
                return self._grasped_obj

    # -------------- Setters --------------
    def set_gripper_position(self, position):
        """
        Set the internal end-effector (gripper) position. This should only be called from the robot class to set
        the position received from the callback.

        Parameters
        ----------
        position : np.array
            3-dim (x,y,z) array containing the position of the end-effector.

        """

        self._gripper_position = position

    def set_gripper_orientation(self, orientation):
        """
        Set the internal end-effector (gripper) orientation. This should only be called from the robot class to set
        the orientation received from the callback.

        Parameters
        ----------
        orientation : np.array
            4-dim (qx,qy,qz,qw) array containing the orientation of the end-effector ion quaternions.

        """

        self._gripper_orientation = orientation

    # -------------- Actions --------------
    def grasp(self, width, speed=None, force=None, epsilon=None, wait=False):
        """
        Grasps an object with a given width, speed and force. The operation is successful if the distance d between
        the gripper fingers is: width−epsilon_inner < d < width+epsilon_outer.

        Parameters
        ----------
        width : float
            Width of the object to grasp.
        speed : float
            Speed of the gripper fingers while grasping.
        force : float
            Force of the gripper fingers while grasping.
        epsilon : float
            Threshold used to indicate whether the grasp was successful or not.
        wait : bool
            If true, it is waited until the grasp has been completed

        """

        # check what values are provided and fallback to defaults if needed
        if not speed:
            speed = self.def_speed
        if speed > self.max_speed:
            print("Speed is too high. Reducing to %f." % self.max_speed)
            speed = self.max_speed
        if not force:
            force = self.def_force
        if force > self.max_force:
            print("Force is too high. Reducing to %f." % self.max_force)
            force = self.max_force
        if not epsilon:
            epsilon = self.def_epsilon

        # prepare goal
        goal = GraspGoal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon.inner = epsilon
        goal.epsilon.outer = epsilon

        # send goal
        self.grasp_action_client.send_goal(goal)
        self._last_action = "grasp"
        self.gripper_open = False
        if wait:
            self.grasp_action_client.wait_for_result()

    def homing(self):
        """
        Performs homing to calibrate gripper fingers.

        Returns
        -------
        bool
            True, if homing was successful.
        """

        self.homing_action_client.send_goal_and_wait(HomingGoal())
        self._last_action = None
        self.gripper_open = True
        return self.homing_action_client.get_result().success

    def open(self):
        """
        Moves the gripper to the maximum width.

        Returns
        -------
        bool
            True, if opening was successful.
        """

        self._last_action = "open"
        self.gripper_open = True
        return self._move(0.15)

    def _move(self, width, speed=None):
        """
        Moves the gripper finger to a certain width with a certain speed.

        Parameters
        ----------
        width : float
            Width between the gripper fingers to be reached.
        speed : float
            Speed of the fingers used while moving.

        Returns
        -------
        bool
            True, if moving was successful.
        """

        # check what values are provided and fallback to defaults if needed
        if not speed:
            speed = self.def_speed
        if speed > self.max_speed:
            print("Speed is too high. Reducing to %f." % self.max_speed)
            speed = self.max_speed

        # prepare goal
        goal = MoveGoal()
        goal.width = width
        goal.speed = speed

        # send goal and wait
        self.move_action_client.send_goal_and_wait(goal)

        return self.move_action_client.get_result().success
