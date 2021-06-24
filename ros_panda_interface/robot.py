import numpy as np
from ros_panda_interface.arm import Arm
from ros_panda_interface.gripper import Gripper

try:
    import rospy
    from franka_msgs.msg import FrankaState
    from franka_control.srv import SetForceTorqueCollisionBehavior, SetForceTorqueCollisionBehaviorRequest
    from ros_panda_controller.msg import RobotModel
    from ros_panda_controller.msg import AgentActions
    from pyquaternion import Quaternion
    ready_for_real_world = True
    missing_module = None
except ImportError as e:
    missing_module = e.name
    ready_for_real_world = False


class Robot:

    def __init__(self, home_gripper, width=0.05):

        # check if everything is ready for real-world training
        if not ready_for_real_world:
            raise ImportError("You can not run any real-world training. The following package is missed:\n %s" %
                              missing_module)

        # initialise robot_agent node
        rospy.init_node('robot_agent', anonymous=True)

        # setup the arm
        self._arm = Arm()

        # setup the gripper
        self._gripper = Gripper(home_gripper)

        # setup a subscriber for information about the arm and gripper
        self._franka_state_sub = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState,
                                                  self._franka_state_callback, queue_size=1, tcp_nodelay=True)

        # setup a subscriber for the robot model
        self._model_sub = rospy.Subscriber('/robot_model_publisher', RobotModel,
                                           self._robot_model_callback, queue_size=1, tcp_nodelay=True)
        self.J = []

        # set the width of the object to grasp
        self.width = width

        # frequency for control
        self.freq = 20  # Hz

        # setup the publisher for the joint actions
        self.action_publisher = rospy.Publisher('/rl_agent_actions', AgentActions, queue_size=1, tcp_nodelay=True)

        # reference position for redundancy resolution, only for homing!
        self.ref_pos_reorient = np.array([0.0, -0.82, 0.0, -2.15, 0.0, 1.75, 0.75])

        # home position of the robot (default position taken from RLBench tasks), only for homing!
        self.home_pos = np.array([0.0, 0.1745, 0.0, -0.8726, 0.0, 1.2217, 0.7854])

        # set the collision behavior
        # WARNING: This allows heavy interaction with the environment!
        collision_service_name = '/franka_control/set_force_torque_collision_behavior'
        rospy.wait_for_service(collision_service_name)
        set_collision_behavior = rospy.ServiceProxy(collision_service_name, SetForceTorqueCollisionBehavior)
        lower_torque_thresholds_nominal = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]  # [Nm]
        upper_torque_thresholds_nominal = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]  # [Nm]
        lower_force_thresholds_nominal = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]     # [N, N, N, Nm, Nm, Nm]
        upper_force_thresholds_nominal = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]     # [N, N, N, Nm, Nm, Nm]
        collision_behavior = SetForceTorqueCollisionBehaviorRequest(lower_torque_thresholds_nominal,
                                                                    upper_torque_thresholds_nominal,
                                                                    lower_force_thresholds_nominal,
                                                                    upper_force_thresholds_nominal)
        set_collision_behavior(collision_behavior)

    # -------------- Callbacks --------------
    def _franka_state_callback(self, msg):
        """
        This callback receives information regarding the robot arm and the gripper.

        Parameters
        ----------
        msg : FrankaState
            Message containing information about the robot.

        """

        # get the joint positions
        self._arm.set_joint_positions(np.asarray(msg.q))
        self._arm.set_joint_velocities(np.asarray(msg.dq))
        self._arm.set_joint_torques(np.asarray(msg.tau_J))

        # get the end-effector position in base frame
        ee_pose_base = np.asarray(msg.O_T_EE).reshape(4, 4, order='F')
        self._gripper.set_gripper_position(ee_pose_base[:3, 3])

    def _robot_model_callback(self, msg):
        """
        This callback receives information regarding the robot.

        Parameters
        ----------
        msg : RobotModel
            Message containing information about the robot model.

        """

        self.J = np.asarray(msg.zeroJacobian)

    # -------------- Getters --------------
    def get_arm_joint_positions(self):
        """
        Returns the current joint positions of the robot arm.

        Returns
        -------
        np.array
            7-dim array containing the joint positions of the arm.

        """

        return self._arm.get_joint_positions()

    def get_arm_joint_velocities(self):
        """
        Returns the current joint velocities of the robot arm.

        Returns
        -------
        np.array
            7-dim array containing the joint velocities of the arm.

        """

        return self._arm.get_joint_velocities()

    def get_arm_joint_torques(self):
        """
        Returns the current joint torques of the robot arm.

        Returns
        -------
        np.array
            7-dim array containing the joint torques of the arm.

        """

        return self._arm.get_joint_torques()

    def get_ee_position(self):
        """
        Returns the positions of the end-effector.

        Returns
        -------
        np.array
            3-dim array containing the end-effector position with respect to the Panda robot base.

        """

        return self._gripper.get_gripper_position()

    def get_gripper_finger_positions(self):
        """
        Returns the gripper finger positions.

        Returns
        -------
        np.array
            2-dim array containing the positions of the gripper fingers.

        """

        return self._gripper.get_joint_positions()

    def get_gripper_state(self):
        """
        Returns gripper state.

        Returns
        -------
        bool
            True, if gripper is open.

        """
        return self._gripper.gripper_open

    # -------------- Actions --------------
    def send_action(self, action, disable_gripper=False):
        """
        Publishes arm actions and calls the gripper action server.

        Parameters
        ----------
        action : np.array
            8-dim array, where action[0:7] are arm actions (desired angular joint velocities [rad/s] for the 7 joints),
            and action[7] is the gripper action (0 -> close gripper; 1 -> open gripper)
        disable_gripper : bool
            If true, no gripper actions are executed.

        """

        # create an action message
        action_msg = AgentActions()

        # setup action mode -> current only JointVelocity is supported
        action_msg.action_mode = "JointVelocity"

        # --- arm actions ---
        # if grasp is executed arm needs to stop (this is done to align with the version
        # of RLBench used within this work)
        if not disable_gripper and ((action[7] < 0.5 and self._gripper.gripper_open) or
                                    (action[7] >= 0.5 and not self._gripper.gripper_open)):
            self.stop(sec=0.5)
        else:
            action_msg.actions = action[0:7].tolist()
            self.action_publisher.publish(action_msg)

        # --- gripper actions ---
        if not disable_gripper:
            if action[7] < 0.5 and self._gripper.gripper_open:
                self._gripper.grasp(self.width, wait=True)
            elif action[7] >= 0.5 and not self._gripper.gripper_open:
                self._gripper.open()

    def stop(self, sec=0.5):
        """
        Stops the robot by reducing the velocity linearly to zero.

        Parameters
        ----------
        sec : float
            Seconds in which the robot needs to stop.

        """

        # get the current velocities of the arm
        cur_vel = self._arm.get_joint_velocities()

        # calculate the steps in which we need to stop
        steps_to_stop = self.freq*sec
        steps_to_stop = np.maximum(steps_to_stop, 5)    # don't allow too jerky stopping

        # calculate the next velocities
        next_vels = [cur_vel*((steps_to_stop-1-i)/steps_to_stop) for i in range(int(steps_to_stop))]

        # setup ros timer
        rate = rospy.Rate(self.freq)
        for vels in next_vels:
            self.send_action(vels, True)
            rate.sleep()

    def move_to_home(self):
        """
        Moves the robot back to its home position.

        """

        # calculate the error to the home position
        e = self._arm.get_joint_positions() - self.home_pos

        # setup ros timer
        rate = rospy.Rate(self.freq)

        # if error is huge, make the ee stand upright first using redundancy resolution
        if np.max(np.abs(e)) > 0.3:
            for i in range(80):
                vels = np.array([0.0]*7)
                vels -= self.resolve_redundancy_joint_velocities(self.ref_pos_reorient, alpha=1.0)
                self.send_action(vels, True)
                rate.sleep()

        # move to reference position -> PD controller
        Kp = 0.4
        Kd = 0.4
        e_prev = e
        t_prev = -100
        while np.max(np.abs(e)) > 0.05:
            e = self.home_pos - self._arm.get_joint_positions()
            P = Kp * e
            t = rospy.rostime.get_rostime()
            dt = t.nsecs - t_prev
            D = Kd * (e - e_prev) /dt
            t_prev = t.nsecs
            rate.sleep()
            vels = P+D
            self.send_action(vels, True)

        # stop if not fully stopped yet
        self.stop()

    def open_gripper(self):
        """
        Opens the gripper.

        """
        self._gripper.open()

    # -------------- Miscellaneous --------------
    def resolve_redundancy_joint_velocities(self, reference_position, alpha=1.0, W=np.array([1.0]*7)):
        """
        Calculates joint velocities, which lie in the nullspace of the robot Jacobian matrix (i.e., do not generate
        movement of the end-effector) and reduce the distance to a reference position.

        Parameters
        ----------
        reference_position : np.array
            Reference Position (7-dim array) to which the distance should be reduced.
        alpha : float
            Step-size parameter controlling the strength (or speed) of redundancy resolution.
        W : np.array
            Weighting for the error.

        Returns
        -------
        np.array
            Joint velocities for redundancy resolution.

        """

        # get the Jacobian -> here col-major order, this is different to Pyrep!
        J = np.reshape(self.J, (6, 7), order='F')
        J = J[0:3]  # take only the part of the linear velocities of the end-effector

        # compute the pseudo inverse
        J_plus = np.linalg.pinv(J)

        # compute the error
        e = (self._arm.get_joint_positions() - reference_position)*W

        # compute the joint velocities
        q_dot_redundancy = alpha * np.matmul((np.identity(7) - np.matmul(J_plus, J)), e)

        return q_dot_redundancy

    def check_if_object_grasped(self):
        """
        Returns true if the object has been grasped successfully.

        Returns
        -------
        bool
            True if the object has been grasped successfully.
        """

        return self._gripper.check_if_object_grasped()
