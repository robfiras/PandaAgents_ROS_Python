from copy import deepcopy
try:
    import rospy
    from ros_panda_controller.msg import AgentActions
    ready_for_real_world = True
    missing_module = None
except ImportError as e:
    missing_module = e.name
    ready_for_real_world = False


class Arm:

    def __init__(self):

        # check if everything is ready for real-world training
        if not ready_for_real_world:
            raise ImportError("You can not run any real-world training. The following package is missed:\n %s" %
                              missing_module)

        # setup arm entities
        self._joint_positions = None
        self._joint_velocities = None
        self._joint_torques = None

    # -------------- Getters --------------
    def get_joint_positions(self):
        """
        Returns the current joint positions.

        Returns
        -------
        np.array
            7-dim array containing the current joint positions.

        """

        if self._joint_positions is None:
            raise ValueError("Did not received any joint positions yet.")
        return deepcopy(self._joint_positions)

    def get_joint_velocities(self):
        """
        Returns the current joint velocities.

        Returns
        -------
        np.array
            7-dim array containing the current joint velocities.

        """

        if self._joint_velocities is None:
            raise ValueError("Did not received any joint velocities yet.")
        return deepcopy(self._joint_velocities)

    def get_joint_torques(self):
        """
        Returns the current joint torques.

        Returns
        -------
        np.array
            7-dim array containing the current joint torques.

        """

        if self._joint_torques is None:
            raise ValueError("Did not received any joint torques yet.")
        return deepcopy(self._joint_torques)

    # -------------- Setters --------------
    def set_joint_positions(self, joint_positions):
        """
        Sets the joint positions. This should only be called from the robot class to set the positions received from
        the callback.

        Parameters
        ----------
        joint_positions : np.array
            7-dim array containing joint positions.

        """

        self._joint_positions = joint_positions

    def set_joint_velocities(self, joint_velocities):
        """
        Sets the joint positions. This should only be called from the robot class to set the velocities received from
        the callback.

        Parameters
        ----------
        joint_velocities : np.array
            7-dim array containing joint positions.

        """

        self._joint_velocities = joint_velocities

    def set_joint_torques(self, joint_torques):
        """
        Sets the joint positions. This should only be called from the robot class to set the torques received from
        the callback.

        Parameters
        ----------
        joint_torques : np.array
            7-dim array containing joint torques.

        """

        self._joint_torques = joint_torques
