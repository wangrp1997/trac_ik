#!/usr/bin/env python

# Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
# Convenience code to wrap TRAC IK
# Modified for ROS2 compatibility

from trac_ik_python.trac_ik_wrap import TRAC_IK


class IK(object):
    def __init__(self, base_link, tip_link,
                 timeout=0.005, epsilon=1e-5, solve_type="Speed",
                 urdf_string=None,
                 lock_joint_name=None,
                 lock_joint_value=None):
        """
        Create a TRAC_IK instance and keep track of it.

        :param str base_link: Starting link of the chain.
        :param str tip_link: Last link of the chain.
        :param float timeout: Timeout in seconds for the IK calls.
        :param float epsilon: Error epsilon.
        :param solve_type str: Type of solver, can be:
            Speed (default), Distance, Manipulation1, Manipulation2
        :param urdf_string str: Required in ROS2. URDF string must be provided.
            In ROS2, get it from parameter server using rclpy or pass directly.
        """
        if urdf_string is None:
            raise ValueError(
                "urdf_string must be provided in ROS2. "
                "Get it from ROS2 parameter server using rclpy.get_parameter('robot_description').value "
                "or read from URDF file."
            )
        self._urdf_string = urdf_string
        self._timeout = timeout
        self._epsilon = epsilon
        self._solve_type = solve_type
        self.base_link = base_link
        self.tip_link = tip_link
        try:
            # This may throw C++ exceptions which SWIG converts to Python exceptions
            self._ik_solver = TRAC_IK(self.base_link,
                                      self.tip_link,
                                      self._urdf_string,
                                      self._timeout,
                                      self._epsilon,
                                      self._solve_type)
            # NOTE:
            # - We intentionally do NOT call getJointNamesInChain()/getLinkNamesInChain() here.
            # - gdb backtrace showed a hard SIGSEGV inside TRAC_IK_TRAC_IK_getJointNamesInChain()
            #   (shared_ptr release in urdf::Joint), which is unrelated to IK solving itself.
            # - For NBV planning we only need number_of_joints and get_ik().
            self.number_of_joints = self._ik_solver.getNrOfJointsInChain()
            try:
                # getJointNamesInChain() is implemented in SWIG using KDL only (urdf_string ignored)
                self.joint_names = list(self._ik_solver.getJointNamesInChain(""))
            except Exception:
                self.joint_names = []
            try:
                self.link_names = list(self._ik_solver.getLinkNamesInChain())
            except Exception:
                self.link_names = []

            # Optionally lock a joint by name (generic; does NOT assume index ordering)
            if lock_joint_name is not None or lock_joint_value is not None:
                if not lock_joint_name or lock_joint_value is None:
                    raise ValueError(
                        f"Both lock_joint_name and lock_joint_value must be provided together. "
                        f"Got lock_joint_name={lock_joint_name!r}, lock_joint_value={lock_joint_value!r}."
                    )
                self.lock_joint(lock_joint_name, float(lock_joint_value))
        except RuntimeError as e:
            # Re-raise RuntimeError with more context
            raise RuntimeError(
                f"Failed to initialize TRAC_IK solver: {str(e)}. "
                f"Base link: {base_link}, Tip link: {tip_link}. "
                f"Make sure both links exist in the URDF and form a valid kinematic chain."
            ) from e
        except Exception as e:
            # Catch any other Python exceptions (including SWIG-converted C++ exceptions)
            error_msg = str(e) if e else "Unknown error"
            raise RuntimeError(
                f"Failed to initialize TRAC_IK solver: {error_msg}. "
                f"Base link: {base_link}, Tip link: {tip_link}. "
                f"Make sure both links exist in the URDF and form a valid kinematic chain."
            ) from e

    def lock_joint(self, joint_name: str, value: float):
        """
        Lock a joint by name by setting its lower/upper limits to the same value.
        This affects subsequent IK solves.
        """
        if not self.joint_names:
            raise RuntimeError(
                "Cannot lock joint because joint_names is empty. "
                "Failed to query chain joint names from KDL."
            )
        if joint_name not in self.joint_names:
            raise ValueError(
                f"Joint {joint_name!r} not found in IK chain. "
                f"Chain joints: {self.joint_names}"
            )
        idx = self.joint_names.index(joint_name)
        lb, ub = self.get_joint_limits()
        if len(lb) != self.number_of_joints or len(ub) != self.number_of_joints:
            raise RuntimeError(
                f"Internal error: joint limits length mismatch. "
                f"lb={len(lb)} ub={len(ub)} expected={self.number_of_joints}"
            )
        lb = list(lb)
        ub = list(ub)
        lb[idx] = float(value)
        ub[idx] = float(value)
        self.set_joint_limits(lb, ub)

    def get_ik(self, qinit,
               x, y, z,
               rx, ry, rz, rw,
               bx=1e-5, by=1e-5, bz=1e-5,
               brx=1e-3, bry=1e-3, brz=1e-3):
        """
        Do the IK call.

        :param list of float qinit: Initial status of the joints as seed.
        :param float x: X coordinates in base_frame.
        :param float y: Y coordinates in base_frame.
        :param float z: Z coordinates in base_frame.
        :param float rx: X quaternion coordinate.
        :param float ry: Y quaternion coordinate.
        :param float rz: Z quaternion coordinate.
        :param float rw: W quaternion coordinate.
        :param float bx: X allowed bound.
        :param float by: Y allowed bound.
        :param float bz: Z allowed bound.
        :param float brx: rotation over X allowed bound.
        :param float bry: rotation over Y allowed bound.
        :param float brz: rotation over Z allowed bound.

        :return: joint values or None if no solution found.
        :rtype: tuple of float.
        """
        if len(qinit) != self.number_of_joints:
            raise Exception("qinit has length %i and it should have length %i" % (
                len(qinit), self.number_of_joints))
        solution = self._ik_solver.CartToJnt(qinit,
                                             x, y, z,
                                             rx, ry, rz, rw,
                                             bx, by, bz,
                                             brx, bry, brz)
        if solution:
            return solution
        else:
            return None

    def get_joint_limits(self):
        """
        Return lower bound limits and upper bound limits for all the joints
        in the order of the joint names.
        """
        lb = self._ik_solver.getLowerBoundLimits()
        ub = self._ik_solver.getUpperBoundLimits()
        return lb, ub

    def set_joint_limits(self, lower_bounds, upper_bounds):
        """
        Set joint limits for all the joints.

        :arg list lower_bounds: List of float of the lower bound limits for
            all joints.
        :arg list upper_bounds: List of float of the upper bound limits for
            all joints.
        """
        if len(lower_bounds) != self.number_of_joints:
            raise Exception("lower_bounds array size mismatch, it's size %i, should be %i" % (
                len(lower_bounds),
                self.number_of_joints))

        if len(upper_bounds) != self.number_of_joints:
            raise Exception("upper_bounds array size mismatch, it's size %i, should be %i" % (
                len(upper_bounds),
                self.number_of_joints))
        self._ik_solver.setKDLLimits(lower_bounds, upper_bounds)
