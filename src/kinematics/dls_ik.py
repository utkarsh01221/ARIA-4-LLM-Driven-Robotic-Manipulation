import numpy as np
import math
import time


class DLSInverseKinematics:

    def __init__(
        self,
        sim,
        client,
        joint_paths,
        tip_path,
        damping=0.5,
        max_iterations=100,
        tolerance=0.005,
        numerical_step=1e-5
    ):

        self.sim = sim
        self.client = client

        self.joint_paths = joint_paths
        self.tip_path = tip_path

        self.damping = float(damping)
        self.max_iterations = int(max_iterations)
        self.tolerance = float(tolerance)
        self.numerical_step = float(numerical_step)

        self.joints = [
            self.sim.getObject(path)
            for path in self.joint_paths
        ]

        self.tip = self.sim.getObject(self.tip_path)

        print("========================================")
        print("       ARIA-4 DLS IK INITIALIZED")
        print("========================================")
        print(f"Joints          : {len(self.joints)}")
        print(f"Damping λ       : {self.damping}")
        print(f"Max iterations  : {self.max_iterations}")
        print(f"Tolerance       : {self.tolerance} m")
        print(f"Jacobian step   : {self.numerical_step}")
        print("========================================")

    # --------------------------------------------------
    # JOINT POSITIONS
    # --------------------------------------------------

    def get_joint_positions(self):

        return np.array(
            [
                self.sim.getJointPosition(joint)
                for joint in self.joints
            ],
            dtype=float
        )

    # --------------------------------------------------
    # END-EFFECTOR POSITION
    # --------------------------------------------------

    def get_tip_position(self):

        position = self.sim.getObjectPosition(
            self.tip,
            -1
        )

        return np.array(
            position,
            dtype=float
        )

    # --------------------------------------------------
    # NUMERICAL JACOBIAN
    # --------------------------------------------------

    def calculate_jacobian(self):

        q_original = self.get_joint_positions()

        n = len(self.joints)

        J = np.zeros((3, n))

        h = self.numerical_step

        for i in range(n):

            # q + h
            q_plus = q_original.copy()
            q_plus[i] += h

            for j in range(n):
                self.sim.setJointPosition(
                    self.joints[j],
                    float(q_plus[j])
                )

            self.client.step()

            p_plus = self.get_tip_position()

            # q - h
            q_minus = q_original.copy()
            q_minus[i] -= h

            for j in range(n):
                self.sim.setJointPosition(
                    self.joints[j],
                    float(q_minus[j])
                )

            self.client.step()

            p_minus = self.get_tip_position()

            # Central finite difference
            J[:, i] = (
                p_plus - p_minus
            ) / (2.0 * h)

        # Restore original configuration
        for i in range(n):

            self.sim.setJointPosition(
                self.joints[i],
                float(q_original[i])
            )

        self.client.step()

        return J

    # --------------------------------------------------
    # DAMPED LEAST SQUARES
    # --------------------------------------------------

    def calculate_joint_update(
        self,
        J,
        error
    ):

        lambda_squared = self.damping ** 2

        identity = np.eye(
            J.shape[0]
        )

        A = (
            J @ J.T
            +
            lambda_squared * identity
        )

        delta_q = (
            J.T
            @
            np.linalg.solve(
                A,
                error
            )
        )

        return delta_q

    # --------------------------------------------------
    # SOLVE IK
    # --------------------------------------------------

    def solve(
        self,
        target_position
    ):

        target = np.array(
            target_position,
            dtype=float
        )

        print("\n========================================")
        print("        ARIA-4 DLS IK SOLVER")
        print("========================================")

        print(
            f"Target XYZ = "
            f"({target[0]:.4f}, "
            f"{target[1]:.4f}, "
            f"{target[2]:.4f})"
        )

        start_time = time.time()

        for iteration in range(
            self.max_iterations
        ):

            current = self.get_tip_position()

            error = target - current

            error_norm = np.linalg.norm(error)

            print(
                f"Iteration {iteration + 1:03d} | "
                f"Error = {error_norm:.6f} m"
            )

            # Converged
            if error_norm <= self.tolerance:

                elapsed = (
                    time.time()
                    -
                    start_time
                )

                print("\nDLS IK CONVERGED")

                print(
                    f"Iterations : {iteration + 1}"
                )

                print(
                    f"Final error: "
                    f"{error_norm:.6f} m"
                )

                print(
                    f"Time       : "
                    f"{elapsed:.3f} s"
                )

                return {
                    "success": True,
                    "joint_positions":
                        self.get_joint_positions(),
                    "tip_position":
                        current,
                    "error":
                        error_norm,
                    "iterations":
                        iteration + 1,
                    "time":
                        elapsed
                }

            # Calculate numerical Jacobian
            J = self.calculate_jacobian()

            # DLS joint update
            delta_q = (
                self.calculate_joint_update(
                    J,
                    error
                )
            )

            # Limit excessively large updates
            max_step = 0.10

            step_norm = np.linalg.norm(
                delta_q
            )

            if step_norm > max_step:

                delta_q *= (
                    max_step
                    /
                    step_norm
                )

            # Current joint configuration
            q = self.get_joint_positions()

            # Update joints
            q_new = q + delta_q

            for i in range(
                len(self.joints)
            ):

                self.sim.setJointPosition(
                    self.joints[i],
                    float(q_new[i])
                )

            self.client.step()

            time.sleep(0.01)

        # Failed
        final_position = (
            self.get_tip_position()
        )

        final_error = np.linalg.norm(
            target - final_position
        )

        elapsed = (
            time.time()
            -
            start_time
        )

        print("\nDLS IK FAILED")

        print(
            f"Final error = "
            f"{final_error:.6f} m"
        )

        return {
            "success": False,
            "joint_positions":
                self.get_joint_positions(),
            "tip_position":
                final_position,
            "error":
                final_error,
            "iterations":
                self.max_iterations,
            "time":
                elapsed
        }