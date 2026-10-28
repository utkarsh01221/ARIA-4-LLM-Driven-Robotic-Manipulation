from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math


class RobotExecution:

    def __init__(self):

        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")

        # ----------------------------------------------------
        # IK TARGET
        # ----------------------------------------------------

        self.target = self.sim.getObject("/IK_Target")

        # ----------------------------------------------------
        # IK TIP
        # ----------------------------------------------------

        self.tip = self.sim.getObject(
            "/base_link/Joint1/Link1/Joint2/Link2/Joint3/Link3/Joint4/Link4/IK_Tip"
        )

        print("========================================")
        print("Connected to CoppeliaSim")
        print("========================================")


    # ========================================================
    # OBJECT POSITION
    # ========================================================

    def get_object_position(self, path):

        handle = self.sim.getObject(path)

        return self.sim.getObjectPosition(
            handle,
            -1
        )


    # ========================================================
    # TIP POSITION
    # ========================================================

    def get_tip_position(self):

        return self.sim.getObjectPosition(
            self.tip,
            -1
        )


    # ========================================================
    # TARGET POSITION
    # ========================================================

    def get_target_position(self):

        return self.sim.getObjectPosition(
            self.target,
            -1
        )


    # ========================================================
    # SMOOTH CARTESIAN MOVEMENT
    # ========================================================

    def move_to(
        self,
        x,
        y,
        z,
        speed=0.10,
        tolerance=0.005,
        timeout=20
    ):

        start = self.sim.getObjectPosition(
            self.target,
            -1
        )

        goal = [
            float(x),
            float(y),
            float(z)
        ]

        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        dz = goal[2] - start[2]

        distance = math.sqrt(
            dx * dx +
            dy * dy +
            dz * dz
        )

        if distance < 0.001:

            return True


        duration = distance / speed

        print(
            f"Moving smoothly → "
            f"X={x:.3f}, "
            f"Y={y:.3f}, "
            f"Z={z:.3f}"
        )

        print(
            f"Distance = {distance:.3f} m | "
            f"Speed = {speed:.3f} m/s | "
            f"Time ≈ {duration:.2f} s"
        )


        start_time = time.time()


        # ----------------------------------------------------
        # SMOOTH MOTION
        # ----------------------------------------------------

        while True:

            elapsed = time.time() - start_time

            if elapsed >= duration:
                break


            ratio = elapsed / duration

            # Smoothstep interpolation
            ratio = ratio * ratio * (3 - 2 * ratio)


            current = [

                start[0] + dx * ratio,

                start[1] + dy * ratio,

                start[2] + dz * ratio

            ]


            self.sim.setObjectPosition(
                self.target,
                -1,
                current
            )

            self.client.step()

            time.sleep(0.01)


        # ----------------------------------------------------
        # FINAL POSITION
        # ----------------------------------------------------

        self.sim.setObjectPosition(
            self.target,
            -1,
            goal
        )


        # ----------------------------------------------------
        # WAIT UNTIL TIP REACHES TARGET
        # ----------------------------------------------------

        while time.time() - start_time < timeout:

            self.client.step()

            tip = self.get_tip_position()


            error = math.sqrt(

                (tip[0] - x) ** 2 +

                (tip[1] - y) ** 2 +

                (tip[2] - z) ** 2

            )


            if error <= tolerance:

                print(
                    f"Reached target "
                    f"(error={error:.4f} m)"
                )

                return True


            time.sleep(0.01)


        print(
            f"ERROR: Target not reached "
            f"(error={error:.4f} m)"
        )

        return False


    # ========================================================
    # OPEN GRIPPER
    # ========================================================

    def open_gripper(self):

        print("Opening gripper")

        self.sim.setStringSignal(
            "gripperCommand",
            "open"
        )


        # Allow Robotiq script to execute
        for _ in range(150):

            self.client.step()

            time.sleep(0.02)


    # ========================================================
    # CLOSE GRIPPER
    # ========================================================

    def close_gripper(self):

        print("Closing gripper")

        self.sim.setStringSignal(
            "gripperCommand",
            "close"
        )


        # Allow gripper + attachment logic to execute
        for _ in range(150):

            self.client.step()

            time.sleep(0.02)


    # ========================================================
    # CLOSE CONNECTION
    # ========================================================

    def close(self):

        print("Connection closed")