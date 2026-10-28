import time


class MotionPlanner:
    APPROACH_Z = 0.385
    PICK_Z = 0.310
    LIFT_Z = 0.470
    PLACE_Z = 0.400
    TRANSIT_Z = 0.550

    def __init__(self, robot, object_manager):
        self.robot = robot
        self.objects = object_manager

    def pick_and_place(self, source, target):
        source = source.lower()
        target = target.lower()

        if source == target:
            raise ValueError(
                "Source and target cannot be the same."
            )

        print("\n########################################")
        print("     VISION COORDINATE CAPTURE")
        print("########################################")

        # ========================================
        # CAPTURE BOTH OBJECTS BEFORE MOVEMENT
        # ========================================

        print("\nCapturing SOURCE position...")
        source_position = self.objects.get_position(source)

        print("\nCapturing TARGET position...")
        target_position = self.objects.get_position(target)

        source_x = source_position[0]
        source_y = source_position[1]

        target_x = target_position[0]
        target_y = target_position[1]

        print("\n========================================")
        print("       SAVED OBJECT COORDINATES")
        print("========================================")
        print(
            f"SOURCE {source.upper()} → "
            f"X={source_x:.4f}, Y={source_y:.4f}"
        )
        print(
            f"TARGET {target.upper()} → "
            f"X={target_x:.4f}, Y={target_y:.4f}"
        )
        print("========================================")

        print("\nCoordinates locked.")
        print("Robot movement can now begin.")

        # ========================================
        # PICK SOURCE
        # ========================================

        print("\n========================================")
        print(f"       PICKING {source.upper()}")
        print("========================================")

        print("\nOpening gripper")
        self.robot.open_gripper()
        time.sleep(0.5)

        print("\nApproaching source")
        if not self.robot.move_to(
            source_x,
            source_y,
            self.APPROACH_Z,
            speed=0.10
        ):
            raise RuntimeError(
                f"Failed to approach {source}"
            )

        print("\nDescending")
        if not self.robot.move_to(
            source_x,
            source_y,
            self.PICK_Z,
            speed=0.04
        ):
            raise RuntimeError(
                f"Failed to reach {source}"
            )

        print("\nGrasping object")
        self.robot.close_gripper()
        time.sleep(2)

        print("\nLifting object")
        if not self.robot.move_to(
            source_x,
            source_y,
            self.LIFT_Z,
            speed=0.06
        ):
            raise RuntimeError(
                f"Failed to lift {source}"
            )

        print(
            f"\n{source.upper()} PICKED SUCCESSFULLY"
        )

        # ========================================
        # PLACE ON SAVED TARGET POSITION
        # ========================================


        print("\n========================================")
        print(f"       TRANSIT TO {target.upper()}")
        print("========================================")

        print("\nMoving to safe transit height")

        if not self.robot.move_to(
            source_x,
            source_y,
            self.TRANSIT_Z,
            speed=0.06
        ):
            raise RuntimeError(
                "Failed to reach safe transit height"
            )

        print("\nMoving horizontally above target")

        if not self.robot.move_to(
            target_x,
            target_y,
            self.TRANSIT_Z,
            speed=0.10
        ):
            raise RuntimeError(
                f"Failed to move above {target}"
            )

        print("\nDescending to placement height")

        if not self.robot.move_to(
            target_x,
            target_y,
            self.PLACE_Z,
            speed=0.04
        ):
            raise RuntimeError(
                "Failed to reach placement position"
            )

        print("\nReleasing object")

        self.robot.open_gripper()
        time.sleep(2)

        print("\nRetracting")

        if not self.robot.move_to(
            target_x,
            target_y,
            self.LIFT_Z,
            speed=0.06
        ):
            raise RuntimeError(
                "Failed to retract"
            )

        print("\n########################################")
        print("       OPERATION COMPLETE")
        print("########################################")
        print(
            f"{source.upper()} → {target.upper()}"
        )
        print("Vision coordinates were captured")
        print("before robot movement.")
        print("########################################")