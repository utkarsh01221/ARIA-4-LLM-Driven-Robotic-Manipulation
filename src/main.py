from robot_execution import RobotExecution
from vision_object_manager import VisionObjectManager
from motion_planner import MotionPlanner
from llm_command_parser import LLMCommandParser
from safety_validator import SafetyValidator
from voice_input import VoiceInput


def main():

    robot = RobotExecution()
    vision_objects = None

    try:
        vision_objects = VisionObjectManager()

        planner = MotionPlanner(
            robot,
            vision_objects
        )

        parser = LLMCommandParser()

        safety = SafetyValidator(
            vision_objects
        )

        voice = VoiceInput()

        print("\n========================================")
        print("       VOICE LLM ROBOT SYSTEM READY")
        print("========================================")

        vision_objects.print_objects()

        while True:

            print("\n----------------------------------------")
            print("Say a robot command.")
            print("Say 'exit' or 'quit' to stop.")
            print("----------------------------------------")

            command = voice.listen()

            if command is None:
                continue

            if command.lower().strip() in {"exit", "quit"}:
                break

            try:

                # ==============================
                # VOICE → LLM
                # ==============================

                instruction = parser.parse(command)

                # ==============================
                # SAFETY CHECK
                # ==============================

                safety.validate_command(
                    instruction
                )

                # ==============================
                # ROBOT EXECUTION
                # ==============================

                if instruction["action"] == "pick_and_place":

                    planner.pick_and_place(
                        instruction["source"],
                        instruction["target"]
                    )

            except (ValueError, RuntimeError) as e:

                print(
                    f"\nCommand rejected: {e}"
                )

    except Exception as e:

        print(
            f"\nSYSTEM ERROR: {e}"
        )

    finally:

        if vision_objects is not None:
            vision_objects.close()

        robot.close()


if __name__ == "__main__":
    main()