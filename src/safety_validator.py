class SafetyValidator:

    COLORS = {"red", "blue", "green", "yellow"}

    def __init__(self, object_manager):
        self.objects = object_manager

    def validate_command(self, instruction):

        if not isinstance(instruction, dict):
            raise ValueError("Invalid robot instruction.")

        if instruction.get("action") != "pick_and_place":
            raise ValueError("Unsafe or unsupported action.")

        source = instruction.get("source")
        target = instruction.get("target")

        if source not in self.COLORS:
            raise ValueError(
                f"Unsafe source object: {source}"
            )

        if target not in self.COLORS:
            raise ValueError(
                f"Unsafe target object: {target}"
            )

        if source == target:
            raise ValueError(
                "Source and target cannot be the same."
            )

        print("\n========================================")
        print("       COMMAND SAFETY CHECK")
        print("========================================")

        print(f"Source : {source}")
        print(f"Target : {target}")

        print("\nChecking source object...")
        if not self.objects.exists(source):
            raise RuntimeError(
                f"Safety check failed: {source.upper()} object not detected."
            )

        print(f"{source.upper()} detected.")

        print("\nChecking target object...")
        if not self.objects.exists(target):
            raise RuntimeError(
                f"Safety check failed: {target.upper()} object not detected."
            )

        print(f"{target.upper()} detected.")

        print("\nSAFETY CHECK PASSED")
        print("========================================")

        return True