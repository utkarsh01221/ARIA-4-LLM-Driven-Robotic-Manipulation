# ============================================================
# ARIA-4 CARTESIAN WORKSPACE VALIDATOR
#
# Application-level safety envelope.
#
# IMPORTANT:
# These bounds represent the permitted operational workspace,
# NOT the complete theoretical kinematic workspace of ARIA-4.
# ============================================================


class WorkspaceValidator:

    X_MIN = 0.70
    X_MAX = 1.05

    Y_MIN = -0.60
    Y_MAX = 0.00

    Z_MIN = 0.30
    Z_MAX = 0.60

    TOLERANCE = 1e-9

    @classmethod
    def validate(cls, x, y, z):

        reasons = []

        # ----------------------------------------------------
        # X CHECK
        # ----------------------------------------------------

        if x < cls.X_MIN - cls.TOLERANCE:

            reasons.append(
                f"X={x:.6f} below X_MIN={cls.X_MIN:.6f}"
            )

        elif x > cls.X_MAX + cls.TOLERANCE:

            reasons.append(
                f"X={x:.6f} above X_MAX={cls.X_MAX:.6f}"
            )

        # ----------------------------------------------------
        # Y CHECK
        # ----------------------------------------------------

        if y < cls.Y_MIN - cls.TOLERANCE:

            reasons.append(
                f"Y={y:.6f} below Y_MIN={cls.Y_MIN:.6f}"
            )

        elif y > cls.Y_MAX + cls.TOLERANCE:

            reasons.append(
                f"Y={y:.6f} above Y_MAX={cls.Y_MAX:.6f}"
            )

        # ----------------------------------------------------
        # Z CHECK
        # ----------------------------------------------------

        if z < cls.Z_MIN - cls.TOLERANCE:

            reasons.append(
                f"Z={z:.6f} below Z_MIN={cls.Z_MIN:.6f}"
            )

        elif z > cls.Z_MAX + cls.TOLERANCE:

            reasons.append(
                f"Z={z:.6f} above Z_MAX={cls.Z_MAX:.6f}"
            )

        # ----------------------------------------------------
        # RESULT
        # ----------------------------------------------------

        accepted = len(reasons) == 0

        return {
            "accepted": accepted,
            "reasons": reasons
        }


def main():

    print("\n============================================")
    print("       ARIA-4 WORKSPACE VALIDATOR")
    print("============================================")

    print(
        f"X: [{WorkspaceValidator.X_MIN}, "
        f"{WorkspaceValidator.X_MAX}] m"
    )

    print(
        f"Y: [{WorkspaceValidator.Y_MIN}, "
        f"{WorkspaceValidator.Y_MAX}] m"
    )

    print(
        f"Z: [{WorkspaceValidator.Z_MIN}, "
        f"{WorkspaceValidator.Z_MAX}] m"
    )

    test_points = [

        ("CENTER", 0.85, -0.30, 0.385),

        ("X_MIN", 0.70, -0.30, 0.385),

        ("X_MAX", 1.05, -0.30, 0.385),

        ("Y_MIN", 0.85, -0.60, 0.385),

        ("Y_MAX", 0.85, 0.00, 0.385),

        ("Z_MIN", 0.85, -0.30, 0.30),

        ("Z_MAX", 0.85, -0.30, 0.60),

        ("OUTSIDE_X_LOW", 0.699, -0.30, 0.385),

        ("OUTSIDE_X_HIGH", 1.051, -0.30, 0.385),

        ("OUTSIDE_Y_LOW", 0.85, -0.601, 0.385),

        ("OUTSIDE_Y_HIGH", 0.85, 0.001, 0.385),

        ("OUTSIDE_Z_LOW", 0.85, -0.30, 0.299),

        ("OUTSIDE_Z_HIGH", 0.85, -0.30, 0.601),
    ]

    print("\n============================================")
    print("              TEST RESULTS")
    print("============================================")

    for name, x, y, z in test_points:

        result = WorkspaceValidator.validate(
            x,
            y,
            z
        )

        status = (
            "ACCEPTED"
            if result["accepted"]
            else "REJECTED"
        )

        print(
            f"\n{name}"
        )

        print(
            f"Point  : "
            f"[{x:.6f}, {y:.6f}, {z:.6f}]"
        )

        print(
            f"Result : {status}"
        )

        if result["reasons"]:

            for reason in result["reasons"]:

                print(
                    f"Reason : {reason}"
                )


if __name__ == "__main__":
    main()