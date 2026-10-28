from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math


JOINT_PATHS = [
    "/base_link/Joint1",
    "/base_link/Joint1/Link1/Joint2",
    "/base_link/Joint1/Link1/Joint2/Link2/Joint3",
    "/base_link/Joint1/Link1/Joint2/Link2/Joint3/Link3/Joint4",
]


def main():

    client = RemoteAPIClient()
    sim = client.require("sim")

    print("\n========================================")
    print("        ARIA-4 JOINT LIMITS")
    print("========================================")

    for i, path in enumerate(
        JOINT_PATHS,
        start=1
    ):

        joint = sim.getObject(path)

        cyclic, interval = (
            sim.getJointInterval(joint)
        )

        minimum = interval[0]
        range_value = interval[1]

        maximum = minimum + range_value

        print(f"\nJOINT {i}")

        print(f"Cyclic : {cyclic}")

        print(
            f"Minimum: {minimum:.6f} rad "
            f"({math.degrees(minimum):.3f} deg)"
        )

        print(
            f"Maximum: {maximum:.6f} rad "
            f"({math.degrees(maximum):.3f} deg)"
        )

        print(
            f"Range  : {range_value:.6f} rad "
            f"({math.degrees(range_value):.3f} deg)"
        )


if __name__ == "__main__":
    main()