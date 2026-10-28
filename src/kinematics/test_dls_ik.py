from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from dls_ik import DLSInverseKinematics


def main():

    client = RemoteAPIClient()

    sim = client.require("sim")

    joint_paths = [

        "/base_link/Joint1",

        "/base_link/Joint1/"
        "Link1/Joint2",

        "/base_link/Joint1/"
        "Link1/Joint2/"
        "Link2/Joint3",

        "/base_link/Joint1/"
        "Link1/Joint2/"
        "Link2/Link3/Joint4"
    ]

    tip_path = (
        "/base_link/Joint1/"
        "Link1/Joint2/"
        "Link2/Joint3/"
        "Link3/Joint4/"
        "Link4/IK_Tip"
    )

    dls = DLSInverseKinematics(
        sim=sim,
        client=client,
        joint_paths=joint_paths,
        tip_path=tip_path,
        damping=0.5,
        max_iterations=100,
        tolerance=0.005,
        numerical_step=1e-5
    )

    target = [
        0.85,
        -0.35,
        0.420
    ]

    result = dls.solve(
        target
    )

    print("\n========================================")
    print("             DLS RESULT")
    print("========================================")

    print(
        f"Success    : "
        f"{result['success']}"
    )

    print(
        f"Iterations : "
        f"{result['iterations']}"
    )

    print(
        f"Error      : "
        f"{result['error']:.6f} m"
    )

    print(
        f"Time       : "
        f"{result['time']:.3f} s"
    )

    print(
        "\nJoint positions:"
    )

    for i, q in enumerate(
        result["joint_positions"],
        start=1
    ):

        print(
            f"q{i} = "
            f"{q:.6f} rad "
            f"({q * 180 / 3.141592653589793:.2f}°)"
        )


if __name__ == "__main__":
    main()