# ARIA-4 DLS vs CoppeliaSim simIK Benchmark

## Experimental setup

- Robot: ARIA-4, 4-DOF articulated manipulator
- Targets: 20 identical Cartesian XYZ targets for both methods
- Cartesian convergence tolerance: 5 mm
- DLS damping parameter: 0.5
- DLS maximum iterations: 100
- DLS numerical Jacobian central-difference step: 1e-5 rad
- simIK: existing CoppeliaSim simIK controller, XYZ position constraints
- simIK calculation method: damped least squares
- simIK group maximum internal iterations: 100

## Results

| Metric | Independent DLS | CoppeliaSim simIK |
|---|---:|---:|
| Test targets | 20 | 20 |
| Success rate | 100.00% | 100.00% |
| Mean final Cartesian error | 4.201 mm | 0.230 mm |
| Minimum final error | 2.939 mm | 0.115 mm |
| Maximum final error | 4.938 mm | 0.352 mm |
| Mean measured time | 13.350 s | 0.050 s |
| Mean DLS solver iterations | 8.95 | N/A |
| Mean external simIK convergence steps | N/A | 1.00 |

## Key observation

Across the 20 identical target positions, both methods achieved 100% success under the 5 mm convergence criterion. The independent numerical-Jacobian DLS implementation produced a mean final Cartesian error of 4.201 mm, while the CoppeliaSim simIK configuration produced 0.230 mm. The simIK mean error is approximately 94.5% lower than the DLS mean error.

## Important interpretation of timing

The measured times should **not** be presented as a pure algorithmic speed comparison. The DLS experiment repeatedly computes a numerical Jacobian through Remote API simulation stepping, while the simIK controller executes inside CoppeliaSim's simulation loop. Therefore, the measured wall-clock times include different communication and simulation overheads.

Similarly, the DLS iteration count and the Python-observed simIK convergence-step count are not equivalent quantities. The simIK group itself can perform multiple internal iterations during a group calculation.

## Research conclusion

The experiment demonstrates that the independently implemented DLS solver is able to solve the tested ARIA-4 Cartesian position targets reliably, achieving 100% convergence under the 5 mm criterion. The existing CoppeliaSim simIK implementation provides substantially smaller final Cartesian errors for the same target set. Therefore, simIK is suitable as the production/reference IK controller for ARIA-4, while the independent DLS implementation provides a useful transparent numerical IK implementation for research validation and comparison.

## Recommended paper wording

> An independent damped-least-squares inverse-kinematics solver was developed for ARIA-4 using a numerically estimated Cartesian Jacobian and evaluated against the CoppeliaSim simIK solver. Both methods were tested on the same 20 Cartesian target positions using a 5 mm convergence criterion. Both approaches achieved a 100% success rate. The independent DLS solver obtained a mean final Cartesian position error of 4.201 mm, whereas the CoppeliaSim simIK configuration achieved 0.230 mm. The results indicate that the independent solver provides reliable numerical IK performance, while the integrated simIK implementation achieves lower residual position error for the tested workspace.

## Files

- `ARIA-4_DLS_vs_simIK_comparison.csv` — target-by-target comparison
- `ARIA-4_DLS_vs_simIK_summary.csv` — summary metrics
- `ARIA-4_error_comparison.png` — error graph
- `ARIA-4_time_comparison.png` — measured time graph
- `ARIA-4_error_distribution.png` — error distribution
- `ARIA-4_DLS_vs_simIK_report.md` — research-ready analysis
