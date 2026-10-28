from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
DLS = ROOT / "dls_batch_results.csv"
SIMIK = ROOT / "simik_batch_results.csv"
OUT = ROOT / "benchmark_output"
OUT.mkdir(exist_ok=True)

dls = pd.read_csv(DLS)
simik = pd.read_csv(SIMIK)

for df in (dls, simik):
    df["success"] = df["success"].astype(str).str.lower().map(
        {"true": True, "false": False, "1": True, "0": False}
    ).fillna(False)

comparison = pd.DataFrame({
    "test": dls["test"],
    "target_x_m": dls["target_x"],
    "target_y_m": dls["target_y"],
    "target_z_m": dls["target_z"],
    "dls_success": dls["success"],
    "simik_success": simik["success"],
    "dls_error_mm": dls["final_error_mm"],
    "simik_error_mm": simik["final_error_mm"],
    "dls_time_s": dls["time_s"],
    "simik_time_s": simik["time_s"],
    "dls_iterations": dls["iterations"],
    "simik_convergence_steps": simik["convergence_steps"],
})

comparison["error_difference_mm"] = (
    comparison["dls_error_mm"] - comparison["simik_error_mm"]
)

comparison["dls_error_reduction_percent"] = (
    comparison["error_difference_mm"] /
    comparison["dls_error_mm"] * 100
)

comparison.to_csv(OUT / "ARIA-4_DLS_vs_simIK_comparison.csv", index=False)

summary = pd.DataFrame({
    "Metric": [
        "Test targets",
        "DLS success rate (%)",
        "simIK success rate (%)",
        "DLS mean error (mm)",
        "simIK mean error (mm)",
        "DLS max error (mm)",
        "simIK max error (mm)",
        "DLS mean time (s)",
        "simIK mean time (s)",
        "DLS mean iterations",
        "simIK mean convergence steps",
    ],
    "Value": [
        len(dls),
        dls["success"].mean() * 100,
        simik["success"].mean() * 100,
        dls["final_error_mm"].mean(),
        simik["final_error_mm"].mean(),
        dls["final_error_mm"].max(),
        simik["final_error_mm"].max(),
        dls["time_s"].mean(),
        simik["time_s"].mean(),
        dls["iterations"].mean(),
        simik["convergence_steps"].mean(),
    ]
})

summary.to_csv(OUT / "ARIA-4_DLS_vs_simIK_summary.csv", index=False)

plt.figure(figsize=(10, 5.5))
plt.plot(comparison["test"], comparison["dls_error_mm"], marker="o", label="Independent DLS")
plt.plot(comparison["test"], comparison["simik_error_mm"], marker="o", label="CoppeliaSim simIK")
plt.axhline(5.0, linestyle="--", label="5 mm tolerance")
plt.xlabel("Test target")
plt.ylabel("Final Cartesian position error (mm)")
plt.title("ARIA-4: DLS vs simIK Final Cartesian Error")
plt.xticks(comparison["test"])
plt.grid(True, alpha=0.25)
plt.legend()
plt.tight_layout()
plt.savefig(OUT / "ARIA-4_error_comparison.png", dpi=200)
plt.close()

plt.figure(figsize=(10, 5.5))
plt.plot(comparison["test"], comparison["dls_time_s"], marker="o", label="Independent DLS")
plt.plot(comparison["test"], comparison["simik_time_s"], marker="o", label="CoppeliaSim simIK")
plt.xlabel("Test target")
plt.ylabel("Measured elapsed time (s)")
plt.title("ARIA-4: DLS vs simIK Measured Experiment Time")
plt.xticks(comparison["test"])
plt.grid(True, alpha=0.25)
plt.legend()
plt.tight_layout()
plt.savefig(OUT / "ARIA-4_time_comparison.png", dpi=200)
plt.close()

plt.figure(figsize=(8, 5.5))
plt.boxplot(
    [dls["final_error_mm"], simik["final_error_mm"]],
    labels=["Independent DLS", "CoppeliaSim simIK"]
)
plt.ylabel("Final Cartesian position error (mm)")
plt.title("ARIA-4: Final Cartesian Error Distribution")
plt.grid(True, axis="y", alpha=0.25)
plt.tight_layout()
plt.savefig(OUT / "ARIA-4_error_distribution.png", dpi=200)
plt.close()

print("Benchmark analysis completed.")
print(f"Outputs: {OUT}")
