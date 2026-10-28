# ARIA-4 — Articulated Robot Intelligent Arm

### LLM-Driven Intelligent Robotic Manipulation with Voice and Vision-Based Control

ARIA-4 is a simulation-based robotic manipulation project that integrates large language models, computer vision, inverse kinematics, voice commands, and workspace safety validation for intelligent pick-and-place operations.

## Key Features

- Natural-language robot control using a locally hosted LLM
- Voice-command input
- OpenCV-based color object detection
- Vision-based object localization
- Custom damped least-squares inverse kinematics
- Forward kinematics and numerical validation
- Workspace and motion safety validation
- CoppeliaSim robotic manipulation environment
- Automated object pick-and-place execution

## Technology Stack

- Python
- CoppeliaSim
- OpenCV
- NumPy
- Ollama and Qwen3
- ZeroMQ Remote API

## Robot Platform

The simulation uses a four-degree-of-freedom robotic arm based on the WLKATA MT4 CAD model.

The original CAD geometry was developed by WLKATA Robotics and is available at:

https://github.com/wlkata/MT4-STL

The original model is provided under Apache License 2.0. The ARIA-4 project does not claim authorship of the original mechanical geometry.

## Project Structure

- `src/` — Main software implementation
- `simulation/` — CoppeliaSim simulation scene
- `experiments/` — Experimental scripts and benchmarks
- `results/` — Validation results and data
- `assets/` — Project images and supporting resources

## Research and Experimental Validation

The project includes experiments evaluating forward kinematics, independent inverse kinematics, workspace reachability, safety validation, vision localization, and perception robustness.

## Current Status

Integrated pick-and-place execution has been demonstrated in simulation. Physical robot deployment remains future work.

## License

Original project software and third-party assets are subject to their applicable licenses. See `LICENSE` and `THIRD_PARTY_NOTICES.md` for details.