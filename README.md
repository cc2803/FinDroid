
# FinDroid

**FinDroid** is an autonomous rover prototype simulated using **Webots r2025a**. It integrates software modules for control, perception, and simulation environments, enabling research and development of navigation behaviors in a robotics context. 

---

## Table of Contents

* [Features](#features)
* [Getting Started](#getting-started)

  * [Prerequisites](#prerequisites)
  * [Installation](#installation)
  * [Running the Simulation](#running-the-simulation)
* [Project Structure](#project-structure)
* [Usage](#usage)
* [Contributing](#contributing)

---

## Features

* Autonomous rover behavior in simulation, pre-Trained YOLOv5n model used for object classification.
* Integration with Webots simulation environments
* Modular structure supporting controllers, objects, and world definitions
* Use of **Python** and **C++** for controller logic and simulation interface ([GitHub][1])
* Results relayed to a FLASK web application to display findings.

---

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing.

### Prerequisites

* Webots r2025a installed
* Python (version X.Y)
* C++ compiler (supporting the standard used in project)
* Any dependencies (Python modules, etc.) listed in `requirements.txt` or similar

### Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/cc2803/FinDroid.git
   cd FinDroid
   ```

2. Install Python dependencies:

   ```bash
   pip install -r requirements.txt
   ```

3. Build any C++ components if necessary (if there are compiled controllers).

### Running the Simulation

1. Open Webots, load an appropriate world file from `worlds/` directory.
2. Ensure the controllers are built / accessible.
3. Start the simulation. You should see the rover in the simulated environment behaving according to the controller logic.

---

## Project Structure

Here’s a high‑level view of the repo layout:

| Directory / File  | Purpose                                                             |
| ----------------- | ------------------------------------------------------------------- |
| `controllers/`    | Contains the control logic (Python / C++) for the rover’s behavior. |
| `objects_webots/` | Definition of objects used in the simulation environment.           |
| `protos/`         | Custom Webots proto files defining robot and scene components.      |
| `worlds/`         | World files for Webots — environments in which rover is simulated.  |

---

## Usage

* Customize or extend controller behavior by modifying files in `controllers/`.
* Design new environments by adding to `worlds/`.
* Add or modify objects, robot designs in `protos/` or `objects_webots/`.
* Use Webots tools to record, visualize, or analyze rover behavior.

---

## Contributing

Contributions are welcome! Here’s how you can help:

* Reporting bugs / issues
* Proposing new features or improvements
* Submitting pull requests with well‑documented changes
* Adding tests or example scenarios

Please follow the style / contribution guidelines (if any), write clear commit messages, and include explanations for major architectural decisions.

---

[1]: https://github.com/cc2803/FinDroid "GitHub - cc2803/FinDroid: An autonomous rover prototype simulated on WeBots r2025a software."
