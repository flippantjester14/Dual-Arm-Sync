````markdown
# Dual-Arm Visual Synchronization (Joint-Space Control)

This repository contains a PyBullet-based simulation of decentralized dual-arm visual tracking.  
Robot A executes predefined motion patterns while Robot B tracks the object using a wrist camera and joint-space control with redundancy handling.

The main entry point for the simulation is:

- `run_complete_simulation.py`

---

## 1. Prerequisites

- Ubuntu (or any Linux with Python 3 installed)
- Python 3.8+ recommended
- Git

Check your Python version:

```bash
python3 --version
````

---

## 2. Clone the Repository

```bash
git clone https://github.com/flippantjester14/Dual-Arm-Sync.git
cd Dual-Arm-Sync
```

---

## 3. Create and Activate Virtual Environment

Create a virtual environment named `venv`:

```bash
python3 -m venv venv
```

Activate it:

```bash
source venv/bin/activate
```

You should see `(venv)` in your terminal prompt.

To deactivate later:

```bash
deactivate
```

---

## 4. Install Python Dependencies

Make sure you are inside the `Dual-Arm-Sync` folder and the virtual environment is active.

Install all required packages from `requirements.txt`:

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

If you get any build errors, install missing system packages as needed (for example, `sudo apt-get install python3-dev`).

---

## 5. Run the Simulation

With the virtual environment active and dependencies installed:

```bash
python3 run_complete_simulation.py
```

This will:

* Launch PyBullet with GUI
* Spawn Robot A and Robot B
* Run the motion patterns for Robot A
* Run joint-space visual tracking for Robot B using the wrist camera

If you want to run without GUI (headless), you can modify the script to disable GUI or pass a flag if implemented.

---

## 6. Typical Workflow Summary

Each time you want to run the project:

```bash
cd Dual-Arm-Sync
source venv/bin/activate
python3 run_complete_simulation.py
```

After you are done:

```bash
deactivate
```

---

## 7. Folder Structure (high-level)

* `run_complete_simulation.py` – main entry script
* `requirements.txt` – Python dependencies
* Other Python modules – controllers, vision, utility functions
* `LICENSE` – license file for this project
