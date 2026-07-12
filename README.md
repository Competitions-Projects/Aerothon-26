# SkyScan — AeroTHON 2026

Autonomous payload-delivery drone system built for **AeroTHON 2026**.  performs autonomous takeoff, QR-based mission entry, corridor navigation, GPS-bounded area survey, obstacle/red-zone avoidance, and precision payload delivery — all running on ROS 2.

Developed by the **Aerounwired Aeromodeling Club**, NIT Calicut.

---

## What This Repo Is For

This is the central codebase for autonomous stack. Different team members are building the following modules **independently and in parallel**:

| Module | Owner | Function |
|---|---|---|
| `qr_scanning` | — | QR code detection & decoding for mission waypoints |
| `YOLO` | — | YOLO-based object/target detection |
| `Object_avoidance` | — | ROS2 and Nav2 Based Obstacle Avoidance |

Each module is a standalone ROS 2 package that can be built, launched, and tested on its own. They're integrated together via shared topics/messages defined in `skyscan_msgs` — see [`docs/INTEGRATION.md`](docs/INTEGRATION.md) for the interface contract (topic names, message types, update rates).

---

## Hardware Stack

- **Flight Controller:** CubePilot Orange
- **Companion Computer:** Jetson Orin Nano 8GB
- **Cameras:** Arducam IMX519 (RGB), Orbbec Gemini 330 (depth)
- **Rangefinder:** MTF-01P (optical flow + rangefinder)
- **OS / Middleware:** Ubuntu 22.04, ROS 2 Humble

---

## Getting Started

### 1. Clone the repo
```bash
git clone https://github.com/Competitions-Projects/Aerothon-26.git
cd Aerothon-26
```

## Contribution Workflow

1. Pull latest `main` before starting work:
   ```bash
   git pull origin main
   ```
2. Create a branch for your module:
   ```bash
   git checkout -b qr-scanning-dev
   ```
3. Commit and push your branch (avoid pushing directly to `main`):
   ```bash
   git add .
   git commit -m "Add QR detection node"
   git push origin qr-scanning-dev
   ```
4. Open a Pull Request to merge into `main` once your module builds and runs cleanly.

---

## Design Report

Full technical design report (LaTeX source + compiled PDF) is in [`docs/design-report/`](docs/design-report/).

---

## Team

Club Aerounwired Aeromodeling Club — NIT Calicut
