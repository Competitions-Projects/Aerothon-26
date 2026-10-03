# Jetson Orin Nano Setup Notes

JetPack 7.2.1 (Jetson Linux r39.2.1), installed with the Jetson ISO method. Docker setup at the end for running the Dockerfile on the GPU.

Official guide, everything is downloaded from here:
https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/quick_start.html

Links
- Jetson ISO r39.2.1: https://developer.nvidia.com/downloads/embedded/l4t/r39_release_v2.1/iso/jetsoninstaller-r39.2.1-2026-08-07-18-30-47-arm64.iso
- JetPack downloads: https://developer.nvidia.com/embedded/jetpack/downloads
- BalenaEtcher: https://etcher.balena.io/#download-etcher
- JetPack 6.x update path (only if firmware is old): https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/update_firmware.html
- Docker setup page: https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/setup_docker.html

## Things needed

- Jetson Orin Nano Dev Kit + 19V power supply
- NVMe SSD or microSD (64GB+). Not in the box.
- USB drive, 16 GB+
- PC/laptop with 25 GB+ free
- DisplayPort monitor, keyboard, mouse

ISO goes on the USB drive only, never on the microSD (SD card images aren't supported from JetPack 7.2). The installer copies Jetson Linux from the USB onto the SSD/microSD.

---

# Part 1: Install Jetson Linux

## Step 1: Check firmware (the gatekeeper)

Firmware has to be 36.x or newer, otherwise the JetPack 7 installer won't work.

1. Connect monitor + keyboard, power on.
2. Press Esc a few times when the NVIDIA logo shows. Opens the UEFI menu.
3. Firmware version is near the top.
   - 36.x or newer (JetPack 6.x generation): fine, go to Step 2.
   - Older than 36: do the JetPack 6.x update path first (link above), then come back.

Quick check if it already boots into Linux:

```bash
cat /etc/nv_tegra_release
```

R36.x is fine. R35.x or older means update first.

## Step 2: Download the ISO (on PC)

- Get Jetson ISO r39.2.1 from the link above (or the green download button on NVIDIA's page).
- Saves as a `.iso` file.

## Step 3: Flash the ISO to the USB (on PC)

Can't just copy the file over, it has to be flashed as a bootable installer.

1. Plug in the blank USB (16 GB+).
2. Open BalenaEtcher.
3. Flash from file -> pick the `.iso`.
4. Select target -> pick the USB.
5. Flash! and wait a few minutes.
6. Eject the USB safely and unplug it.

## Step 4: Hardware connections

Board parts:

1. Module with heatsink and fan. The processor is under it.
2. 40-pin expansion header. GPIO for sensors, ICs, expansion boards.
3. Power LED. Green when powered on.
4. USB-C port. Data, recovery mode, host connection. Does not power the board.
5. Gigabit Ethernet (RJ45).
6. 4x USB 3.2 Gen2 Type-A. Flash drive, keyboard, mouse, USB cameras.
7. DisplayPort.
8. DC power jack, 19V barrel.
9. 2x MIPI CSI-2 camera connectors, 22-pin flex.

microSD slot is under the module. NVMe goes on the carrier board. Storage has to be in before powering on, the OS gets installed on it.

**With monitor**

1. Storage installed (NVMe or microSD).
2. DisplayPort cable to the monitor.
3. Keyboard and mouse in the USB ports.
4. USB drive from Step 3 in a USB port.
5. 19V power into the barrel jack. Turns on by itself, green LED next to the USB-C port.

**Headless serial (no monitor)**

1. USB-to-TTL cable from PC to the Button Header on the carrier board:
   - header pin 3 (RXD) -> cable TX
   - header pin 4 (TXD) -> cable RX
   - header pin 7 (GND) -> cable ground
2. Serial console on PC (PuTTY or minicom), 115200 baud.
3. USB drive in the Jetson.
4. NVMe or microSD installed.
5. Plug in the 19V power.

## Step 5: Install Jetson Linux

**Phase 1: boot from the USB**

1. Power on with the USB plugged in.
2. Press Esc as soon as the NVIDIA logo shows.
3. Arrow keys to Boot Manager, Enter.
4. Pick the USB drive, Enter.

**Phase 2: confirm the firmware update (crucial)**

1. Installer checks the board firmware and asks to update the QSPI firmware.
2. Press Y right away.
3. Only 30 seconds to press it. If missed, the install fails later. Restart the install and press Y when the prompt comes up.

**Phase 3: firmware update runs**

- Two rounds.
- Don't touch or unplug anything. It may reboot on its own, that's normal.

**Phase 4: install to SSD or microSD**

1. After the update, the blue GRUB menu shows up. Pick "Install Jetson ISO r39.2.1", Enter.
2. Pick the target: NVMe SSD or microSD.
3. Confirm. This wipes the target drive, so double check which one.
4. Wait for the progress bar, then Reboot.
5. Unplug the USB when it asks.

## Step 6: First boot

Setup wizard:

1. Accept the EULA.
2. Language, keyboard, time zone.
3. Wi-Fi or Ethernet.
4. Username and password.
5. Log in to the Ubuntu desktop.

Then update and install the JetPack stuff (CUDA, TensorRT, etc):

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y nvidia-jetpack
```

## Step 7: MAXN SUPER

Default power mode is usually 25W. MAXN SUPER lets it draw full power for max CPU/GPU speed.

- Desktop: click the power mode in the top bar -> Power Mode -> MAXN SUPER.
- Terminal:

```bash
sudo nvpmodel -q          # current mode
sudo nvpmodel -m 2        # MAXN SUPER
sudo jetson_clocks        # optional, keeps clocks at max
```

Mode numbers can change between releases. Check with `sudo nvpmodel -q --verbose` or `/etc/nvpmodel.conf`.

---

# Part 2: Docker

NVIDIA's page for this is linked at the top.

## Step 1: Install Docker + NVIDIA Container Toolkit

Host only needs Docker and the NVIDIA runtime so containers can use the GPU.

Check if Docker is already there:

```bash
docker --version
```

If not, or the NVIDIA runtime isn't set up:

```bash
sudo apt update
sudo apt install -y nvidia-container curl
curl https://get.docker.com | sh
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl daemon-reload
sudo systemctl restart docker
```

Optional, so sudo isn't needed every time:

```bash
sudo usermod -aG docker $USER
newgrp docker
```

## Step 2: Make nvidia the default runtime

Why: without this, Docker only uses the GPU when `--runtime nvidia` is passed to `docker run`. `docker build` runs CPU only, so any `RUN` line that builds CUDA code, sets up TensorRT, or tests PyTorch fails (no GPU found). With it set, every build step and run uses the NVIDIA runtime.

1. Open the config:

   ```bash
   sudo nano /etc/docker/daemon.json
   ```

2. Contents (keep any settings already in the file, just add these):

   ```json
   {
     "default-runtime": "nvidia",
     "runtimes": {
       "nvidia": {
         "path": "nvidia-container-runtime",
         "runtimeArgs": []
       }
     }
   }
   ```

3. Save and close: Ctrl+O, Enter, Ctrl+X.
4. Restart Docker:

   ```bash
   sudo systemctl restart docker
   ```

5. Check:

   ```bash
   docker info | grep -i runtime
   ```

   Should show `Default Runtime: nvidia`.

## Step 3: Power mode

MAXN SUPER (Part 1, Step 7) so the GPU has full power for containers.

## Step 4: Test the GPU

```bash
docker run --rm -it -v "$PWD":/workspace -w /workspace nvcr.io/nvidia/pytorch:25.08-py3
```

Inside the container:

```bash
python3 -c "import torch; print(torch.cuda.is_available())"
```

Should print `True`.

## Step 5: Dockerfile base image

JetPack 7.2 uses CUDA 13. Images built for JetPack 6 (CUDA 12, r36 tags) can fail with CUDA error 801 or silently run on CPU. Use a CUDA 13 arm64 image, e.g.:

```dockerfile
FROM nvcr.io/nvidia/cuda:13.0.0-devel-ubuntu24.04
```

- arm64 only, build on the Jetson itself.
- Keep images on the NVMe if there is one, they get big.

## Step 6: Build and run

Build from the folder with the Dockerfile:

```bash
docker build -t <your-image-name> .
```

Run with the GPU (`--runtime nvidia` or `--gpus all`, or set it in docker-compose.yml):

```bash
docker run --runtime nvidia --network host -it <your-image-name>
```
