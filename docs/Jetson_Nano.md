# Jetson Orin Nano Setup Notes (headless, laptop only)

JetPack 7.2.1 (Jetson Linux r39.2.1), Jetson ISO method. No monitor, so everything is done with a laptop: serial cable for the first setup, SSH after that. Docker at the end for running the Dockerfile on the GPU.

Official guide, everything is downloaded from here:
https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/quick_start.html

Links
- Jetson ISO r39.2.1: https://developer.nvidia.com/downloads/embedded/l4t/r39_release_v2.1/iso/jetsoninstaller-r39.2.1-2026-08-07-18-30-47-arm64.iso
- JetPack downloads: https://developer.nvidia.com/embedded/jetpack/downloads
- BalenaEtcher: https://etcher.balena.io/#download-etcher
- JetPack 6.x update path (only if firmware is old): https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/update_firmware.html
- Docker setup page: https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/setup_docker.html
- Hardware layout (Button Header pins): https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html

## Things needed

- Jetson Orin Nano Dev Kit + 19V power supply
- NVMe SSD or microSD (64GB+). Not in the box.
- USB drive, 16 GB+
- Laptop with 25 GB+ free
- USB-to-TTL serial adapter, 3.3V logic (CP2102, FTDI or CH340) + 3 female-to-female jumper wires
- Phone with hotspot (for SSH when there's no router)
- Ethernet cable (optional, more reliable than Wi-Fi)

Notes
- Serial adapter has to be 3.3V. Don't connect its 5V/VCC wire to anything.
- ISO goes on the USB drive only, never on the microSD (SD card images aren't supported from JetPack 7.2). The installer copies Jetson Linux from the USB onto the SSD/microSD.
- Wi-Fi module is already on the board (comes with the dev kit). Nothing to install.
- If a DisplayPort monitor + keyboard can be borrowed for an hour, Part 1 gets easier (no serial needed). NVIDIA's page has a monitor version of every step.

---

# Part 1: Install Jetson Linux (serial console)

## Step 1: Make the ISO USB (on laptop)

1. Download Jetson ISO r39.2.1 from the link above. Saves as a `.iso` file.
2. Can't just copy the file over, it has to be flashed as a bootable installer.
3. Plug in the blank USB (16 GB+), open BalenaEtcher.
4. Flash from file -> pick the `.iso`.
5. Select target -> pick the USB.
6. Flash! and wait a few minutes. Eject the USB.

## Step 2: Hardware connections

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

Connections:

1. NVMe or microSD installed.
2. ISO USB drive in a USB-A port.
3. Serial adapter to the Button Header on the carrier board (3 wires):
   - header pin 3 (RXD) -> adapter TX
   - header pin 4 (TXD) -> adapter RX
   - header pin 7 (GND) -> adapter GND
   - TX and RX are crossed on purpose
   - pin 1 location: see the picture under "Headless serial" on NVIDIA's quick start page, or the Hardware Layout page
4. Adapter USB end into the laptop.
5. 19V power goes in last, when ready to power on. Turns on by itself, green LED next to the USB-C port.

## Step 3: Open the serial console (on laptop)

Do this before powering on the Jetson. The window stays blank until it boots.

**Windows**
1. Device Manager -> Ports (COM & LPT) -> note the port, e.g. COM5.
2. No port showing: install the driver for the adapter chip (CP210x or CH340).
3. PuTTY -> Serial -> COM port -> speed 115200 -> Open.

**Mac / Linux**
```bash
ls /dev/tty*                      # find the port, e.g. /dev/ttyUSB0
screen /dev/ttyUSB0 115200
```
Linux: may need sudo, or add the user to the `dialout` group. Mac port looks like `/dev/tty.usbserial-xxxx`.

## Step 4: Check firmware (the gatekeeper)

Firmware has to be 36.x or newer, otherwise the JetPack 7 installer won't work.

1. Power on the Jetson.
2. Click the serial window, press Esc repeatedly. Opens the UEFI menu.
3. Firmware version is near the top.
   - 36.x or newer (JetPack 6.x generation): fine, go to Step 5.
   - Older than 36: do the JetPack 6.x update path first (link above), then come back.
4. Power off.

If the screen stays blank or drops to a UEFI shell when booting the ISO, the firmware is probably too old. Power off, don't keep retrying.

## Step 5: Install Jetson Linux

**Phase 1: boot from the USB**

1. Power on with the USB plugged in.
2. Press Esc in the serial window while the pre-boot options show.
3. Arrow keys to Boot Manager, Enter.
4. Pick the USB drive, Enter.

**Phase 2: confirm the firmware update (crucial)**

1. Installer checks the board firmware and asks to update the QSPI firmware.
2. Press Y right away. Keep the serial window focused.
3. Only 30 seconds to press it. If missed, the install fails later. Restart the install and press Y when the prompt comes up.

**Phase 3: firmware update runs**

- Two rounds.
- Don't touch or unplug anything. It may reboot on its own, that's normal.

**Phase 4: install to SSD or microSD**

1. After the update, the GRUB menu shows up. Pick "Install Jetson ISO r39.2.1", Enter.
2. Pick the target: NVMe SSD or microSD.
3. Confirm. This wipes the target drive, so double check which one.
4. Wait for the progress bar, then Reboot.
5. Unplug the USB when it asks.

## Step 6: First setup (still over serial)

The setup questions should show up as text in the serial window. NVIDIA's page doesn't describe the headless version of this, so the exact look isn't confirmed. If nothing shows up after a few minutes, a monitor is needed for this one step.

1. Accept the EULA.
2. Language, keyboard, time zone.
3. Network: Wi-Fi or Ethernet. Skip if not offered, do it in Step 7.
4. Username, password, computer name.
5. Log in.

Write down the username, password and computer name.

## Step 7: Wi-Fi and IP address (in the serial window)

Join Wi-Fi if not done in the setup:

```bash
nmcli device wifi list
sudo nmcli device wifi connect "WIFI_NAME" password "WIFI_PASSWORD"
```

Get the IP:

```bash
hostname -I
```

Looks like `192.168.1.45`. Write it down.

College/hostel Wi-Fi often blocks devices from talking to each other. If SSH fails in Step 8, use the phone hotspot: laptop and Jetson both on it. Or Ethernet cable from the Jetson to the router.

---

# Part 2: Connect from the laptop (SSH)

Serial cable isn't needed after this.

## Step 1: SSH in

Laptop on the same Wi-Fi as the Jetson. In a terminal (PowerShell on Windows):

```bash
ssh username@192.168.1.45
```

Or with the computer name: `ssh username@computername.local`. Type the password.

## Step 2: Update and install JetPack parts

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y nvidia-jetpack
```

## Step 3: MAXN SUPER

Default power mode is usually 25W. MAXN SUPER lets it draw full power for max CPU/GPU speed. No desktop menu without a monitor, so use the terminal:

```bash
sudo nvpmodel -q          # current mode
sudo nvpmodel -m 2        # MAXN SUPER
sudo jetson_clocks        # optional, keeps clocks at max
```

Mode numbers can change between releases. Check with `sudo nvpmodel -q --verbose` or `/etc/nvpmodel.conf`. NVIDIA's page only shows the desktop menu way, so verify the number on the board.

## Step 4: Copy files to the Jetson

From the laptop:

```bash
scp -r my_project username@192.168.1.45:~/
```

Or push to GitHub and `git clone` on the Jetson. VS Code "Remote - SSH" extension also works for editing files on the Jetson from the laptop.

---

# Part 3: Docker

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

## Step 3: Test the GPU

```bash
docker run --rm -it -v "$PWD":/workspace -w /workspace nvcr.io/nvidia/pytorch:25.08-py3
```

Inside the container:

```bash
python3 -c "import torch; print(torch.cuda.is_available())"
```

Should print `True`.

## Step 4: Dockerfile base image

JetPack 7.2 uses CUDA 13. Images built for JetPack 6 (CUDA 12, r36 tags) can fail with CUDA error 801 or silently run on CPU. Use a CUDA 13 arm64 image, e.g.:

```dockerfile
FROM nvcr.io/nvidia/cuda:13.0.0-devel-ubuntu24.04
```

- arm64 only, build on the Jetson itself.
- Keep images on the NVMe if there is one, they get big.

## Step 5: Build and run

Build from the folder with the Dockerfile:

```bash
docker build -t <your-image-name> .
```

Run with the GPU (`--runtime nvidia` or `--gpus all`, or set it in docker-compose.yml):

```bash
docker run --runtime nvidia --network host -it <your-image-name>
```

## Step 6: YOLO speed

A `.pt` model runs slowly on the Jetson. Convert to TensorRT on the Jetson itself, then load the `.engine` file in the ROS code:

```bash
yolo export model=best.pt format=engine half=True
```

---

# Part 4: On the drone

## Power

- Battery is 6S (about 22 to 25V). Too high to plug straight into the Jetson.
- Use a BEC set to 12V, rated 5A or more, into the barrel jack. Jetson dev kit input is roughly 9 to 19V, check the carrier board spec.
- Check the BEC output with a multimeter before connecting the Jetson. Many BECs default to 5V, which won't run it.
- Check barrel jack polarity with a multimeter against the supplied adapter. Center pin should be positive.
- Don't power it from the Pixhawk power module. Its output is 5V and too weak.
- USB-C does not power the board.

## Flight controller link

- Jetson to CubePilot over serial (UART), usually through MAVROS.
- Jetson 40-pin header UART pins to the CubePilot TELEM port, with ground connected.
- Exact pins and the MAVROS command: to be filled in once the TELEM port is chosen.

## Start on boot

No typing in flight, so run the container detached with a restart policy:

```bash
docker run -d --restart unless-stopped --runtime nvidia --network host <your-image-name>
```

Starts every time the Jetson powers on.

## Reaching it in the field

- Venue may have no Wi-Fi. Use the phone hotspot, with the laptop and Jetson both on it, then SSH to the Jetson's IP.
- Join the hotspot from the Jetson ahead of time (`nmcli`, Part 1 Step 7) so it connects by itself on boot.
- Test this at home first.
