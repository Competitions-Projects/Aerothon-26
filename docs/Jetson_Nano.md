# Jetson Orin Nano Setup, JetPack 6.2 (headless with a TTL cable)

Notes for getting the Orin Nano Dev Kit running on JetPack 6.2 with no monitor, just a laptop and a USB-to-TTL serial cable. The serial cable is only for the first setup. After that it's SSH over Wi-Fi. Written to look back at later, so it's casual but every step is in there. Docker is at the end for running the Dockerfile on the GPU.

Official guide this is based on:
https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/quick_start.html

That page is mostly about JetPack 7.2.1. The JetPack 6 stuff lives on the "JetPack 6.x Update Path" page, and that's what these notes follow.

Links
- JetPack 6.x Update Path (the main page for this): https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/update_firmware.html
- JetPack 6.2.1 SD card image: https://developer.nvidia.com/embedded/jetpack-sdk-621
- JetPack 5.1.3 SD card image (only for the firmware bridge): https://developer.nvidia.com/embedded/jetpack-sdk-513
- BalenaEtcher: https://etcher.balena.io/#download-etcher
- Hardware layout (Button Header): https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html
- Carrier board spec (Button Header details): https://developer.nvidia.com/embedded/downloads

## Things needed

- Jetson Orin Nano Dev Kit + the 19V power supply from the box
- microSD card, 64GB UHS-1 or bigger. Not in the box.
- Laptop/PC with a microSD card reader (a USB one is fine)
- USB-to-TTL serial adapter, 3.3V logic (CP2102, FTDI or CH340) + 3 female-to-female jumper wires
- Internet for the Jetson. Ethernet cable is strongly recommended here, setting up Wi-Fi over a text console is a pain.
- Phone with hotspot (handy later for SSH when there's no router)

The adapter has to be 3.3V. The 5V/VCC wire stays unconnected.

## How it goes

JetPack 6.2 goes on as an SD card image, so it's just flashing a microSD card with Etcher and booting from it. No ISO and no Ubuntu PC needed (the ISO method is only for JetPack 7.2 and later).

The catch is the firmware. Some dev kits come with old factory firmware that can't boot JetPack 6 at all. So first thing is checking the firmware version.

- Firmware is 36.x or newer: flash JetPack 6.2 and boot, done.
- Firmware is older than 36: go through a JetPack 5.1.3 card first just to update the firmware, then flash JetPack 6.2.

The whole thing is done through the serial console, so the cable stays connected until SSH works.

NVMe note: this way installs to the microSD card. Putting it on an NVMe SSD needs NVIDIA SDK Manager on an Ubuntu PC, which isn't covered here.

---

# Part 1: Get JetPack 6.2 on the board

## Step 1: Wire up the serial cable

The serial adapter goes on the Button Header on the carrier board. Three wires:

- header pin 3 (RXD) to the adapter TX wire
- header pin 4 (TXD) to the adapter RX wire
- header pin 7 (GND) to the adapter ground wire

TX and RX are swapped on purpose, that's how serial works. To find pin 1, look at the picture under "Headless serial" on NVIDIA's Update Path page (the link at the top) or the Hardware Layout page.

USB end of the adapter goes into the laptop. The 19V power supply stays unplugged for now.

## Step 2: Open the serial console on the laptop

Do this before powering on the Jetson. The window stays blank until the board boots.

**Windows**
1. Device Manager, Ports (COM & LPT), note the port (something like COM5).
2. If no port shows up, install the driver for the adapter chip (CP210x or CH340).
3. PuTTY: Connection type Serial, Serial line is the COM port, Speed is 115200, then Open.

**Mac / Linux**
```bash
ls /dev/tty*                      # find the port, e.g. /dev/ttyUSB0
screen /dev/ttyUSB0 115200
```
On Linux it might need sudo, or add the user to the `dialout` group. On a Mac the port looks like `/dev/tty.usbserial-xxxx`.

WSL isn't good for this part. Plain Windows with PuTTY is simpler.

## Step 3: Check the firmware version

1. Plug in the 19V power supply. It turns on by itself. No microSD card needed for this.
2. Click the serial window and keep pressing Esc. This opens the UEFI setup menu in the console.
3. The firmware version is on a line near the top.
   - 36.x or newer: skip Step 4, go to Step 5.
   - Older than 36 (something like 3.x, 4.x or 5.x): do Step 4.

If nothing shows up, check the TX/RX wires aren't the wrong way round and the speed is 115200.

## Step 4: Old firmware, update it with JetPack 5.1.3 (only if needed)

JetPack 5.1.3 is just a bridge, it wakes up the firmware update feature on old boards. It gets used once and then it's gone.

1. Download the JetPack 5.1.3 SD card image for the Orin Nano from the link above. Has to be the updated one, the file is called `JP513-orin-nano-sd-card-image_b29.zip`.
2. Flash it to the microSD card with BalenaEtcher (Etcher takes the zip as is, no need to unzip). Flash from file, select the zip, select the microSD card, Flash.
3. Power off the Jetson, put the microSD card in the slot on the underside of the module, and plug in Ethernet.
4. Power on. The first-boot setup should show up as text in the serial window. NVIDIA's page doesn't show what the headless version looks like, so it isn't confirmed. If nothing appears after a few minutes, a monitor is needed for that one step.
5. Go through it: EULA, language, keyboard, time zone, username and password.
6. Make sure the board is online. With Ethernet plugged in it should be. If using Wi-Fi:

   ```bash
   nmcli device wifi list
   sudo nmcli device wifi connect "WIFI_NAME" password "WIFI_PASSWORD"
   ```

7. After it boots, a background service schedules the firmware update on its own. Check it from the serial window:

   ```bash
   sudo systemctl status nv-l4t-bootloader-config
   ```

   Done looks like inactive with a successful exit.
8. Reboot:

   ```bash
   sudo reboot
   ```

   The firmware update runs on the way up and progress shows in the serial window. It prints something like "Update Progress - 10%". Don't unplug anything. It boots back into JetPack 5.1.3 afterwards.
9. Install the QSPI updater:

   ```bash
   sudo nvbootctrl dump-slots-info      # just shows the current firmware version
   sudo apt update
   sudo apt install nvidia-l4t-jetson-orin-nano-qspi-updater
   ```

10. Reboot again. The QSPI update runs, wait for it to finish.
11. Power off and take the JetPack 5.1.3 card out. The firmware is ready for JetPack 6 now, that card is done.

Don't pull the power while any firmware update is running.

## Step 5: Flash JetPack 6.2 to the microSD card

1. Download the JetPack 6.2.1 SD card image for the Orin Nano from the link above (6.2.x all work the same way).
2. Flash it to the microSD card with BalenaEtcher. If the same card was used for 5.1.3, it just gets overwritten.
3. Put the card in the slot on the underside of the module.

## Step 6: First boot (over serial)

1. Serial window open, Ethernet plugged in, then power on.
2. The setup should show up as text in the serial window (same caveat as before, NVIDIA's page doesn't show the headless version). Go through it: EULA, language, keyboard, time zone, network, username, password, computer name.
3. Write down the username, password and computer name.
4. Log in at the serial window.
5. If Wi-Fi isn't connected yet:

   ```bash
   nmcli device wifi list
   sudo nmcli device wifi connect "WIFI_NAME" password "WIFI_PASSWORD"
   ```

6. Check if JetPack 6.2 scheduled another firmware update:

   ```bash
   sudo systemctl status nv-l4t-bootloader-config
   ```

   If it did, `sudo reboot` once more and let the update run. Boots back into JetPack 6.2 after.

Once the IP is known (Part 2), the serial cable isn't needed anymore.

## Step 7: MAXN SUPER

The default power mode is usually 25W. MAXN SUPER lets the board use full power for max CPU and GPU speed. No desktop menu without a monitor, so it's the terminal:

```bash
sudo nvpmodel -q          # current mode
sudo nvpmodel -m 2        # MAXN SUPER
sudo jetson_clocks        # optional, locks clocks at max
```

These commands aren't on NVIDIA's page (it only shows the desktop menu way), so check the mode number on the board with `sudo nvpmodel -q --verbose`.

## Step 8: Update and check the JetPack parts

The JetPack 6.2 SD card image already has CUDA, cuDNN and TensorRT in it. Just update:

```bash
sudo apt update
sudo apt upgrade -y
```

If something turns out to be missing later:

```bash
sudo apt install -y nvidia-jetpack
```

---

# Part 2: Connect from the laptop (SSH)

Once the board is on Wi-Fi, a screen or cable isn't needed anymore. Everything happens from the laptop.

## Find the IP

On the Jetson:

```bash
hostname -I
```

Gives something like `192.168.1.45`. Note it down.

## SSH in

Laptop has to be on the same Wi-Fi as the Jetson. In a terminal (PowerShell on Windows):

```bash
ssh username@192.168.1.45
```

`username` is whatever got created during first boot. Computer name works too: `ssh username@computername.local`. Type the password and that's it.

College and hostel Wi-Fi usually blocks devices from talking to each other, so SSH can fail even when both are online. Fix is the phone hotspot, with the laptop and the Jetson both connected to it. Ethernet cable from the Jetson to a router also works.

## Copy the project over

From the laptop:

```bash
scp -r my_project username@192.168.1.45:~/
```

Or push to GitHub and `git clone` on the Jetson. VS Code with the "Remote - SSH" extension is nice too, edits files on the Jetson straight from the laptop.

---

# Part 3: Docker

JetPack 6.x SD card images usually come with Docker and the NVIDIA runtime already installed. Check before installing anything.

## Is Docker already there

```bash
docker --version
docker info | grep -i runtime
```

If both work and the runtimes line shows `nvidia`, skip to "Make nvidia the default runtime". If not:

```bash
sudo apt update
sudo apt install -y nvidia-container curl
curl https://get.docker.com | sh
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl daemon-reload
sudo systemctl restart docker
```

NVIDIA's Docker page: https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/setup_docker.html

To not type sudo every time:

```bash
sudo usermod -aG docker $USER
newgrp docker
```

## Make nvidia the default runtime

This one matters because of the Dockerfile. Without it, Docker only uses the GPU when `--runtime nvidia` is passed to `docker run`. `docker build` runs CPU only, so any `RUN` line that needs CUDA, TensorRT or a GPU PyTorch check fails with "no GPU found". With it set, every build step and every run uses the NVIDIA runtime.

1. Open the config:

   ```bash
   sudo nano /etc/docker/daemon.json
   ```

2. It should contain this. If the file already has other settings, keep them and just add these lines:

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

## Base image for the Dockerfile

JetPack 6.2 is Jetson Linux r36.4.x with CUDA 12.6, so the base image has to be an r36 one. Images made for JetPack 7 (r39, CUDA 13) won't work here, and old r35 ones (JetPack 5) won't either.

```dockerfile
FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0
```

- r36.4.0 is the one NVIDIA publishes for the whole 36.4 family (JetPack 6.2 and 6.2.1). Newer r36.4.x tags may exist, check NGC.
- arm64 only, build on the Jetson itself.
- Images get big. They live on the microSD, so keep an eye on space (`df -h`).

## Quick test

```bash
docker run --rm -it --runtime nvidia nvcr.io/nvidia/l4t-jetpack:r36.4.0 nvcc --version
```

Should print the CUDA version (12.6). That only shows CUDA is there. Real proof the GPU is being used comes when the YOLO code runs. On the Jetson (not in the container) run:

```bash
sudo tegrastats
```

and watch `GR3D_FREQ`. It's the GPU load, it should jump while YOLO runs.

## Build and run

Build from the folder that has the Dockerfile:

```bash
docker build -t <your-image-name> .
```

Run with the GPU (`--runtime nvidia` or `--gpus all`, or set it in docker-compose.yml):

```bash
docker run --runtime nvidia --network host -it <your-image-name>
```

## YOLO speed

A `.pt` model is slow on the Jetson. Convert it to TensorRT on the Jetson itself and load the `.engine` file in the ROS code:

```bash
yolo export model=best.pt format=engine half=True
```

---

# Part 4: Arducam IMX519 camera

This is the 16MP camera from the parts list. It isn't plug-and-play on Jetson (unlike the IMX219), so it needs Arducam's driver. Arducam's page: https://docs.arducam.com/Nvidia-Jetson-Camera/Native-Camera/Quick-Start-Guide/

Things to know first:
- The driver only works on the official NVIDIA dev kit carrier board. Third-party boards aren't guaranteed.
- The JetPack version has to be on Arducam's supported list. For the Orin Nano that's JetPack 6.2 (L4T 36.4.3, 36.4.4, 36.4.7, 36.5.0). JetPack 7 isn't on it, which is why these notes use 6.2. Check the version with `cat /etc/nv_tegra_release`.
- The driver gets built for the exact kernel that's running. So do the `apt upgrade` first, install the camera driver last, and don't upgrade again afterwards without testing the camera.

## Plug in the camera

1. Power the Jetson off first.
2. Cable: the dev kit's camera port is 22-pin. The IMX519 module is normally 15-pin, so it needs a 15-pin to 22-pin cable (a 22-22 one only if the module itself is 22-pin). Arducam's page has pictures of both.
3. The camera ports are on the board edge opposite the GPIO header.
4. Gently pull up the plastic edges of the port.
5. Push the ribbon in. The silver contacts face the heatsink side. It has to go all the way in, and the cable shouldn't be bent.
6. Push the plastic part back down while holding the cable.
7. Note which port it's in. CAM0 is `sensor_id=0`, CAM1 is `sensor_id=1`.

## Install the driver

On the Jetson (over SSH is fine):

```bash
cd ~
wget https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.3/install_full.sh
chmod +x install_full.sh
./install_full.sh -m imx519
```

Then reboot:

```bash
sudo reboot
```

If it complains about permissions, put `sudo` in front of the install line.

## Check that it shows up

```bash
v4l2-ctl --list-devices
ls /dev/video*
dmesg | grep -i imx519
```

There should be a video device, and the dmesg line should say the imx519 driver loaded. Nothing there means the camera isn't detected, see the problems list below.

## Test it

Preview needs a monitor, so with no screen it's easier to save a picture or a short video and copy it to the laptop.

One picture (this command isn't from Arducam's page, but it's standard Jetson):

```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=1 ! "video/x-raw(memory:NVMM),width=1920,height=1080" ! nvjpegenc ! filesink location=test.jpg
```

Short video (from Arducam's page, Ctrl+C to stop):

```bash
SENSOR_ID=0
FRAMERATE=30
gst-launch-1.0 -e nvarguscamerasrc sensor-id=$SENSOR_ID ! "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=$FRAMERATE/1" ! nvv4l2h264enc ! h264parse ! mp4mux ! filesink location=cam$SENSOR_ID.mp4
```

Copy it to the laptop, run this on the laptop:

```bash
scp username@192.168.1.45:~/test.jpg .
```

To see the formats the camera offers: `v4l2-ctl --list-formats-ext`.

Live preview with a monitor on the Jetson (with SSH, `export DISPLAY=:0` first):

```bash
gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1,format=NV12" ! nvvidconv flip-method=0 ! "video/x-raw,width=960,height=720" ! nvvidconv ! nvegltransform ! nveglglessink -e
```

## Using the camera inside Docker

The container needs the NVIDIA runtime plus the Argus socket and the video device. This isn't from Arducam's page, so test it:

```bash
docker run --runtime nvidia --network host -it \
  -v /tmp/argus_socket:/tmp/argus_socket \
  --device /dev/video0 \
  <your-image-name>
```

If the camera still doesn't open in the container, `--privileged` is the quick (but blunt) fix.

## If something goes wrong

- **Not detected at all:** power off and reseat the cable (all the way in, contacts facing the heatsink side). Try the other camera port. Check the JetPack version is on the list above. Check `uname -r` and compare with what the installer targeted.
- **Driver loads but no frames:** this has been reported on JetPack 6.2 forums. Try the other port and a different cable, reboot, and check `dmesg | grep -i imx519` for errors.
- **Wrong cable:** the Orin Nano port is 22-pin, a plain 15-15 cable won't fit it.
- **Don't force device tree overlays by hand.** One forum user did and the board stopped booting. Let Arducam's script do it.
- **Still stuck:** Arducam's support is at forum.arducam.com.
- **Autofocus:** Arducam has an example called `Jetson_IMX519_Focus_Example`, linked from the quick start page.

---

# Part 5: Later, on the drone

- **Power:** the battery is 6S (about 22 to 25V), too high for the Jetson. Use a BEC set to 12V, 5A or more, into the barrel jack. The dev kit takes roughly 9 to 19V, check the carrier board spec. Measure the BEC output with a multimeter before plugging in, a lot of BECs are 5V by default. Check the barrel jack polarity too (center should be positive). The Pixhawk power module output is 5V, too weak for this. USB-C doesn't power the board.
- **Flight controller:** Jetson to CubePilot over serial (UART), usually through MAVROS. Jetson 40-pin header UART pins to the CubePilot TELEM port, with ground connected. Exact pins to be filled in once the TELEM port is picked.
- **Start on boot:** nobody can type in flight, so run the container detached with a restart policy:

  ```bash
  docker run -d --restart unless-stopped --runtime nvidia --network host <your-image-name>
  ```

- **No Wi-Fi at the venue:** phone hotspot. Join it from the Jetson ahead of time (`nmcli`) so it connects by itself on boot. Test this at home first.
