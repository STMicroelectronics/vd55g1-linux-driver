# ST-VD55G1 Linux driver

## Supported Devices

This driver supports the following devices :

### 1st generation

- VD55G1 : Monochrome sensor

### 2nd generation

- VD55G4 : Monochrome sensor
- VD65G4 : Color sensor

## Installation

### Driver

Compile the driver using the Makefile

```
make
```

Place it in the kernel modules folder.

```
sudo cp vd55g1.ko /lib/modules/$(uname -r)
```

Resolve modules dependencies.

```
sudo depmod -a
```

### Device tree

Compile the device tree overlay matching your platform and plugin board from the `dts` folder.

```
sudo dtc <device-tree>.dts -o /boot/firmware/overlays/<device-tree>.dtbo
```

Set the device tree overlay in your platform. This may differ from platform to platform. Please refer to your platform documentation.

This is how to set the device tree overlay for Raspberry Pi.

```
echo "dtoverlay=<device-tree>" | sudo tee -a /boot/firmware/config.txt
```

Finally, run `sudo reboot` to test your changes.

### Run Auto Wake Up mode

The VD55G1 can autonomously detect a change in the scene and trigger a decision to the host. This special mode is called Auto Wake Up (awu).

You can enable this mode via v4l2-ctl.
```
v4l2-ctl -d /dev/v4l-subdev2 -c auto_wake_up=1
```

During the detection phase of the mode, no frame is sent by the sensor. To avoid libcamera repeatedly throwing timeout errors, you need to use the file awu_avoid_timeout.yaml
```
LIBCAMERA_RPI_CONFIG_FILE=awu_avoid_timeout.yaml rpicam-hello --timeout 0
```
