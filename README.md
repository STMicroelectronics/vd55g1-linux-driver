# ST-VD55G1 Linux driver

## Installation

### Driver

Compile the driver using the Makefile

```
make
```

Place it in the kernel modules folder.

```
sudo cp st-vd55g1.ko /lib/modules/$(uname -r)
```

Resolve modules dependencies.

```
sudo depmod -a
```

### Device tree

 Compile the device tree overlay matching your platform and plugin board from the `dts` folder.

```
sudo dtc <device-tree>.dts -o /boot/overlays/<device-tree>.dtbo
```

Set the device tree overlay in your platform. This may differ from platform to platform. Please refer to your platform documentation.

This is how to set the device tree overlay for Raspberry Pi.

```
echo "dtoverlay=<device-tree>" | sudo tee -a /boot/config.txt
```

Finally, run `sudo reboot` to test your changes.


## Usage

Run any video capture application to stream from the sensor.
QV4L2 testbench is a well featured test application.

```
sudo apt install qv4l2
qv4l2
```
