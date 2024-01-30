# ST-VD55G1 Linux driver

## Installation

### Driver

Compile the driver using the Makefile, place it in the kernel modules folder, and resolve modules dependencies.

```
make
sudo cp st-vd55g1.ko /lib/modules/$(uname -r)
sudo depmod -a
```

### Device tree

Compile the device tree overlay matching your platform and plugin board from the `dts` folder.

```
sudo dtc <device-tree>.dts -o /boot/overlays/<device-tree>.dtbo
```

Set the device tree overlay in your platform.
Here is an example for Raspberry Pi and may differ from platforms to
platforms. Please refer to your platform documentation.

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
