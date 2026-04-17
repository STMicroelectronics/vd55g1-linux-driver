# ST-VD55G1 Linux driver

## Armbian (Rockchip) specific setup

On Armbian with Rockchip SoCs (e.g. Rock 3A / RK3568), the CSI2 DPHY and CIF
pipeline drivers must be built as kernel modules (`=m`) so that module load
ordering can be controlled. The sensor driver must probe before the DPHY driver
so the v4l2 async framework can match the sensor subdev.

### Image rootfs files

The following files must be present in the Armbian image rootfs:

- `/etc/modprobe.d/rkcif-probe-order.conf` — softdep rules to enforce module
  load order (sensor and dphy-hw before dphy, sensor before rkcif):

```
softdep phy_rockchip_csi2_dphy pre: phy_rockchip_csi2_dphy_hw vd55g1
softdep video_rkcif pre: vd55g1
softdep rkcif_mipi_lvds pre: vd55g1
```

- `/etc/systemd/system/csi2-dphy0-rebind.service` — rebind the DPHY driver
  after boot to work around a probe race between `csi2-dphy0` and its HW
  backend (the driver returns `-EINVAL` instead of deferring):

```ini
[Unit]
Description=Rebind csi2-dphy0 after dphy-hw is ready
After=systemd-modules-load.service
DefaultDependencies=no

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'echo csi2-dphy0 > /sys/bus/platform/drivers/rockchip-csi2-dphy/bind'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Enable it with `sudo systemctl enable csi2-dphy0-rebind.service`.

### Driver and device tree overlay installation

Add the following line to `/boot/armbianEnv.txt`:
```
user_overlays=pcb4511_vd65g4
```

Then install the driver and compile the device tree overlay:
```
sudo mkdir -p /lib/modules/$(uname -r)/extra
sudo cp vd55g1.ko /lib/modules/$(uname -r)/extra/
sudo dtc <device-tree>.dts -o /boot/overlay-user/<device-tree>.dtbo
sudo depmod -a
sudo reboot
```

After reboot, verify the sensor appears in the media pipeline:

```
media-ctl -p
```

The sensor entity (e.g. `vd55g1 5-0010`) should be linked to
`rockchip-csi2-dphy0`.
