# panther_lights_controller_rpi_apa102

## Host machine prerequisites

### `Raspberry pi GPIO access`
Check if `gpio` group exist:
```
$ groups
```
If there's no `gpio` group, create it:
```bash
$ sudo groupadd gpio
```
and add current user to this group:
```bash
$ sudo adduser $USER gpio
```
Check if `/dev/gpiomem` exists and has correct permissions:
```bash
$ ls -l /dev/gpiomem
crw-rw---- 1 root gpio 244, 0 Dec 28 22:51 /dev/gpiomem
```
If it doesn’t then set the correct permissions as follows:
```bash
sudo chown root.gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem
```
### `SPI access`

Check if spi is enabled
```
ls -l /dev | grep 'spi'
crw-------  1 root     root    153,   0 Apr  1 17:23 spidev0.0
crw-------  1 root     root    153,   1 Apr  1 17:23 spidev0.1
```
If there is no `spidev` files, add this line to `/boot/usercfg.txt`:
```plain
dtparam=spi=on
```
and restart the system.

Check if spi group exist:
```bash
$ groups
```
If there's no spi group, create it:
```bash
sudo groupadd spi
``` 
and add current user to it:
```bash
sudo adduser $USER spi
```
change `spidev0` ownership:
```bash
sudo chown root.spi /dev/spidev*
sudo chmod g+rw /dev/spidev*
```

### udev rules
In `/etc/udev/rules.d` create file `local.rules`:
```bash
$ sudo touch /etc/udev/rules.d/local.rules
```
Using `tee` add following lines:
```
$ sudo tee -a /etc/udev/rules.d/local.rules 
ACTION=="add", KERNEL=="spidev0.0", GROUP="spi", MODE="0660"
ACTION=="add", KERNEL=="gpiomem", GROUP="gpio", MODE="0660"
```
`Ctrl-D` to save
