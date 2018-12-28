# Programming Intel/Altera EP4CGX15 with USB Blaster and UrJTAG

Build actual [UrJTAG](http://urjtag.org/) version

```
git clone git://git.code.sf.net/p/urjtag/git
cd urjtag/urjtag
./autogen.sh
./configure \
  --prefix=/opt/urjtag \
  --enable-maintainer-mode \
  --enable-jedec-exp \
  --enable-bus \
  --enable-cable \
  --enable-lowlevel \
  --enable-stapl
sudo make install
sudo ln -s /opt/urjtag/bin/jtag /usr/local/bin/urjtag
```
Copy files [ep4cgx15/STEPPINGS](ep4cgx15/STEPPINGS) and [ep4cgx15/ep4cgx15](ep4cgx15/ep4cgx15) to `/opt/urjtag/share/urjtag/altera/ep4cgx15` and add line
```
0010100000000001        ep4cgx15                EP4CGX15
```
to the file `/opt/urjtag/share/urjtag/altera/PARTS`.

Check that USB Blaster is accessible from regular user account.

If not create file UDEV rules file `/etc/udev/usbblaster.rules` as root
```
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="09fb", ATTR{idProduct}=="6001", MODE:="0660", GROUP:="plugdev"
```
and make link to it.
```
cd /etc/udev/rules.d
ln -s ../usbblaster.rules 89-usbblaster.rules
udevadm control --reload-rules
```
Export design FPGA configuration file to SVF from File menu of Quartus programmer.

Prepare UrJTAG command file
```
#!/usr/bin/env urjtag

cable usbblaster
detect
part 0
svf fpga_configuration.svf
```
Change file mode to executable and run it.
