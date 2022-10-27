sudo bash -c 'echo "SUBSYSTEMS==\"usb\", ATTRS{product}==\"FT231X USB UART\", MODE=\"0777\", SYMLINK+=\"xbee\"" > /etc/udev/rules.d/50-subt_superduckiebot.rules'
sudo udevadm trigger
