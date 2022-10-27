from digi.xbee.devices import XBeeDevice
from digi.xbee.util import utils

# TODO: Replace with the serial port where your local module is connected to.
PORT = "/dev/ttyUSB0"
# TODO: Replace with the baud rate of your local module.
BAUD_RATE = 115200




def main():

    device = XBeeDevice(PORT, BAUD_RATE)

    try:
        device.open(force_settings=True)

        # Read the operating PAN ID of the device.
        pan_id = device.get_pan_id()
        #
        # Read the output power level.
        p_level = device.get_power_level()

        address64 = device.get_64bit_addr()
        address16 = device.get_16bit_addr()
        Node_identifier = device.get_node_id()
        Firmware_version = device.get_firmware_version()
        Hardware_version = device.get_hardware_version()
        Role = device.get_role()

        print('address64 = ',address64)
        print('address16 = ',address16)
        print('Node_identifier = ',Node_identifier)
        print('Firmware_version = ',Firmware_version)
        print('Hardware_version = ',Hardware_version)
        print('Role = ',Role)
        print('pan_id = ',pan_id)
        print('p_level = ',p_level)


    finally:
        if device is not None and device.is_open():
            device.close()


if __name__ == '__main__':
    main()
