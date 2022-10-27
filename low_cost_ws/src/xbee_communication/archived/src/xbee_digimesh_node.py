#!/usr/bin/env python3

from digi.xbee.devices import DigiMeshDevice
from digi.xbee.exception import *
from digi.xbee.models.address import *

class XBee_Operation(object):
    def __init__(self):
        self.PORT = "/dev/ttyUSB0"
        self.BAUD_RATE = 115600
        self.device = DigiMeshDevice(self.PORT, self.BAUD_RATE)

        self.device.open()
        self.device.add_data_received_callback(self.receive)
        self.sourceAddr = str(self.device.get_64bit_addr())[-4:]
        print("xbee initialized")

    def send(self, dest, payload):
        try:
            self.device.send_data_64(XBee64BitAddress.from_hex_string(dest), payload)
        except (TimeoutException, TransmitException) as e:
            print(e)
            return
        print("success")
        

    def receive(self, xbee_message):
        address = xbee_message.remote_device.get_64bit_addr()
        data = xbee_message.data.decode("utf8")
        print("Received data from %s: %s" % (address, data))

    def on_shutdown(self):
        if self.device is not None and self.device.is_open():
            self.device.close()

if __name__ == "__main__":
    xbee_node = XBee_Operation()

    try:
        while(1):
            a = input("Enter the dest addr: ") 
            d = input("Enter the data: ")
            xbee_node.send(a, d)
    finally:
        xbee_node.on_shutdown()
