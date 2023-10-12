from typing import List

import pypozyx
import serial
import yaml
from pypozyx import (
    Coordinates,
    DeviceCoordinates,
    DeviceRange,
    NetworkID,
    PozyxConstants,
    PozyxSerial,
)
from pypozyx.core import PozyxException
from pypozyx.definitions.constants import POZYX_SUCCESS
from pypozyx.structures.generic import SingleRegister
from serial.tools.list_ports import comports


# %% ../06_uwb.ipynb 5
class UWB():
    def __init__(self, port = None):
        self.port = port
        self.network_id = None
        self.pose = None
        self._pozyx_handler = None        
        self._env_config = None
        self.ranges_all = []
        self.anchor_ids = []
        
        self._height = 500

        self._status = 0
        
    @property
    def network_id(self):
        return self._network_id.id
    
    @property
    def network_id_str(self) -> str:
        """A getter method of network id string

        Convert network id to string to show in readable.

        Returns:
            str: A string of id number in hexadecimal of The Pozyx
        """
        return str(self._network_id)

    @network_id.setter
    def network_id(self, value: int = None) -> None:
        """A setter method of port string

        Args:
            value (int, optional): A integer id number in hexadecimal or decimal of The Pozyx. Defaults to None.
        """
        if value is None:
            self._network_id = NetworkID()
        else:
            self._network_id = NetworkID(value)

    @property
    def pose(self) -> List[float]:
        """A getter method of UWB pose

        Returns:
            list[float]: (pose.x, pose.y, pose.z)
        """
        return (self._pose.x, self._pose.y, self._pose.z)

    @pose.setter
    def pose(self, value: List[float] = None) -> None:
        """A setter method of UWB pose

        Args:
            value (List[float], optional): (pose.x, pose.y, pose.z) Defaults to None.
        """
        if value is None:
            self._pose = Coordinates()
        else:
            self._pose.x = value[0]
            self._pose.y = value[1]
            self._pose.z = value[2]

    @property
    def height(self) -> float:
        """A getter method of UWB pose height

        Returns:
            float: The default height for 2.5D localization.
        """
        return self._height

    @height.setter
    def height(self, value: float = 0) -> float:
        """A setter method of UWB pose height

        Args:
            value (int, optional): The default height for 2.5D localization.. Defaults to 0.
        """
        self._height = value


    @property
    def env_config(self) -> dict:
        """A getter method of environment config

        Returns:
            dict: The environment config in dict format.
        """
        return self._env_config


    def port_list(self) -> List[str]:
        """A getter method of port list.

        Returns:
            List[str]: The list contains UWB port device path like `/dev/ttyACM0`.
        """
        return self._port_list


    @property
    def status(self) -> int:
        """A getter method of UWB status.

        Returns:
            int: The status got from Pozyx. 0 is success.
        """
        return self._status

    def load_env_config(self, config_file_path: str) -> bool:
        """Load UWB anchors' environment config.

        Args:
            config_file_path (str): The environment config file path.

        Returns:
            bool: True for success, False for failure.
        """
        with open(config_file_path, "r") as config_file:
            try:
                self._env_config = yaml.safe_load(config_file)
                self.ranges_all = [DeviceRange() for _ in range(len(self._env_config))]
                self.anchor_ids = [config[1]['id'] for config in self._env_config.items()]
                print(hex(self.network_id), [hex(anchor_id) for anchor_id in self.anchor_ids])
            except yaml.YAMLError as ex:
                # print(ex)
                return False
        return True

    def scan_port(self, network_id=None) -> None:
        """Scan all port connecting to host. Store port device path in port list.
        """
        if network_id is None:
            network_id = self.network_id
            
        port_list = []
        for port in comports():
            try:
                if "Pozyx Labs" in port.manufacturer:
                    port_list.append(port.device)
                    continue
            except TypeError:
                pass
            try:
                if "Pozyx" in port.product:
                    port_list.append(port.device)
                    continue
            except TypeError:
                pass
        print(f'port_list: {port_list}')
        if len(port_list) == 1:
            return port_list[0]
        for port in port_list:
            try:
                pozyx_handler = PozyxSerial(port)
                network_id_ = NetworkID()
                pozyx_handler.getNetworkId(network_id_)
                print(f'read net: {network_id_}')
                print(f'network_id: {hex(network_id)}')
                print(f'self network_id: {hex(self.network_id)}')
                if network_id == network_id_:
                    return port
            except Exception as e:
                # print(e)
                pass

    def connect(self, port=None, network_id=None) -> bool:
        """Try to connect pozyx device.

        Returns:
            bool: Pozyx status
        """
        self._status = PozyxConstants.STATUS_SUCCESS
        
        if network_id is not None:
            self.network_id = network_id
        
        if port is not None and network_id is None:
            self.port = port
        else:
            self.port = self.scan_port(network_id)
            
        print(f'port: {self.port}')
        
        try:
            self._pozyx_handler = PozyxSerial(self.port)
            self._status &= self._pozyx_handler.getNetworkId(self._network_id)
            return True
        except Exception as e:
            # print(e)
            return False


    def validate(self, clear_port=False) -> bool:
        whoami = SingleRegister()
        try:
            ret_validate = self._pozyx_handler.getWhoAmI(whoami) == POZYX_SUCCESS
        except:
            return False
        
        if clear_port and not ret_validate:
            self.port = None
        
        return ret_validate

    def write_env_config(self) -> bool:
        """Write environment anchor location into Pozyx UWB device.

        Returns:
            bool: Pozyx status
        """
        self._status = PozyxConstants.STATUS_SUCCESS
        ANCHOR_FLAG = 1
        self._status &= self._pozyx_handler.clearDevices()
        for anchor_name, config in self.env_config.items():
            coordinate = Coordinates(config["x"], config["y"], config["z"])
            device_coordinate = DeviceCoordinates(config["id"], ANCHOR_FLAG, coordinate)
            self._status &= self._pozyx_handler.addDevice(device_coordinate)
        if len(self.env_config) > 4:
            self._status &= self._pozyx_handler.setSelectionOfAnchorsAutomatic(len(self.env_config))
        return self._status

    def localize_2_5D(self) -> bool:
        """Localize method in 2.5D. Need to know height.

        Returns:
            bool: Pozyx status
        """
        self._status &= self._pozyx_handler.doPositioning(
            self._pose,
            PozyxConstants.DIMENSION_2_5D,
            self._height,
            PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY,
        )
        return self._status

    def localize_3D(self) -> bool:
        """Localize method in 3D. The height will be determined by Pozyx UWB device.

        Returns:
            bool: Pozyx status
        """
        self._status &= self._pozyx_handler.doPositioning(
            self._pose,
            PozyxConstants.DIMENSION_3D,
            self._height,
            PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY,
        )
        return self._status

    def range_from(self, dest_id) -> float:
        """Range method from this Pozyx UWB device to the destination Pozyx UWB device.

        Args:
            dest_id (_type_): The target Pozyx UWB device id want to be ranged.

        Returns:
            float: The range from this Pozyx UWB device to the destination Pozyx UWB device.
        """
        ranges = DeviceRange()
        self._status &= self._pozyx_handler.doRanging(dest_id, ranges)
        return ranges

    def range_all(self) -> bool:
        """Range method from this Pozyx UWB device to the destination Pozyx UWB device.

        Args:
            dest_id (_type_): The target Pozyx UWB device id want to be ranged.

        Returns:
            float: The range from this Pozyx UWB device to the destination Pozyx UWB device.
        """
        try:
            for i, anchor_id in enumerate(self.anchor_ids):
                self._status &= self._pozyx_handler.doRanging(anchor_id, self.ranges_all[i])
            return True
        except:
            return False
