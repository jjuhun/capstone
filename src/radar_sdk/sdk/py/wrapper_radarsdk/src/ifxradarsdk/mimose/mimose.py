# ===========================================================================
# Copyright (C) 2021-2022 Infineon Technologies AG
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# ===========================================================================

"""Python wrapper for Infineon Mimose ATR22 Radar sensor"""

from ctypes import *
import typing

import numpy as np

from ..common.base_types import MdaComplex
from ..common.cdll_helper import load_library, declare_prototype
from ..common.sdk_base import ifx_mda_destroy_c
from .types import ifx_Mimose_Config_t, MimoseMetadata, ifx_Mimose_Config_Limits_t, ifx_Mimose_RF_Band_t, ifx_Mimose_Frame_Config_t



class DeviceMimose():
    @staticmethod
    def __load_cdll() -> CDLL:
        """Initialize the module and return ctypes handle"""
        # find and load shared library
        dll = load_library("sdk_mimose")

        # declare prototypes such that ctypes knows the arguments and return types
        declare_prototype(dll, "ifx_mimose_create", None, c_void_p)
        declare_prototype(dll, "ifx_mimose_destroy", [c_void_p], None)
        declare_prototype(dll, "ifx_mimose_create_by_uuid", [c_void_p], c_void_p)
        declare_prototype(dll, "ifx_mimose_create_dummy", None, c_void_p)
        declare_prototype(dll, "ifx_mimose_get_config_defaults", [c_void_p, POINTER(ifx_Mimose_Config_t)], None)
        declare_prototype(dll, "ifx_mimose_get_config", [c_void_p, POINTER(ifx_Mimose_Config_t)], None)
        declare_prototype(dll, "ifx_mimose_set_config", [c_void_p, POINTER(ifx_Mimose_Config_t)], None)
        declare_prototype(dll, "ifx_mimose_start_acquisition", [c_void_p], None)
        declare_prototype(dll, "ifx_mimose_stop_acquisition", [c_void_p], None)
        declare_prototype(dll, "ifx_mimose_update_rc_lut", [c_void_p], None)
        declare_prototype(dll, "ifx_mimose_get_register_value", [c_void_p, c_short], c_short)
        declare_prototype(dll, "ifx_mimose_set_registers", [c_void_p, POINTER(c_uint32), c_size_t], None)
        declare_prototype(dll, "ifx_mimose_get_next_frame", [c_void_p, POINTER(MdaComplex), POINTER(MimoseMetadata)], POINTER(MdaComplex))
        declare_prototype(dll, "ifx_mimose_get_next_frame_timeout", [c_void_p, POINTER(MdaComplex), POINTER(MimoseMetadata), c_uint16], POINTER(MdaComplex))
        declare_prototype(dll, "ifx_mimose_check_config", [c_void_p, POINTER(ifx_Mimose_Config_t), c_uint16], c_bool)
        declare_prototype(dll, "ifx_mimose_get_number_of_samples_limits", [c_void_p, c_bool * 4, POINTER(c_uint16), POINTER(c_uint16)], None)
        declare_prototype(dll, "ifx_mimose_get_pulse_repetition_time_limits_s", [c_void_p, c_bool * 4, POINTER(c_float), POINTER(c_float)], None)
        declare_prototype(dll, "ifx_mimose_get_frame_repetition_time_limits_s", [c_void_p, c_bool * 4, c_uint16, c_uint16, c_float, POINTER(c_float), POINTER(c_float)], None)
        declare_prototype(dll, "ifx_mimose_get_rf_center_frequency_limits_Hz", [c_void_p, c_int, POINTER(c_uint64), POINTER(c_uint64)], None)
        declare_prototype(dll, "ifx_mimose_get_default_limits", [c_void_p, POINTER(ifx_Mimose_Config_Limits_t)], None)

        return dll

    _cdll = __load_cdll.__func__()

#    @classmethod
#    def get_list(cls) -> typing.List[str]:
#        """Return list of UUIDs of available boards
#
#        The function returns a list of unique ids (uuids) that correspond to
#        available boards.
#
#        Note: boards which are already instantiated will not appear in the list.
#
#        **Examples**
#            uuids_all   = Device.get_list()
#        """
#
#        ifx_list = cls._cdll.ifx_mimose_get_list()
#
#        return move_ifx_list_to_python_list(ifx_list,
#                                            lambda p: cast(p, POINTER(DeviceListEntry)).contents.uuid.decode("ascii"))
#
    def __init__(self, uuid: typing.Optional[str] = None, dummy: typing.Optional[bool] = False):
        """Create and initialize Atr22 device controller

        Search for an Infineon radar sensor board connected to the host machine
        and connect to the first found Mimose sensor device.

        The device is automatically closed by the destructor. If you want to
        close the device yourself, you can use the keyword del:
            device = DeviceMimose()
            # do something with device
            ...
            # close device
            del device

        However, we suggest to use a context manager:
            with open DeviceMimose() as device:
                # do something with device

        If uuid is given the radar device with the given uuid is opened. If
        no parameters are given, the first found radar device will be opened.

        Examples:
          - Open first found radar device:
            device = DeviceMimose()
          - Open Mimose radar device with uuid 0123456789abcdef0123456789abcdef
            device = DeviceMimose(uuid="0123456789abcdef0123456789abcdef")
          - Open Mimose Radar dummy device:
            device = DeviceMimose(dummy=True)

        Optional parameters:
            uuid:       open the ltr11 doppler radar device with unique id given by uuid.
                        The uuid is represented as a 32 character string of hexadecimal
                        characters. In addition, the uuid may contain dash characters (-),
                        which will be ignored.
                        Both examples are valid and correspond to the same
                        uuid:
                            0123456789abcdef0123456789abcdef
                            01234567-89ab-cdef-0123-456789abcdef
        """

        if uuid:
            h = self._cdll.ifx_mimose_create_by_uuid(uuid.encode("ascii"))
        elif dummy:  # RadarSensor.BGT24ATR22 = 257
            h = self._cdll.ifx_mimose_create_dummy()
        else:
            h = self._cdll.ifx_mimose_create()

        # The pointer is returned as an integer. Explicitly casting it to a void pointer
        # ensures it is not truncated by integer handling in certain situations
        self.handle = c_void_p(h)
        self.config = self.get_config_defaults()
        #self.registers = MimoseRegisters()
        self.limits = self.get_default_limits()

#    def get_firmware_information(self) -> dict:
#        """Gets information about the firmware of a connected device"""
#        info_p = self._cdll.ifx_ltr11_get_firmware_information(self.handle)
#        return info_p.contents.to_dict(True)
#
#    def get_sensor_information(self) -> dict:
#        """Gets information about the connected device"""
#        info_p = self._cdll.ifx_ltr11_get_sensor_information(self.handle)
#        info = info_p.contents.to_dict(True)
#
#        for entry in ["lp_cutoff_list", "hp_cutoff_list", "if_gain_list"]:
#            info[entry] = create_python_list_from_terminated_list(info[entry])
#
#        return info

    def set_config(self, config: typing.Optional[ifx_Mimose_Config_t] = None) -> None:
        """Set Mimose configuration"""
        if config:
            self._cdll.ifx_mimose_set_config(self.handle, byref(config))
        else:
            self._cdll.ifx_mimose_set_config(self.handle, byref(self.config))

    def get_config(self) -> ifx_Mimose_Config_t:
        """Get current Mimose configuration"""
        config = ifx_Mimose_Config_t()
        self._cdll.ifx_mimose_get_config(self.handle, byref(config))
        return config

    def get_config_defaults(self) -> ifx_Mimose_Config_t:
        """Get default device configuration"""
        config = ifx_Mimose_Config_t()
        self._cdll.ifx_mimose_get_config_defaults(self.handle, byref(config))
        return config

    def start_acquisition(self) -> None:
        """Start acquisition of time domain data

        Start the acquisition of time domain data from the connected device.
        Start the acquisition of time domain data from the connected device.
        If the data acquisition is already running the function has no effect.
        """
        self._cdll.ifx_mimose_start_acquisition(self.handle)

    def get_register(self, register_address) -> None:
        """Reads the value of a single register.

        Reads the value of a single register from the MIMOSE device and returns the value.
        """
        return self._cdll.ifx_mimose_get_register_value(self.handle, c_short(register_address))

    def set_register(self, register_address, register_value) -> None:
        """Sets the value of a single register.

        Writes the value of a single register to the MIMOSE device.
        """
        new_val = c_uint32(np.left_shift(np.bitwise_and(register_address,0xFFFF), 16) + np.bitwise_and(register_value,0xFFFF))
        self._cdll.ifx_mimose_set_registers(self.handle, byref(new_val), 1)

    def stop_acquisition(self) -> None:
        """Stop acquisition of time domain data

        Stop the acquisition of time domain data from the connected device.
        If the data acquisition is already stopped the function has no effect.
        """
        self._cdll.ifx_mimose_stop_acquisition(self.handle)

    def get_next_frame(self, timeout_ms: typing.Optional[int] = None) -> typing.Tuple[np.ndarray, MimoseMetadata]:
        """Retrieve next frame of time domain data from Mimose device.

        Retrieve the next complete frame of time domain data from the connected
        device. The samples from all chirps and all enabled RX antennas will be
        copied to the provided data structure frame.

        The frame is returned as numpy array with dimensions the num_of_samples, 
        which is a member variable in the Ltr11Config structure. 
        The metadata of type Ltr11Metadata is returned. 

        If timeout_ms is given, the exception ErrorTimeout is raised if a
        complete frame is not available within timeout_ms milliseconds.
        """
        metadata = MimoseMetadata()

        if timeout_ms:
            frame = self._cdll.ifx_mimose_get_next_frame_timeout(
                self.handle, None, byref(metadata), timeout_ms)
        else:
            frame = self._cdll.ifx_mimose_get_next_frame(
                self.handle, None, byref(metadata))

        frame_numpy = frame.contents.to_numpy()
        ifx_mda_destroy_c(frame)

        return frame_numpy, metadata

    def update_rc_lut(self) -> None:
        """updates RC look up table through device tuning.
        
        If the returned system clock in RC mode differs more than a certain extent
        compared to the desired clock, this function can be called to retune the
        RC Look up table (LUT) which can have device and environment specific variations.
        """
        self._cdll.ifx_mimose_update_rc_lut(self.handle)
#
#    def register_dump_to_file(self, filename: str) -> None:
#        """Dump register list to a file"""
#        filename_buffer = filename.encode("ascii")
#        filename_buffer_p = c_char_p(filename_buffer)
#        self._cdll.ifx_ltr11_register_dump_to_file(self.handle, file_name_buffer_p)

    def check_config(self, config: ifx_Mimose_Config_t, frame_configuration_index: int) -> bool:
        """
        Checks if the given configuration is valid
        :param config: Pointer to the config structure to check that it is valid
        :param frame_configuration_index: Configuration index 0 or 1 which should be checked
        :return: True if the given config is valid, false otherwise
        """
        config_valid = self._cdll.ifx_mimose_check_config(self.handle, byref(config), frame_configuration_index)
        return bool(config_valid)

    def get_number_of_samples_limits(self, config: typing.Optional[ifx_Mimose_Config_t] = None) -> (int, int):
        """
        Returns the number of samples limits (min and max).
        :return: Minimum number of samples, Maximum number of samples
        """
        min_num_samples = c_uint16()
        max_num_samples = c_uint16()
        if config:
            self._cdll.ifx_mimose_get_number_of_samples_limits(self.handle,
                                                               config.FrameConfig[0].selected_pulse_configs,
                                                               byref(min_num_samples),
                                                               byref(max_num_samples))
        else:
            self._cdll.ifx_mimose_get_number_of_samples_limits(self.handle,
                                                               self.config.FrameConfig[0].selected_pulse_configs,
                                                               byref(min_num_samples),
                                                               byref(max_num_samples))
        return min_num_samples.value, max_num_samples.value

    def get_pulse_repetition_time_limits_s(self, config: typing.Optional[ifx_Mimose_Config_t] = None) -> (float, float):
        """
        Returns the pulse repetition time (prt) limits in seconds.
        :return: Minimum prt (depending on selected pulse configs), Maximum prt (Fixed not depending on selected pulse configs)
        """
        min_prt = c_float()
        max_prt = c_float()
        if config:
            self._cdll.ifx_mimose_get_pulse_repetition_time_limits_s(self.handle,
                                                                     config.FrameConfig[0].selected_pulse_configs,
                                                                     byref(min_prt),
                                                                     byref(max_prt))
        else:
            self._cdll.ifx_mimose_get_pulse_repetition_time_limits_s(self.handle,
                                                                     self.config.FrameConfig[0].selected_pulse_configs,
                                                                     byref(min_prt),
                                                                     byref(max_prt))
        return min_prt.value, max_prt.value

    def get_frame_repetition_time_limits_s(self, config: typing.Optional[ifx_Mimose_Config_t] = None) -> (float, float):
        """
        Returns the frame repetition time limits in seconds.
        :return: Minimum frame repetition time in seconds, Maximum frame repetition time in seconds (Max fixed not depending on the parameters)
        """
        min_frt = c_float()
        max_frt = c_float()
        if config:
            self._cdll.ifx_mimose_get_frame_repetition_time_limits_s(self.handle,
                                                                     config.FrameConfig[0].selected_pulse_configs,
                                                                     config.FrameConfig[0].num_of_samples,
                                                                     config.AFC_Config.afc_duration_ct,
                                                                     config.FrameConfig[0].pulse_repetition_time_s,
                                                                     byref(min_frt),
                                                                     byref(max_frt))
        else:
            self._cdll.ifx_mimose_get_frame_repetition_time_limits_s(self.handle,
                                                                     self.config.FrameConfig[0].selected_pulse_configs,
                                                                     self.config.FrameConfig[0].num_of_samples,
                                                                     self.config.AFC_Config.afc_duration_ct,
                                                                     self.config.FrameConfig[0].pulse_repetition_time_s,
                                                                     byref(min_frt),
                                                                     byref(max_frt))
        return min_frt.value, max_frt.value

    def get_rf_center_frequency_limits_Hz(self, config: typing.Optional[ifx_Mimose_Config_t] = None) -> (int, int):
        """
        Returns the RF Center Frequency limits in Hz.
        :return: Minimum RF frequence, Maximum RF frequence
        """
        min_rf_center_frequency = c_uint64()
        max_rf_center_frequency = c_uint64()
        if config:
            self._cdll.ifx_mimose_get_rf_center_frequency_limits_Hz(self.handle,
                                                                    config.AFC_Config.band,
                                                                    byref(min_rf_center_frequency),
                                                                    byref(max_rf_center_frequency))
        else:
            self._cdll.ifx_mimose_get_rf_center_frequency_limits_Hz(self.handle,
                                                                    self.config.AFC_Config.band,
                                                                    byref(min_rf_center_frequency),
                                                                    byref(max_rf_center_frequency))
        return min_rf_center_frequency.value, max_rf_center_frequency.value

    def get_default_limits(self) -> ifx_Mimose_Config_Limits_t:
        """
        Returns the default limiting values for a configuration.
        :return: The limits instance to be written
        """
        limits = ifx_Mimose_Config_Limits_t()
        self._cdll. ifx_mimose_get_default_limits(self.handle, byref(limits))
        return limits

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self._close()
#
    def _close(self):
        """Destroy device handle"""
        if hasattr(self, "handle") and self.handle:
            self._cdll.ifx_mimose_destroy(self.handle)
            self.handle = None

    def __del__(self):
        try:
            self._close()
        except:
            # just like in C++ exceptions raised in a destructor cannot be catched
            # for this reason we ignore them
            pass
