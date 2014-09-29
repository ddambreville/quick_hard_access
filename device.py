class Device(object):

    """Class which describes a device."""

    def __init__(self, dcm, mem, name):
        self.dcm = dcm
        self.mem = mem
        self.name = name

        self.prefix = self.dcm.getPrefix()[1]

    def _get_ack(self):
        """Get board ack number."""
        return self.get("Ack")

    def _set_ack(self, request):
        """Set board ack number."""
        self.set("Ack", request[0], request[1])

    def _get_address(self):
        """Get board adress."""
        return self.get("Address")

    def _set_address(self, request):
        """Set board adress."""
        self.set("Address", request[0], request[1])

    def _get_availability(self):
        """Get board availability."""
        return self.get("Available")

    def _set_availability(self, request):
        """Set board availability."""
        self.set("Available", request[0], request[1])

    def _get_bid(self):
        """Get board bid."""
        return self.get("Bid")

    def _set_bid(self, request):
        """Set board bid."""
        self.set("Bid", request[0], request[1])

    def _get_board_id(self):
        """Get board board id."""
        return self.get("BoardId")

    def _set_board_id(self, request):
        """Set board board id."""
        self.set("BoardId", request[0], request[1])

    def _get_bootloader_version(self):
        """Get board bootloader version."""
        return self.get("BootLoaderVersion")

    def _set_bootloader_version(self, request):
        """Set board bootloader version."""
        self.set("BootLoaderVersion", request[0], request[1])

    def _get_bus(self):
        """Get board bus."""
        return self.get("Bus")

    def _set_bus(self, request):
        """Set board bus."""
        self.set("Bus", request[0], request[1])

    def _get_error(self):
        """Get board error."""
        return self.get("Error")

    def _set_error(self, request):
        """Set board error."""
        self.set("Error", request[0], request[1])

    def _get_motorboard_serial_number(self):
        """Get board serial number."""
        return self.get("MotorBoardSerialNumber")

    def _set_motorboard_serial_number(self, request):
        """Set board serial number."""
        self.set("MotorBoardSerialNumber", request[0], request[1])

    def _get_nack(self):
        """Get board nack number."""
        return self.get("Nack")

    def _set_nack(self, request):
        """Set board nack number."""
        self.set("Nack", request[0], request[1])

    def _get_prog_version(self):
        """Get board prog version."""
        return self.get("ProgVersion")

    def _set_prog_version(self, request):
        """Set board prog version."""
        self.set("ProgVersion", request[0], request[1])

    def _get_robot_id(self):
        """Get board robot id."""
        return self.get("RobotId")

    def _set_robot_id(self, request):
        """Set board robot id."""
        self.set("RobotId", request[0], request[1])

    def _get_stored_serial_number(self):
        """Get board serial number."""
        return self.get("StoredSerialNumber")

    def _set_stored_serial_number(self, request):
        """Set board serial number."""
        self.set("StoredSerialNumber", request[0], request[1])

    def _get_struc_version(self):
        """Get board structure version."""
        return self.get("StructVersion")

    def _set_struc_version(self, request):
        """Set board structure version."""
        self.set("StructVersion", request[0], request[1])

    def _get_board_type(self):
        """Get board type."""
        return self.get("Type")

    def _set_board_type(self, request):
        """Set board type."""
        self.set("Type", request[0], request[1])

    def _get_brake_version(self):
        """Get board brake version."""
        return self.get("brakeVersion")

    def _set_brake_version(self, request):
        """Set board brake version."""
        self.set("brakeVersion", request[0], request[1])

    def _get_capacitive_version(self):
        """Get board capacitive version."""
        return self.get("capacitiveVersion")

    def _set_capacitive_version(self, request):
        """Set board capacitive version."""
        self.set("capacitiveVersion", request[0], request[1])

    def _get_current_calibration_m1(self):
        """Get board current calibration of motor 1."""
        return self.get("currentCalibrationResultMotor1")

    def _set_current_calibration_m1(self, request):
        """Set board current calibration of motor 1."""
        self.set("currentCalibrationResultMotor1", request[0], request[1])

    def _get_current_calibration_m2(self):
        """Get board current calibration of motor 2."""
        return self.get("currentCalibrationResultMotor2")

    def _set_current_calibration_m2(self, request):
        """Set board current calibration of motor 2."""
        self.set("currentCalibrationResultMotor2", request[0], request[1])

    ack = property(_get_ack, _set_ack)
    address = property(_get_address, _set_address)
    availability = property(_get_availability, _set_availability)
    bid = property(_get_bid, _set_bid)
    board_id = property(_get_board_id, _set_board_id)
    bootloader_version = property(
        _get_bootloader_version, _set_bootloader_version)
    bus = property(_get_bus, _set_bus)
    error = property(_get_error, _set_error)
    motorboard_serial_number = property(
        _get_motorboard_serial_number, _set_motorboard_serial_number)
    nack = property(_get_nack, _set_nack)
    prog_version = property(_get_prog_version, _set_prog_version)
    robot_id = property(_get_robot_id, _set_robot_id)
    stored_serial_number = property(
        _get_stored_serial_number, _set_stored_serial_number)
    struc_version = property(_get_struc_version, _set_struc_version)
    board_type = property(_get_board_type, _set_board_type)
    brake_version = property(_get_brake_version, _set_brake_version)
    capacitive_version = property(
        _get_capacitive_version, _set_capacitive_version)
    current_calibration_m1 = property(
        _get_current_calibration_m1, _set_current_calibration_m1)
    current_calibration_m2 = property(
        _get_current_calibration_m2, _set_current_calibration_m2)

    def get(self, attribut):
        """Accessor which uses ALMemory module."""
        return self.mem.getData(self.prefix + self.name + "/" + attribut)

    def set(self, attribut, timed_commands, update_type="ClearAll"):
        """Mutator which uses DCM module."""
        key = self.name + "/" + attribut
        self.dcm.set([key, update_type, timed_commands])
