import ConfigParser
import os
import zipfile
import subdevice


class Timer(object):

    """Class which defines a timer synchronized on DCM module"""

    def __init__(self, dcm, time_out):
        self.dcm = dcm
        self.time_out = time_out

        self.time_init = dcm.getTime(0)

    def dcm_time(self):
        """Return DCM time."""
        return self.dcm.getTime(0) - self.time_init

    def is_time_not_out(self):
        """To comment"""

        return self.dcm_time() < self.time_out

    def is_time_out(self):
        """Returns True if time is out"""
        return self.dcm_time() >= self.time_out


def wait(dcm, time_to_wait):
    """Wait time synchronized on DCM module"""
    initial_time = dcm.getTime(0)
    while dcm.getTime(0) - initial_time < time_to_wait:
        pass


def use_section(my_file, section):
    """Use a section from a configuration file."""
    to_use = []

    config = ConfigParser.ConfigParser()
    config.optionxform = str
    config.read(my_file)
    config_section = config._sections[section]
    config_section.pop("__name__")

    for key, value in config_section.items():
        if value == "1":
            to_use.append(key)

    return to_use


def read_section(my_file, section):
    """Read section from a configuration file."""
    config = ConfigParser.ConfigParser()
    config.optionxform = str
    config.read(my_file)

    if config.has_section(section):
        config_section = config._sections[section]
        config_section.pop("__name__")

        for key, value in config_section.items():
            config_section[key] = value.split()

        return config_section

    else:
        return {}


def read_parameter(my_file, section, parameter):
    """Read parameter from configuration file."""
    section_to_return = read_section(my_file, section)
    return section_to_return[parameter][0]


def read_int_tuple(my_file, section, parameter):
    """To comment."""
    param = read_parameter(my_file, section, parameter)
    param = param.split(",")
    for i, elt in enumerate(param):
        param[i] = int(elt)
    return tuple(param)


def read_over_section(config_before, my_file, section):
    """To comment."""
    config_after = config_before.copy()

    config_modif = read_section(my_file, section)
    for key, value in config_modif.items():
        config_after[key] = value

    return config_after


def log_results(log_dic, *params):
    """Logger."""
    for param in params:
        key = param[0]
        value = param[1]
        if log_dic.has_key(key) is False:
            log_dic[key] = []

        log_dic[key].append(value)


def log_file_write(file_path, subdevice):
    """To comment."""
    subdevice_type = subdevice.type
    subdevice_shortname = subdevice.short_name

    try:
        os.makedirs(os.path.dirname(file_path))
    except:
        pass

    with open(file_path, 'w') as my_file:
        my_file.write("% " + subdevice_type + " " + subdevice_shortname.replace(
            "/", "") + " " + str(subdevice.state) + "\n")

        [my_file.write(key + " ") for key in subdevice.log_dic]
        my_file.write("\n")

        first_value_list = True

        list_size_ref = 0
        for value_list in subdevice.log_dic.values():
            if first_value_list:
                first_value_list = False
                list_size_ref = len(value_list)
            else:
                assert len(value_list) == list_size_ref

        if list_size_ref != 0:
            line_num = 0
            while line_num < list_size_ref:
                for value_list in subdevice.log_dic.values():
                    my_file.write("{} ".format(value_list[line_num]))
                my_file.write("\n")
                line_num += 1


def zip_results(zipFile, filePath):
    """To comment."""
    with zipfile.ZipFile(zipFile, mode='a', compression=zipfile.ZIP_STORED) as zip_object:
        zip_object.write(filePath)


def floatExcept(my_string):
    try:
        return float(my_string)
    except (RuntimeError, TypeError, NameError):
        return my_string


def separate(my_string, separator):
    """To comment."""
    dum = my_string.split(separator)
    try:
        while True:
            dum.remove("")
    except (RuntimeError, TypeError, NameError):
        pass
    return dum


def int_to_two_digit_string(number):
    """Convert an integer in a two digits string."""
    if number < 10:
        return "0" + str(number)
    else:
        return str(number)


class SlidingAverage(object):

    """Create a sliding average object."""

    def __init__(self, nb_points):
        self.nb_points = nb_points
        self.current_point = 0
        self.points = []

    def point_add(self, value):
        """Add a point in the list."""
        if len(self.points) < self.nb_points:
            self.points.append(value)
        else:
            self.points[self.current_point] = value
            if self.current_point == self.nb_points - 1:
                self.current_point = 0
            else:
                self.current_point += 1

    def calc(self):
        """Calculate the next averaged point."""
        my_sum = 0
        for point in self.points:
            my_sum += point

        return float(my_sum) / len(self.points)


def read_list_file(filepath):
    """Read a file and return a list containing the information."""
    my_list = []
    for line in open(filepath, "r"):
        line = line.rstrip('\n')
        if line[0] != '#':
            my_list.append(line)
    return my_list


def reduce_name(name, sep, *strToRemove):
    """Reduce name by removing specified strings from it"""
    name_sp = name.split(sep)
    for rm_st in strToRemove:
        if rm_st in name_sp:
            name_sp.remove(rm_st)

    reduced_name = name_sp[0]
    for elt in name_sp[1:]:
        reduced_name += (sep + elt)

    return reduced_name


class Logger(object):

    """a completer"""

    def __init__(self):
        self.log_dic = {}

    def log(self, *params):
        """Log the chosen parameters"""
        for param in params:
            key = param[0]
            value = param[1]
            if self.log_dic.has_key(key) is False:
                self.log_dic[key] = []

            self.log_dic[key].append(value)

    def log_file_write(self, file_path):
        """Write results from the dictionnary."""
        try:
            os.makedirs(os.path.dirname(file_path))
        except:
            pass

        with open(file_path, 'w') as my_file:
            # creating the header
            my_file.write(",".join(self.log_dic.keys()))
            my_file.write("\n")

            first_value_list = True

            list_size_ref = 0
            for value_list in self.log_dic.values():
                if first_value_list:
                    first_value_list = False
                    list_size_ref = len(value_list)
                else:
                    assert len(value_list) == list_size_ref

            if list_size_ref != 0:
                line_num = 0
                while line_num < list_size_ref:
                    tmp_list = list()
                    for value_list in self.log_dic.values():
                        tmp_list.append(str(value_list[line_num]))
                    my_file.write(",".join(tmp_list))
                    my_file.write("\n")
                    line_num += 1


def jointlistcreation_from_state(dico):
    """
    Return a list of all joints in a configuration file of position
    ----- Example of use -----
    - The dictionnary is:
    {"HeadPitch/Position/Actuator" : max,
    "HeadYaw/Position/Actuator" : max}
    - The method returns:
    ["HeadPitch", "HeadYaw"]
    """
    list_to_create = list()
    for key in dico:
        joint = key.split("/")[0]
        list_to_create.append(str(joint))
    return list_to_create


def is_stiffness_null(dcm, mem, joint_list):
    """Returns True if all joint hardness are null."""
    cpt = 0
    for joint in joint_list:
        joint_hardness_actuator = subdevice.JointHardnessActuator(
            dcm,
            mem,
            str(joint)
        )
        if float(joint_hardness_actuator.value) >= 0.0:
            cpt += 1
    if cpt == 0:
        return True
    else:
        return False


class switch( object ):
    """
    Class which allow to make a switch/case
    operation in Python
    """
    value = None
    def __new__(self, value):
        self.value = value
        return True

def case(*args):
    """case function"""
    return any((arg == switch.value for arg in args))
