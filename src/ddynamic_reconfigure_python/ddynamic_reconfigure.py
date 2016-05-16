#!/usr/bin/env python

"""
Dynamic dynamic reconfigure server.

Just register your variables for the dynamic reconfigure
and call start with a callback.

Author: Sammy Pfeiffer
"""

from dynamic_reconfigure.server import Server
from dynamic_reconfigure.parameter_generator_catkin import ParameterGenerator
from dynamic_reconfigure.encoding import extract_params
from rospkg import RosPack
import rospy


class DDynamicReconfigure(ParameterGenerator):

    def __init__(self, name=None):
        global id
        self.group = self.Group(self, "Default", "", True, 0, 0)
        id = 1
        if name is None:
            self.name = rospy.get_name() + "_dyn_rec"
        else:
            self.name = name
        self.constants = []
        rp = RosPack()
        self.dynconfpath = rp.get_path('dynamic_reconfigure')

    def get_type(self):
        class TypeClass(object):
            def __init__(self, config_description):
                self.config_description = config_description
                self.min = {}
                self.max = {}
                self.defaults = {}
                self.level = {}
                self.type = {}
                self.all_level = 0

                for param in extract_params(config_description):
                    self.min[param['name']] = param['min']
                    self.max[param['name']] = param['max']
                    self.defaults[param['name']] = param['default']
                    self.level[param['name']] = param['level']
                    self.type[param['name']] = param['type']
                    self.all_level = self.all_level | param['level']
        return TypeClass(self.group.to_dict())

    def register(self, name, description, default=None, min=None, max=None, edit_method=""):
        """Register variable, like gen.add() but deducting the type"""
        if type(default) == int:
            if edit_method == "":
                self.add(name, "int", 0, description, default, min, max)
            else: # enum
                self.add(name, "int", 0, description, default, min, max, edit_method)
        elif type(default) == float:
            self.add(name, "double", 0, description, default, min, max)
        elif type(default) == str:
            self.add(name, "str", 0, description, default)
        elif type(default) == bool:
            self.add(name, "bool", 0, description, default)

    def start(self, callback):
        self.dyn_rec_srv = Server(self.get_type(), callback)


if __name__ == '__main__':
    rospy.init_node('test_ddynrec')

    print "Instancing DDynamicReconfigure"
    ddynrec = DDynamicReconfigure()

    ddynrec.register("decimal", "float/double variable", 0.0, -1.0, 1.0)
    ddynrec.register("integer", "integer variable", 0, -1, 1)
    ddynrec.register("bool", "bool variable", True)
    ddynrec.register("string", "string variable", "string dynamic variable")
    enum_method = ddynrec.enum([ ddynrec.const("Small",      "int", 0, "A small constant"),
                       ddynrec.const("Medium",     "int", 1, "A medium constant"),
                       ddynrec.const("Large",      "int", 2, "A large constant"),
                       ddynrec.const("ExtraLarge", "int", 3, "An extra large constant")],
                     "An enum example")
    ddynrec.register("enumerate", "enumerate variable", 1, 0, 3, edit_method=enum_method)

    print "Registered variables to be dynamic reconfigure"

    def dyn_rec_callback(config, level):
        rospy.loginfo("Received reconf call: " + str(config))
        return config

    ddynrec.start(dyn_rec_callback)
    rospy.spin()
