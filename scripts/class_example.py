#!/usr/bin/env python

"""
Dynamic dynamic reconfigure server example with a class.

Author: Sammy Pfeiffer
"""

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

class Example(object):
    def __init__(self):
        # Create a D(ynamic)DynamicReconfigure
        self.ddr = DDynamicReconfigure("class_example")

        # Add variables (name, description, default value, min, max, edit_method)
        self.ddr.add_variable("decimal", "float/double variable", 0.0, -1.0, 1.0)
        self.ddr.add_variable("integer", "integer variable", 0, -1, 1)
        self.ddr.add_variable("bool", "bool variable", True)
        self.ddr.add_variable("string", "string variable", "string dynamic variable")
        enum_method = self.ddr.enum([ self.ddr.const("Small",      "int", 0, "A small constant"),
                           self.ddr.const("Medium",     "int", 1, "A medium constant"),
                           self.ddr.const("Large",      "int", 2, "A large constant"),
                           self.ddr.const("ExtraLarge", "int", 3, "An extra large constant")],
                         "An enum example")
        self.ddr.add_variable("enumerate", "enumerate variable", 1, 0, 3, edit_method=enum_method)

        self.add_variables_to_self()

        self.ddr.start(self.dyn_rec_callback)

    def add_variables_to_self(self):
        var_names = self.ddr.get_variable_names()
        for var_name in var_names:
            self.__setattr__(var_name, None)

    def dyn_rec_callback(self, config, level):
        rospy.loginfo("Received reconf call: " + str(config))
        # Update all variables
        var_names = self.ddr.get_variable_names()
        for var_name in var_names:
            self.__dict__[var_name] = config[var_name]
        return config

if __name__ == '__main__':
    rospy.init_node('test_ddynrec')

    # Create a D(ynamic)DynamicReconfigure
    E = Example()

    rospy.spin()
