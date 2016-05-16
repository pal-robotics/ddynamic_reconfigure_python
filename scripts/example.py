#!/usr/bin/env python

"""
Dynamic dynamic reconfigure server example.

Author: Sammy Pfeiffer
"""

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

def dyn_rec_callback(config, level):
    rospy.loginfo("Received reconf call: " + str(config))
    return config

if __name__ == '__main__':
    rospy.init_node('test_ddynrec')

    # Create a D(ynamic)DynamicReconfigure
    ddynrec = DDynamicReconfigure("example_dyn_rec")

    # Add variables (name, description, default value, min, max, edit_method)
    ddynrec.add_variable("decimal", "float/double variable", 0.0, -1.0, 1.0)
    ddynrec.add_variable("integer", "integer variable", 0, -1, 1)
    ddynrec.add_variable("bool", "bool variable", True)
    ddynrec.add_variable("string", "string variable", "string dynamic variable")
    enum_method = ddynrec.enum([ ddynrec.const("Small",      "int", 0, "A small constant"),
                       ddynrec.const("Medium",     "int", 1, "A medium constant"),
                       ddynrec.const("Large",      "int", 2, "A large constant"),
                       ddynrec.const("ExtraLarge", "int", 3, "An extra large constant")],
                     "An enum example")
    ddynrec.add_variable("enumerate", "enumerate variable", 1, 0, 3, edit_method=enum_method)

    # Start the server
    ddynrec.start(dyn_rec_callback)

    rospy.spin()
