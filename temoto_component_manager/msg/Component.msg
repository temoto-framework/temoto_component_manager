# Name of the component
string component_name

# Type of the requested component (required).
string component_type

# Name of the ROS package where component is located. (optional)
# If the package name is left undefined
# then the first known entry under the specified type is executed
string package_name

# name of the executable (optional)
# When undefined, first defined entry is used.
string executable

# Output topics. Each element is a pair of strings where the 
# first element (key) describes the type of the topic and the
# second element (value) describes the name of the topic itself.
# Key is required but if the value is left empty, a default
# topic name is used (indicated by the component info file)
# by the component manager
diagnostic_msgs/KeyValue[] output_topics

# Same idea applies here as it did with the output_topics
diagnostic_msgs/KeyValue[] input_topics

# Similar to output topics but in this case the key is the type
# of the parameter
diagnostic_msgs/KeyValue[] required_parameters

