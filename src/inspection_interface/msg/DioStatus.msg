# Digital I/O status snapshot
std_msgs/Header header
bool   connected         # true if gpiochip is accessible
bool[] di_values         # digital input states (index = channel)
bool[] do_values         # digital output readback states (index = channel)
string error_code        # "OK" or error description
