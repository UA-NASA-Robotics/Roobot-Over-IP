# ROS Node can talk to the module
bool module_connection
# Module is operational
bool module_operational
# Exact state of the module (See module codec sysadmin status report for details)
uint8 module_state
# Module is in error state (note not necessarily the same as !module_operational)
bool module_error
string module_error_message