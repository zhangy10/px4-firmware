
# parameter_client_reset_request : Used by the server to reset one or all parameter value(s) on the client

uint64 timestamp			# time since system start (microseconds)

bool update_all             # If this is true then ignore the parameter_name
char[17] parameter_name     # Parameter name with null termination
