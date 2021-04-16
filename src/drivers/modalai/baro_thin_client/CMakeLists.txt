
px4_add_module(
	MODULE drivers__modalai__baro_thin_client
	MAIN baro_thin_client
	SRCS
		baro_thin_client.cpp
	DEPENDS
		drivers_barometer
	)
