
px4_add_board(
	PLATFORM nuttx
	VENDOR modalai
	MODEL voxl2io-v2
	TOOLCHAIN arm-none-eabi
	CONSTRAINED_FLASH
	ARCHITECTURE cortex-m3
	DRIVERS
	MODULES
		px4iofirmware
	)
