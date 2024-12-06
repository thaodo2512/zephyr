# Thao Do

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_MOTOR_DRIVE_L298 l298.c)
