#!/bin/bash

west build -b rpi_pico -p always samples/boards/raspberrypi/rpi_pico/uart_pio/

cp build/compile_commands.json .
