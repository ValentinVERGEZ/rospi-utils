#!/bin/bash

## Simply launch the witd (wit.ai daemon) on RaspberryPi
# Warning : work with static paths
WITD_PATH="/home/pi/wit-ai-raspPi"

# Launch witd
${WITD_PATH}/witd-armv6 | tee ${WITD_PATH}/wit.log