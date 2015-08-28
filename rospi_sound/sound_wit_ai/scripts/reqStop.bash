#!/bin/bash

rosservice call /witd_command "requestId: $1
requestType: 1"	# STOP