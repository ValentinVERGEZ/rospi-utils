#!/bin/bash

rosservice call /witd_command "requestId: $1
requestType: 0"	# START