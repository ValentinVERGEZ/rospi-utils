### TODO - Move all the state info to CloudNodeState.msg
## Timestamp
time stamp
## State
# Constants
uint8   IDLE=0
uint8   RECORDING=1
uint8   TRANSFERRING_DATA=2
# Value
uint8   state

## Request relative
bool    last_request_success
uint32  last_request_id
# Error or sucess message, convenient for debug
string  last_request_message
uint32  actual_request_id

## Request response
string raw_data