#ifndef _SOUND_WIT_AI__WITD_NODE_
#define _SOUND_WIT_AI__WITD_NODE_

// Includes
#include <ros/ros.h>
#include <curl/curl.h>

#include <stdio.h>
#include <stdlib.h>
#include <sound_wit_ai/ACCESS_TOKEN.h>
#include <string>

#include "sound_messages/WitdCommand.h"
#include "sound_messages/WitdOutput.h"

// Prototypes
size_t write_callback(char *ptr, size_t size, size_t nmemb, void *userdata);
bool service_callback(sound_messages::WitdCommand::Request &req,
                    sound_messages::WitdCommand::Response &res);
int curlGET(char* AddURL);

#endif // _SOUND_WIT_AI__WITD_NODE_
