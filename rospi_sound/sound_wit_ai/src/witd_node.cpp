#include <sound_wit_ai/witd_node.h>

// Global var containting infos to ouput
sound_messages::WitdOutput msg;
bool uglyMutex;


//Struct receiving LibCurl output
struct BufferStruct
{
  char* buffer;
  size_t size;
};

// Main
int main(int argc, char** argv)
{
	CURL *curl;
	CURLcode res;

	ros::init(argc, argv, "witd_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

    ros::ServiceServer service = nh.advertiseService("witd_command", service_callback);
    ros::Publisher output_pub = nh.advertise<sound_messages::WitdOutput>("witd_output", 1000);
    uglyMutex=false;
    // Init msg
    ROS_INFO("IDLE");
    while(uglyMutex);uglyMutex=true;
        msg.stamp = ros::Time::now();
        msg.state = msg.IDLE;
        msg.last_request_id = 0;
        msg.actual_request_id = 0;
        msg.last_request_success = true;
        msg.last_request_message = "No request received";
        msg.raw_data = "";
    uglyMutex=false;

	while(nh.ok()) {
		ros::spinOnce();

        while(uglyMutex);uglyMutex=true;
            msg.stamp = ros::Time::now();
            output_pub.publish(msg);
        uglyMutex=false;

		loop_rate.sleep();
	}

	return 0;
}


// Callback for filling libcurl output struct
size_t write_callback(char *ptr, size_t size, size_t nmemb, void *data)
{
    ROS_INFO("CURL Response RECEIVED !!");

    size_t realsize = size * nmemb;

    // Create str from data
    std::string tmp;
    tmp += ptr;
    tmp.resize(nmemb);

    // Fill output struct
    if(msg.state != msg.RECORDING) { // Ugly way to avoid the {"status":"ok"} response
	    ROS_INFO("IDLE");
	    while(uglyMutex);uglyMutex=true;
		msg.raw_data = tmp.c_str();
		msg.last_request_id = msg.actual_request_id;
		msg.last_request_success = true;
		msg.last_request_message = "OK";
		msg.state = msg.IDLE;
	    uglyMutex=false; 
    }

    ROS_INFO_STREAM(tmp);

    return realsize;
}


bool service_callback(sound_messages::WitdCommand::Request &req,
                    sound_messages::WitdCommand::Response &res)
{
    // On START
    if(req.requestType == req.START) {
        // If new id accept
        while(uglyMutex);uglyMutex=true;
        if(req.requestId != msg.actual_request_id && msg.state==msg.IDLE) {
            // Construct response
            std::string tmp;
            tmp += "Start request id #";
            tmp +=req.requestId;
            tmp +=" accepted";
            res.message=tmp.c_str();
            res.accepted=true;

            // Update msg
	    ROS_INFO("RECORDING");
            msg.actual_request_id = req.requestId;
            msg.state=msg.RECORDING;
            uglyMutex=false; 

            // Launch request
            std::string req_str("http://localhost:9877/start?access_token=");
            req_str += ACCESS_TOKEN;
            curlGET((char*)req_str.c_str());
        }
        // If same id refuse
        else {
            // Construct response
            std::string tmp;
            tmp += "Start request id #";
            tmp += req.requestId;
            tmp += " rejected\n";

            if(req.requestId == msg.actual_request_id) {
                tmp += "ID already used";
            }
            else if(msg.state!=msg.IDLE) {
                tmp += "Node currently busy";
            }
            else {
                tmp += "Unknown reason";
            }
            uglyMutex=false; 

            res.message=tmp.c_str();
            res.accepted=false;
        }
    }
    // On STOP
    else if(req.requestType == req.STOP) {
        // If same id accept
        while(uglyMutex);uglyMutex=true;
        if(req.requestId == msg.actual_request_id && msg.state==msg.RECORDING) {
            // Construct response
            std::string tmp;
            tmp += "Stop request id #";
            tmp +=req.requestId;
            tmp +=" accepted";
            res.message=tmp.c_str();
            res.accepted=true;     

            // Update msg
	    ROS_INFO("Transferring data");
            msg.state=msg.TRANSFERRING_DATA;       
            uglyMutex=false; 

	    // Ugly way to publish the new state before the blocking CURL process
	    ros::NodeHandle nh;
	    ros::Publisher output_pub = nh.advertise<sound_messages::WitdOutput>("witd_output", 1000, true);
	    output_pub.publish(msg);

            // Launch request
            std::string req_str("http://localhost:9877/stop");
            curlGET((char*)req_str.c_str());

        }
        // If other id refuse
        else {
            // Construct response
            std::string tmp;
            tmp += "Stop request id #";
            tmp += req.requestId;
            tmp += " rejected\n";

            if(req.requestId != msg.actual_request_id) {
                tmp += "Bad ID used";
            }
            else if(msg.state!=msg.RECORDING) {
                tmp += "Nothing to stop";
            }
            else {
                tmp += "Unknown reason";
            }
            uglyMutex=false; 

            res.message=tmp.c_str();
            res.accepted=false;
        }

    }
    // On unknown - refuse
    else {
        // Construct response
        res.accepted=false;
        res.message="Unknown request type";
    }

    return true;
}


// Libcurl get request
int curlGET(char* AddURL)
{
  curl_global_init(CURL_GLOBAL_ALL);
 
  CURL *myHandle;
  CURLcode result;
 
  myHandle = curl_easy_init();
  curl_easy_setopt(myHandle, CURLOPT_WRITEFUNCTION, write_callback);
  curl_easy_setopt(myHandle, CURLOPT_URL, AddURL);
  result = curl_easy_perform(myHandle);
  curl_easy_cleanup(myHandle);

  ROS_INFO("Curl result code : %d",result);

  return 0;
}
