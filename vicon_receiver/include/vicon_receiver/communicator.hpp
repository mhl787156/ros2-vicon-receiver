#if !defined(COMMUNICATOR_HPP)
#define COMMUNICATOR_HPP

#include "DataStreamClient.h"
#include "DataStreamRetimingClient.h"
#include "rclcpp/rclcpp.hpp"
#include "publisher.hpp"
#include <iostream>
#include <map>
#include <chrono>
#include <string>

#include "blocking_collections.h"

#ifdef _WIN32
#include <io.h>
#include <Windows.h>
#endif

#ifdef unix
#include <unistd.h> 
#endif

#include "boost/thread.hpp"

using namespace std;
using namespace code_machina;


void mySleep(int sleepMs)
{
#ifdef unix
    sleep(sleepMs);   // usleep takes sleep time in us (1 millionth of a second)
#endif
#ifdef _WIN32
    Sleep(sleepMs);
#endif
}

// Main Node class
class Communicator : public rclcpp::Node
{
private:
    // ViconDataStreamSDK::CPP::Client vicon_client;
    ViconDataStreamSDK::CPP::RetimingClient vicon_client;
    string hostname;
    unsigned int buffer_size;
    string ns_name;
    float frame_rate;
    uint32_t frame_number = 0;
    // map<string, Publisher> pub_map;
    map<string, shared_ptr<BlockingCollection<PositionStruct>>> queue_map;
    // boost::mutex mutex;

    void queue_monitor(const string subject_name, const string segment_name, shared_ptr<BlockingCollection<PositionStruct>> queue);
public:
    Communicator();
    ~Communicator();

    // Initialises the connection to the DataStream server
    bool connect();

    // Stops the current connection to a DataStream server (if any).
    bool disconnect();

    // Main loop that request frames from the currently connected DataStream server and send the 
    // received segment data to the Publisher class.
    void get_frame();

    // functions to create a segment publisher in a new thread
    void create_publisher(const string subject_name, const string segment_name);
    void create_publisher_thread(const string subject_name, const string segment_name);
};

#endif // COMMUNICATOR_HPP
