#include "vicon_receiver/communicator.hpp"

using namespace ViconDataStreamSDK::CPP;
using namespace code_machina;

Communicator::Communicator() : Node("vicon")
{
    // get parameters
    this->declare_parameter<std::string>("hostname", "127.0.0.1");
    this->declare_parameter<int>("buffer_size", 200);
    this->declare_parameter<std::string>("namespace", "vicon");
    this->declare_parameter<float>("output_frame_rate_hz", 50);
    this->get_parameter("hostname", hostname);
    this->get_parameter("buffer_size", buffer_size);
    this->get_parameter("namespace", ns_name);
    this->get_parameter("output_frame_rate_hz", frame_rate);
}

Communicator::~Communicator() 
{
    this->disconnect();
}

bool Communicator::connect()
{   
    // If vicon client is already connected just ignore
    if (vicon_client.IsConnected().Connected) {
        return true;
    }
    RCLCPP_INFO(this->get_logger(), "Not Connected, Establishing Connection");

    // connect to server
    RCLCPP_INFO(this->get_logger(), "Connecting to %s ...", hostname.c_str());
    int counter = 0;
    while (!vicon_client.IsConnected().Connected)
    {
        // Connect to the client and set the frame rate
        this->get_parameter("output_frame_rate_hz", frame_rate);
        bool ok = (vicon_client.Connect(hostname, this->frame_rate).Result == Result::Success);
        if (!ok)
        {
            counter++;
            RCLCPP_WARN(this->get_logger(), "Connect failed, reconnecting ( %d )...", counter);
            mySleep(1);
        }
    }
    RCLCPP_INFO(this->get_logger(),  "Connection successfully established with %s", hostname.c_str());

    // perform further initialization
    // vicon_client.EnableLightweightSegmentData();
    RCLCPP_INFO(this->get_logger(), "Initialisation complete");

    return true;
}

bool Communicator::disconnect()
{
    if (!vicon_client.IsConnected().Connected)
        return true;
    mySleep(1);
    // vicon_client.DisableLightweightSegmentData();
    RCLCPP_INFO(this->get_logger(), "Disconnecting from %s ...", hostname.c_str());
    vicon_client.Disconnect();
    RCLCPP_INFO(this->get_logger(), "Successfully disconnected");
    if (!vicon_client.IsConnected().Connected)
        return true;
    return false;
}

void Communicator::get_frame()
{
    vicon_client.WaitForFrame();
    frame_number++;
    // Output_GetFrameNumber frame_number = vicon_client.GetFrameNumber();

    unsigned int subject_count = vicon_client.GetSubjectCount().SubjectCount;

    for (unsigned int subject_index = 0; subject_index < subject_count; ++subject_index)
    {
        // get the subject name
        string subject_name = vicon_client.GetSubjectName(subject_index).SubjectName;

        // count the number of segments
        unsigned int segment_count = vicon_client.GetSegmentCount(subject_name).SegmentCount;

        for (unsigned int segment_index = 0; segment_index < segment_count; ++segment_index)
        {
            auto current_time = this->now();

            // get the segment name
            string segment_name = vicon_client.GetSegmentName(subject_name, segment_index).SegmentName;

            // get position of segment in millimeters
            PositionStruct current_position;
            Output_GetSegmentGlobalTranslation trans =
                vicon_client.GetSegmentGlobalTranslation(subject_name, segment_name);
            Output_GetSegmentGlobalRotationQuaternion rot =
                vicon_client.GetSegmentGlobalRotationQuaternion(subject_name, segment_name);
            
            for (size_t i = 0; i < 4; i++)
            {
                // Set values and convert from millimeters to meters
                if (i < 3)
                    current_position.translation[i] = trans.Translation[i] / 1000.0;
                current_position.rotation[i] = rot.Rotation[i];
            }
            current_position.receive_time = current_time;
            current_position.segment_name = segment_name;
            current_position.subject_name = subject_name;
            current_position.translation_type = "Global";
            current_position.frame_number = frame_number;

            // send position to publisher
            auto queue_map_it = queue_map.find(subject_name + "/" + segment_name);
            if (queue_map_it != queue_map.end())
            {
                // send position to correct queue
                queue_map_it->second->add(current_position);
            }
            else
            {
                create_publisher(subject_name, segment_name);
            }
        }
    }
}


void Communicator::create_publisher(const string subject_name, const string segment_name)
{
    std::string topic_name = ns_name + "/" + subject_name + "/" + segment_name;
    std::string key = subject_name + "/" + segment_name;

    RCLCPP_INFO(this->get_logger(), "Creating publisher for segment %s from subject %s", segment_name.c_str(), subject_name.c_str());

    auto queue = std::make_shared<BlockingCollection<PositionStruct>>(100);

    // create publisher
    queue_map.insert(std::map<std::string, shared_ptr<BlockingCollection<PositionStruct>>>::value_type(key, queue));

    boost::thread(&Communicator::queue_monitor, this, subject_name, topic_name, queue);
}

// Also includes queue to monitor
void Communicator::queue_monitor(const string subject_name, const string topic_name, shared_ptr<BlockingCollection<PositionStruct>> queue) {
    Publisher publisher(subject_name, topic_name, this);

    while (true) {

        PositionStruct data;
      
        // take will block if there is no data to be taken
        auto status = queue->take(data);
        
        if(status == BlockingCollectionStatus::Ok)
        {
            // RCLCPP_INFO(this->get_logger(), "Processing data");
            publisher.publish(data);
        }
        
        // Status can also return BlockingCollectionStatus::Completed meaning take was called 
        // on a completed collection. Some other thread can call complete_adding after we pass 
        // the is_completed check but before we call take. In this example, we can ignore that
        // status since the loop will break on the next iteration.
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<Communicator>();
    // executor.add_node(node);
    
    while (rclcpp::ok()){
        node->connect();
        node->get_frame();
        // executor.spin_once();
    }

    // node->disconnect();
    rclcpp::shutdown();
    return 0;
}