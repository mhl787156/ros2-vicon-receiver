#include "vicon_receiver/communicator.hpp"

using namespace ViconDataStreamSDK::CPP;

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
        bool ok = (vicon_client.Connect(hostname).Result == Result::Success);
        if (!ok)
        {
            counter++;
            RCLCPP_WARN(this->get_logger(), "Connect failed, reconnecting ( %d )...", counter);
            mySleep(1);
        }
    }
    RCLCPP_INFO(this->get_logger(),  "Connection successfully established with %s", hostname.c_str());

    // perform further initialization
    vicon_client.EnableSegmentData();
    vicon_client.EnableMarkerData();
    vicon_client.EnableUnlabeledMarkerData();
    vicon_client.EnableMarkerRayData();
    vicon_client.EnableDeviceData();
    vicon_client.EnableDebugData();

    vicon_client.SetStreamMode(StreamMode::ClientPull);
    vicon_client.SetBufferSize(buffer_size);

    RCLCPP_INFO(this->get_logger(), "Initialisation complete");

    return true;
}

bool Communicator::disconnect()
{
    if (!vicon_client.IsConnected().Connected)
        return true;
    mySleep(1);
    vicon_client.DisableSegmentData();
    vicon_client.DisableMarkerData();
    vicon_client.DisableUnlabeledMarkerData();
    vicon_client.DisableDeviceData();
    vicon_client.DisableCentroidData();
    RCLCPP_INFO(this->get_logger(), "Disconnecting from %s ...", hostname.c_str());
    vicon_client.Disconnect();
    RCLCPP_INFO(this->get_logger(), "Successfully disconnected");
    if (!vicon_client.IsConnected().Connected)
        return true;
    return false;
}

void Communicator::get_frame()
{
    vicon_client.GetFrame();
    Output_GetFrameNumber frame_number = vicon_client.GetFrameNumber();

    unsigned int subject_count = vicon_client.GetSubjectCount().SubjectCount;

    map<string, Publisher>::iterator pub_it;

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
            current_position.frame_number = frame_number.FrameNumber;

            // send position to publisher
            boost::mutex::scoped_try_lock lock(mutex);

            if (lock.owns_lock())
            {
                // get publisher
                pub_it = pub_map.find(subject_name + "/" + segment_name);
                if (pub_it != pub_map.end())
                {
                    Publisher & pub = pub_it->second;

                    if (pub.is_ready)
                    {
                        pub.publish(current_position);
                    }
                }
                else
                {
                    // create publisher if not already available
                    lock.unlock();
                    create_publisher(subject_name, segment_name);
                }
            }
        }
    }
}

void Communicator::create_publisher(const string subject_name, const string segment_name)
{
    boost::thread(&Communicator::create_publisher_thread, this, subject_name, segment_name);
}

void Communicator::create_publisher_thread(const string subject_name, const string segment_name)
{
    std::string topic_name = ns_name + "/" + subject_name + "/" + segment_name;
    std::string key = subject_name + "/" + segment_name;

    RCLCPP_INFO(this->get_logger(), "Creating publisher for segment %s from subject %s", segment_name.c_str(), subject_name.c_str());

    // create publisher
    boost::mutex::scoped_lock lock(mutex);
    pub_map.insert(std::map<std::string, Publisher>::value_type(key, Publisher(subject_name, topic_name, this)));

    // we don't need the lock anymore, since rest is protected by is_ready
    lock.unlock();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Communicator>();
    
    while (rclcpp::ok()){
        node->connect();
        node->get_frame();
    }

    // node->disconnect();
    rclcpp::shutdown();
    return 0;
}