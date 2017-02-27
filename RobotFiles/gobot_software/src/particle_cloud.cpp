#include "particle_cloud.hpp"

using boost::asio::ip::tcp;

boost::asio::io_service io_service;
tcp::socket socket_particle_cloud(io_service);
tcp::acceptor m_acceptor(io_service);

// to subscribe to the /particle_cloud topic
ros::Subscriber sub_particle_cloud;

// Connects this node to the application opening a socket on a dedicated port
bool connectParticleCloud(gobot_software::Port::Request &req, gobot_software::Port::Response &res){
    std::cout << "(Particle cloud) Connecting the particle cloud node " << req.port << std::endl;

    int port = req.port; 

    // resets the state of the socket
    if(socket_particle_cloud.is_open())
        socket_particle_cloud.close();

    if(m_acceptor.is_open())
        m_acceptor.close();

    socket_particle_cloud = tcp::socket(io_service);
    m_acceptor = tcp::acceptor(io_service, tcp::endpoint(tcp::v4(), port));
    m_acceptor.set_option(tcp::acceptor::reuse_address(true));

    m_acceptor.accept(socket_particle_cloud);

    std::cout << "(Particle cloud node) We are connected " << std::endl;

    return true;
}

// the service to subscribe to the /particle_cloud topic in order to start sending the data
bool sendParticleCloudService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::cout << "(Particle cloud node) sendParticleCloudService " << std::endl;
    ros::NodeHandle n;
    /// in case sub_particle_cloud would have subscribed to another topic before we unsubscribe first
    sub_particle_cloud.shutdown();
    sub_particle_cloud = n.subscribe("/particlecloud", 1, getParticleCloud);
    return true;
}

// Gets the particle cloud from the topic /particle_cloud, the different with what we send is that it also contains a timestamp 
// which we don't bother sending
void getParticleCloud(const geometry_msgs::PoseArray& particle_cloud){
    std::cout << "(Particle cloud node) received a particle_cloud of size " << particle_cloud.poses.size() << std::endl;
    std::vector<Position> points;
    for(size_t i = 0; i < particle_cloud.poses.size(); i++)
        points.push_back({static_cast<double> (particle_cloud.poses.at(i).position.x), 
                          static_cast<double> (particle_cloud.poses.at(i).position.y) });
    sendParticleCloud(points);
}

void sendParticleCloud(const std::vector<Position>& particle_cloud){
    // Send the particle cloud to the application as an array of Point
    try {
        boost::system::error_code ignored_error;
        std::cout << "(Particle cloud node) Particle cloud size to send" << particle_cloud.size() << std::endl;
        boost::asio::write(socket_particle_cloud, boost::asio::buffer(particle_cloud), boost::asio::transfer_all(), ignored_error);
    } catch (std::exception& e) {
        e.what();
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "particle_cloud_transfer");
    std::cout << "(Particle cloud node) Ready to be launched." << std::endl;

    ros::NodeHandle n;

    ros::ServiceServer connect_service = n.advertiseService("connect_particle_cloud", connectParticleCloud);

    ros::ServiceServer send_data_service = n.advertiseService("send_particle_cloud_data", sendParticleCloudService);

    ros::Rate loop_rate(20);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}