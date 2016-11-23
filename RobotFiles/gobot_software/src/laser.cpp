#include "laser.hpp"

using boost::asio::ip::tcp;

boost::asio::io_service io_service;
tcp::socket socket_laser(io_service);
ros::Subscriber sub_laser;
tcp::acceptor l_acceptor(io_service);

bool startLaser(gobot_software::Port::Request &req, gobot_software::Port::Response &res){
    std::cout << "(Laser) Starting laser_sender" << std::endl;
    ros::NodeHandle n;

    int laserPort = req.port; 

    socket_laser = tcp::socket(io_service);
    l_acceptor = tcp::acceptor(io_service, tcp::endpoint(tcp::v4(), laserPort));
    l_acceptor.set_option(tcp::acceptor::reuse_address(true));

    std::cout << "(Laser) Connecting to ports : " << laserPort << std::endl;
    l_acceptor.accept(socket_laser);
    std::cout << "(Laser) We are connected " << std::endl;

    sub_laser = n.subscribe("/scan", 1, getLaserData);

    return true;
}

void getLaserData(const sensor_msgs::LaserScan::ConstPtr& msg){
    std::vector<float> scan;
    //std::cout << "received a laser scan !" << std::endl;
    scan.push_back(msg->angle_min);
    scan.push_back(msg->angle_max);
    scan.push_back(msg->angle_increment);
    for(int i = 0; i < (msg->angle_max - msg->angle_min) / msg->angle_increment; i++)
        scan.push_back(msg->ranges[i]);
    scan.push_back(-1.0f);
    sendLaserData(scan);
}

void sendLaserData(const std::vector<float>& scan){
    try { 
        boost::system::error_code ignored_error;
        //std::cout << "(Laser) LaserData " << scan.size() * 4 << std::endl;
        boost::asio::write(socket_laser, boost::asio::buffer(scan), boost::asio::transfer_all(), ignored_error);
    } catch (std::exception& e) {
        e.what();
    }
}

bool stopSendingLaserData(gobot_software::Port::Request &req, gobot_software::Port::Response &res){
    std::cout << "(Laser) Stopping laser_sender" << std::endl;
    sub_laser.shutdown();
    socket_laser.close();
    l_acceptor.close();
    return true;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "laser_data_transfer");
    std::cout << "(Laser) Ready to be launched." << std::endl;

    ros::NodeHandle n;

    ros::ServiceServer start_service = n.advertiseService("start_laser_data_sender", startLaser);
    ros::ServiceServer stop_service = n.advertiseService("stop_laser_data_sender", stopSendingLaserData);

    ros::Rate r(1);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}