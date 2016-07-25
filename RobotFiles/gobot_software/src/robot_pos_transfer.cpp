#include "robot_pos_transfer.hpp"

using boost::asio::ip::tcp;

boost::asio::io_service io_service;
tcp::socket socket_robot(io_service);
ros::Subscriber sub_robot;
tcp::acceptor m_acceptor(io_service);


void sendRobotPos(const std::string& robot_string){
	try {
		boost::system::error_code ignored_error;
		boost::asio::write(socket_robot, boost::asio::buffer(robot_string), boost::asio::transfer_all(), ignored_error);
	} catch (std::exception& e) {
		std::cerr << e.what() << std::endl;
	}
}

void getRobotPos(const geometry_msgs::Pose::ConstPtr& msg){
	std::string robot_string = std::to_string(msg->position.x) + " " + std::to_string(msg->position.y) + " " + std::to_string(msg->orientation.z) + " ";
	sendRobotPos(robot_string);
	sleep(0.5);
}

bool startRobotPos(gobot_software::Port::Request &req,
    gobot_software::Port::Response &res){
	std::cout << "(Robot Pos) Starting robot_pos_sender" << std::endl;
	ros::NodeHandle n;

	int robotPort = req.port;	

	socket_robot = tcp::socket(io_service);
	m_acceptor = tcp::acceptor(io_service, tcp::endpoint(tcp::v4(), robotPort));
	m_acceptor.set_option(tcp::acceptor::reuse_address(true));

	std::cout << "(Robot Pos) Connecting to ports : " << robotPort << std::endl;
	m_acceptor.accept(socket_robot);
	std::cout << "(Robot Pos) We are connected " << std::endl;

	sub_robot = n.subscribe("/robot_pose", 1, getRobotPos);

	return true;
}

bool stopRobotPos(gobot_software::Port::Request &req,
    gobot_software::Port::Response &res){
	std::cout << "(Robot Pos) Stopping robot_pos_sender" << std::endl;

	sub_robot.shutdown();
	socket_robot.close();
	m_acceptor.close();

	return true;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "robot_pos_transfer");
	std::cout << "(Robot Pos) Ready to be launched." << std::endl;

	ros::NodeHandle n;

	ros::ServiceServer start_service = n.advertiseService("start_robot_pos_sender", startRobotPos);
	ros::ServiceServer stop_service = n.advertiseService("stop_robot_pos_sender", stopRobotPos);

	ros::spin();


	return 0;
}
