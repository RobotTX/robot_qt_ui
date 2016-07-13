#include "map_transfer.hpp"

using boost::asio::ip::tcp;

boost::asio::io_service io_service;
tcp::socket socket_map(io_service);
ros::Subscriber sub_map;
tcp::acceptor m_acceptor(io_service);


void sendMap(const std::vector<int8_t>& my_map){
	//std::cout << "(Map transfer) map :" << std::endl;
	try {
		boost::system::error_code ignored_error;
		std::cout << "Map size to send : " << my_map.size() << std::endl;

		boost::asio::write(socket_map, boost::asio::buffer(my_map), boost::asio::transfer_all(), ignored_error);
		//sleep(2);
	} catch (std::exception& e) {
		e.what();
	}
}

void getMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	std::vector<int8_t> my_map;
	for(size_t i = 0; i < msg->info.width * msg->info.height; i++)
		my_map.push_back(msg->data[i]);
	// the user knows that when -2 is encountered a map has entirely been received
	my_map.push_back(-2);
	sendMap(my_map);
}

bool startMap(gobot_software::Port::Request &req,
    gobot_software::Port::Response &res){
	std::cout << "(Map) Starting map_sender" << std::endl;
	ros::NodeHandle n;

	int mapPort = req.port;	


	socket_map = tcp::socket(io_service);
	m_acceptor = tcp::acceptor(io_service, tcp::endpoint(tcp::v4(), mapPort));
	m_acceptor.set_option(tcp::acceptor::reuse_address(true));

	std::cout << "(Map) Connecting to ports : " << mapPort << std::endl;
	m_acceptor.accept(socket_map);
	std::cout << "(Map) We are connected " << std::endl;

	sub_map = n.subscribe("/map", 1, getMap);

	return true;
}

bool stopMap(gobot_software::Port::Request &req,
    gobot_software::Port::Response &res){
	std::cout << "(Map) Stopping map_sender" << std::endl;

	sub_map.shutdown();
	socket_map.close();
	m_acceptor.close();

	return true;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "map_transfer");
	std::cout << "(Map) Ready to be launched." << std::endl;

	ros::NodeHandle n;

	ros::ServiceServer start_service = n.advertiseService("start_map_sender", startMap);
	ros::ServiceServer stop_service = n.advertiseService("stop_map_sender", stopMap);

	ros::spin();


	return 0;
}
