#include "map_transfer.hpp"

using boost::asio::ip::tcp;

boost::asio::io_service io_service;
tcp::socket socket_map(io_service);
ros::Subscriber sub_map;
tcp::acceptor m_acceptor(io_service);

bool lastMap = false;

#define HIGH_THRESHOLD 0.65*100
#define LOW_THRESHOLD 0.196*100

void sendMap(const std::vector<uint8_t>& my_map){
	try {
		boost::system::error_code ignored_error;
		std::cout << "(Map) Map size to send in uint8_t : " << my_map.size() << std::endl;

		boost::asio::write(socket_map, boost::asio::buffer(my_map), boost::asio::transfer_all(), ignored_error);
	} catch (std::exception& e) {
		e.what();
	}
}

void getMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	int map_size = msg->info.width * msg->info.height;
	std::cout << "(Map) Just received a new map" << std::endl;

	/// We want to send the last map as % so we can save it and the software can send it to the other robots
	if(lastMap){
		std::cout << "(Map) Sending the last map" << std::endl;
		std::vector<int8_t> my_map;

		for(size_t i = 0; i < map_size; i++){
			my_map.push_back((int8_t) msg->data[i]);
		}

		my_map.push_back(-3);
		sendMap(my_map);
		lastMap = false;

	} else {
		/// The map we send to the software for display
		std::vector<uint8_t> my_map;
		int last = 205;
		uint32_t count = 0;
		int index = 0;

		for(size_t i = 0; i < map_size; i++){
			int curr = msg->data[i];

		    if(curr < 0)
	            curr = 205;
	        else if(curr < LOW_THRESHOLD)
	            curr = 255;
	        else if(curr < HIGH_THRESHOLD)
	            curr = 205;
	        else 
	            curr = 0;

			if(curr != last){
				my_map.push_back((uint8_t) last);
				my_map.push_back((count & 0xff000000) >> 24);
				my_map.push_back((count & 0x00ff0000) >> 16);
				my_map.push_back((count & 0x0000ff00) >> 8);
				my_map.push_back((count & 0x000000ff));

				last = curr;
				count = 0;
			}
			count++;
			index++;
		}

		my_map.push_back((uint8_t) last);
		my_map.push_back((count & 0xff000000) >> 24);
		my_map.push_back((count & 0x00ff0000) >> 16);
		my_map.push_back((count & 0x0000ff00) >> 8);
		my_map.push_back((count & 0x000000ff));

		// the user knows that when -2 is encountered a map has entirely been received
		my_map.push_back(0);
		my_map.push_back(0);
		my_map.push_back(0);
		my_map.push_back(0);
		my_map.push_back(-2);
		sendMap(my_map);
	}
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

	std::cout << "(Map) Waiting to send the last map" << std::endl;
	lastMap = true;
	while(ros::ok() && lastMap){
		ros::spinOnce();
	}

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
