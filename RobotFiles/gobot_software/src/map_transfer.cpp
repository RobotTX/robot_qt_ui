#include "map_transfer.hpp"

using boost::asio::ip::tcp;

boost::asio::io_service io_service;
tcp::socket socket_map(io_service);
ros::Subscriber sub_map;
tcp::acceptor m_acceptor(io_service);

#define HIGH_THRESHOLD 0.65*100
#define LOW_THRESHOLD 0.196*100

std::string path_computer_software = "/home/gtdollar/computer_software/";
std::string path_gobot_move = "/home/gtdollar/catkin_ws/src/gobot_move/";

void sendMap(const std::vector<uint8_t>& my_map){
	try {
		boost::system::error_code ignored_error;
		std::cout << "(Map) Map size to send in uint8_t : " << my_map.size() << std::endl;
		boost::asio::write(socket_map, boost::asio::buffer(my_map), boost::asio::transfer_all(), ignored_error);
	} catch (std::exception& e) {
		e.what();
	}
}

void sendMap(const std::vector<int8_t>& my_map){
	try {
		boost::system::error_code ignored_error;
		std::cout << "(Map) Last map size to send in int8_t : " << my_map.size() << std::endl;

		boost::asio::write(socket_map, boost::asio::buffer(my_map), boost::asio::transfer_all(), ignored_error);
	} catch (std::exception& e) {
		e.what();
	}
}

void getMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	int map_size = msg->info.width * msg->info.height;
	std::cout << "(Map) Just received a new map" << std::endl;

	sendMap(compress(msg->data, map_size, false));
}

std::vector<uint8_t> compress(std::vector<int8_t> map, int map_size, bool fromPgm){
	std::vector<uint8_t> my_map;
	int last = 205;
	uint32_t count = 0;

	/// If the map comes from a pgm, we send the mapId and mapDate with it
	if(fromPgm){
	   	std::ifstream ifMap(path_computer_software + "Robot_Infos/mapId.txt", std::ifstream::in);
	   	std::string mapId("{0}");
	   	std::string mapDate("0");
	   	if(ifMap){
	   		getline(ifMap, mapId);
	   		getline(ifMap, mapDate);
	   		ifMap.close();
	   	}

	   	std::string str = mapId + " " + mapDate;
	   	for(int i = 0; i < str.size(); i++){
	   		my_map.push_back(static_cast<uint8_t>(str.at(i)));
	   	}

		my_map.push_back(252);
		my_map.push_back(252);
		my_map.push_back(252);
		my_map.push_back(252);
		my_map.push_back(252);
	}


	for(size_t i = 0; i < map_size; i++){
		int curr = map.at(i);

		if(!fromPgm){
		    if(curr < 0)
	            curr = 205;
	        else if(curr < LOW_THRESHOLD)
	            curr = 255;
	        else if(curr < HIGH_THRESHOLD)
	            curr = 205;
	        else 
	            curr = 0;
	    }

		if(curr != last){
			my_map.push_back(static_cast<uint8_t>(last));
			my_map.push_back((count & 0xff000000) >> 24);
			my_map.push_back((count & 0x00ff0000) >> 16);
			my_map.push_back((count & 0x0000ff00) >> 8);
			my_map.push_back((count & 0x000000ff));

			last = curr;
			count = 0;
		}
		count++;
	}

	my_map.push_back(static_cast<uint8_t>(last));
	my_map.push_back((count & 0xff000000) >> 24);
	my_map.push_back((count & 0x00ff0000) >> 16);
	my_map.push_back((count & 0x0000ff00) >> 8);
	my_map.push_back((count & 0x000000ff));

	// the user knows that when 254 is encountered a map has entirely been received
	my_map.push_back(254);
	my_map.push_back(254);
	my_map.push_back(254);
	my_map.push_back(254);

	my_map.push_back((fromPgm) ? 254 : 253);

	return my_map;
}

bool startMap(gobot_software::Port::Request &req,
    gobot_software::Port::Response &res){
	std::cout << "(Map) Starting map_sender" << std::endl;

	int mapPort = req.port;	

	if(socket_map.is_open())
		socket_map.close();

	if(m_acceptor.is_open())
		m_acceptor.close();

	socket_map = tcp::socket(io_service);
	m_acceptor = tcp::acceptor(io_service, tcp::endpoint(tcp::v4(), mapPort));
	m_acceptor.set_option(tcp::acceptor::reuse_address(true));

	std::cout << "(Map) Connecting to ports : " << mapPort << std::endl;
	m_acceptor.accept(socket_map);
	std::cout << "(Map) We are connected " << std::endl;

	return true;
}

bool sendAutoMap(std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res){
	std::cout << "(Map) SendAutoMap " << std::endl;

	ros::NodeHandle n;
	sub_map.shutdown();
	sub_map = n.subscribe("/map", 1, getMap);

	return true;
}

bool stopAutoMap(std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res){
	std::cout << "(Map) StopAutoMap " << std::endl;

	sub_map.shutdown();

	return true;
}

bool sendOnceMap(std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res){
	std::cout << "(Map) SendOnceMap doing nothing for now" << std::endl;

	std::vector<int8_t> my_map;
    std::string mapFileStr = path_gobot_move + "maps/new_map.pgm";

	std::string line;
	std::ifstream mapFile;

	mapFile.open(mapFileStr);
	if(mapFile.is_open()){
		getline(mapFile, line);
		std::cout << "(Map) 1 : " << line << std::endl;

		getline(mapFile, line);
		std::cout << "(Map) 2 : " << line << std::endl;
		int width = std::stoi(line.substr(0,line.find_first_of(" ")));
		int height = std::stoi(line);
		int map_size = width * height;
		std::cout << "(Map) width : " << width << "\n(Map) height : " << height << "\n(Map) size : " << map_size << std::endl;

		getline(mapFile, line);
		std::cout << "(Map) 3 : " << line << std::endl;

		while(getline(mapFile, line)){
			for(int i = 0; i < line.size(); i++){
				my_map.push_back(static_cast<int8_t>(line.at(i)));
			}
		}

		mapFile.close();
		std::cout << "(Map) Got the whole map from file, about to compress and send it" << std::endl;
		sendMap(compress(my_map, map_size, true));

		return true;
		
	} else
		return false;
}

bool stopMap(std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res){

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
	ros::ServiceServer send_once_service = n.advertiseService("send_once_map_sender", sendOnceMap);
	ros::ServiceServer send_auto_service = n.advertiseService("send_auto_map_sender", sendAutoMap);
	ros::ServiceServer stop_auto_service = n.advertiseService("stop_auto_map_sender", stopAutoMap);
	ros::ServiceServer stop_service = n.advertiseService("stop_map_sender", stopMap);

	ros::Rate loop_rate(20);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
