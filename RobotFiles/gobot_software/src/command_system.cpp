#include "command_system.hpp"

const int max_length = 1024;

bool waiting = false;
bool connected = false;
ros::ServiceClient startRobotPosClient;
ros::ServiceClient stopRobotPosClient;

ros::ServiceClient startMetadataClient;
ros::ServiceClient stopMetadataClient;

ros::ServiceClient startMapClient;
ros::ServiceClient stopMapClient;

ros::ServiceClient playPathClient;
ros::ServiceClient pausePathClient;
ros::ServiceClient stopPathClient;

ros::Publisher go_pub;

int metadata_port = 4000;
int robot_pos_port = 4001;
int map_port = 4002;

std::string path_computer_software = "/home/gtdollar/computer_software/";

static const boost::regex cmd_regex("\"(.*?)\"");

bool execCommand(ros::NodeHandle n, std::vector<std::string> command){
	std::string commandStr = command.at(0);
	switch (commandStr.at(0)) {

		/// Command for changing the name of the robot
		case 'a':
			if(command.size() > 1){
				std::cout << "(Command system) New name : " << command.at(1) << std::endl;

				std::ofstream ofs;
				ofs.open(path_computer_software + "Robot_Infos/name.txt", std::ofstream::out | std::ofstream::trunc);
				ofs << command.at(1);
				ofs.close();

				return true;
			} else {
				std::cout << "(Command system) Name missing" << std::endl;
			}
		break;

		/// Command for changing the wifi of the robot
		case 'b':
			if(command.size() > 2){
				std::cout << "(Command system) New wifi : " << command.at(1) << std::endl;
				std::cout << "(Command system) New wifi password : " << command.at(2) << std::endl;
				std::string cmd = "sudo bash " + path_computer_software + "change_wifi.sh \"" + command.at(1) + "\" \""+ command.at(2) + "\"";
				std::cout << "(Command system) Cmd : " << cmd << std::endl;

				system(cmd.c_str());
				return true;
			} else {
				std::cout << "(Command system) Parameter missing" << std::endl;
			}
		break;

		/// Command for the robot to move to a point
		case 'c':
			std::cout << "(Command system) Gobot go to point" << std::endl;
			if(command.size() > 3){
				float posX = std::stof(command.at(1));
				float posY = std::stof(command.at(2));
				int waitTime = std::stoi(command.at(3));

//rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id: "map" }, pose: { position: { x: 2.7, y: -1.5, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'

				geometry_msgs::PoseStamped msg;
				msg.header.frame_id = "map";
				msg.header.stamp = ros::Time::now();
				msg.pose.position.x = posX;
				msg.pose.position.y = posY;
				msg.pose.position.z = 0;
				msg.pose.orientation.x = 0;
				msg.pose.orientation.y = 0;
				msg.pose.orientation.z = 0;
				msg.pose.orientation.w = 1;
				
				go_pub.publish(msg);
			
				if(waitTime >= 0){
					std::cout << "(Command system) Then wait for : " << waitTime << std::endl;
				} else {
					std::cout << "(Command system) Then wait for : Human Action" << std::endl;
				}
				return true;
			} else {
				std::cout << "(Command system) Parameter missing" << std::endl;
			}
		break;

		/// Command for the robot to pause the path
		case 'd':{
			std::cout << "(Command system) Pausing the path" << std::endl;
			std_srvs::Empty arg;
			if(ros::service::call("pause_path", arg))
				std::cout << "Pause path service called with success";
			else
				std::cout << "Pause path service call failed";
			return true;
		}
		break;

		/// Command for the robot to start to scan the map
		case 'e':
			std::cout << "(Command system) Gobot scan the map" << std::endl;
			startMap(n);
			return true;

		break;

		/// Command for the robot to stop to scan the map
		case 'f':
			std::cout << "(Command system) Gobot stop scanning the map" << std::endl;
			stopMap();
			return true;
		break;

		/// Command for the robot to receive the map
		case 'g':
			std::cout << "(Command system) Gobot here is the map" << std::endl;
			return true;
		break;

		/// Command for the robot to receive the ports needed for the map, metadata and robot pos services
		case 'h':
			if(command.size() > 3){

				metadata_port = std::stoi(command.at(1));
				robot_pos_port = std::stoi(command.at(2));
				map_port = std::stoi(command.at(3));
				std::cout << "(Command system) Gobot here are the ports " << metadata_port << ", " << robot_pos_port << ", " << map_port << std::endl;
				
				return true;
			} else {
				std::cout << "(Command system) Parameter missing" << std::endl;
			}
		break;

		/// Command for the robot to save a new path
		case 'i':
			if(command.size() >= 4 && command.size()%3 == 1){

				std::cout << "(Command system) Path received :" << std::endl;

				std::ofstream ofs(path_computer_software + "Robot_Infos/path.txt", std::ofstream::out | std::ofstream::trunc);
				
				if(ofs){

					for(int i = 1; i < command.size(); i+=3)
						ofs << command.at(i) << " " << command.at(i+1) << " " << command.at(i+2) << "\n";

					ofs.close();

				} else std::cout << "sorry could nt open the file " << path_computer_software + "Robot_Infos/path.txt";

				return true;

			} else 
				std::cout << "(Command system) Parameter missing" << std::endl;
			
		break;

		/// Command for the robot to play the saved path
		case 'j':{
			std::cout << "(Command system) Playing the path" << std::endl;
			std_srvs::Empty arg;
			if(ros::service::call("play_path", arg))
				std::cout << "Play path service called with success";
			else
				std::cout << "Play path service call failed";
			return true;
		}
		break;

		/// Command for the robot to delete the saved path
		case 'k':
			{
				std::cout << "(Command system) Deleting the path" << std::endl;
				std::ofstream ofs;
				ofs.open(path_computer_software + "Robot_Infos/path.txt", std::ofstream::out | std::ofstream::trunc);
				ofs.close();
				return true;
			}
		break;

		/// Command to stop the robot while following its path
		case 'l':{
			std::cout << "(Command system) Stopping the path" << std::endl;
			std_srvs::Empty arg;
			if(ros::service::call("stop_path", arg))
				std::cout << "Stop path service called with success";
			else
				std::cout << "Stop path service call failed";
			std::cout << "command system returning true after cmd l";
			std::cout << "(Command system) Stopping the path" << std::endl;
			return true;
		}
		break;

		/// Default/Unknown command
		default:
			std::cerr << "(Command system) Unknown command '" << command.at(0) << "' with " << command.size()-1 << " arguments : ";
			if(command.size() < 10){
				for(int i = 0; i < command.size(); i++){
					std::cerr << command.at(i) << " ";
				}
			} else {
				std::cout << "(Command system) Too many arguments to display (" << command.size() << ")" << std::endl;
			}
			std::cout << std::endl;
		break;
	}
	return false;
}

void startRobotPos(ros::NodeHandle n){
	std::cout << "(Command system) Launching the service to get the robot position" << std::endl;

	gobot_software::Port srv;
	srv.request.port = robot_pos_port;

	if (startRobotPosClient.call(srv)) {
		std::cout << "(Command system) start_robot_pos_sender service started" << std::endl;
	} else {
		std::cerr << "(Command system) Failed to call service start_robot_pos_sender" << std::endl;
	}
}

void stopRobotPos(){
	gobot_software::Port srv;

	if (stopRobotPosClient.call(srv)) {
		std::cout << "(Command system) stop_robot_pos_sender service started" << std::endl;
	} else {
		std::cerr << "(Command system) Failed to call service stop_robot_pos_sender" << std::endl;
	}
}

void startMetadata(ros::NodeHandle n){
	std::cout << "(Command system) Launching the service to get the metadata" << std::endl;

	gobot_software::Port srv;
	srv.request.port = metadata_port;

	if (startMetadataClient.call(srv)) {
		std::cout << "(Command system) start_map_metadata_sender service started" << std::endl;
	} else {
		std::cerr << "(Command system) Failed to call service start_map_metadata_sender" << std::endl;
	}
}

void stopMetadata(){
	gobot_software::Port srv;

	if (stopMetadataClient.call(srv)) 
		std::cout << "(Command system) stop_map_metadata_sender service started" << std::endl;
	else 
		std::cerr << "(Command system) Failed to call service stop_map_metadata_sender" << std::endl;
	
}

void startMap(ros::NodeHandle n){
	std::cout << "(Command system) Launching the service to get the map" << std::endl;

	gobot_software::Port srv;
	srv.request.port = map_port;

	if (startMapClient.call(srv)) {
		std::cout << "(Command system) start_map_sender service started" << std::endl;
	} else {
		std::cerr << "(Command system) Failed to call service start_map_sender" << std::endl;
	}
}

void stopMap(){
	gobot_software::Port srv;

	if (stopMapClient.call(srv)) {
		std::cout << "(Command system) stop_map_sender service started" << std::endl;
	} else {
		std::cerr << "(Command system) Failed to call service stop_map_sender" << std::endl;
	}
}

void getPorts(boost::shared_ptr<tcp::socket> sock, ros::NodeHandle n){

		std::cout << "getPorts launched" << std::endl;
		std::vector<std::string> command;
		std::string commandStr = "";
		char data[max_length];
		bool finishedCmd = 0;

		boost::system::error_code error;
		size_t length = sock->read_some(boost::asio::buffer(data), error);
		std::cout << length << " byte(s) received" << std::endl;
		if ((error == boost::asio::error::eof) || (error == boost::asio::error::connection_reset)){
			std::cout << "(Command system) Connection closed" << std::endl;
			connected = false;
			return;
    	} else if (error) {
			throw boost::system::system_error(error); // Some other error.
    	}

		std::istringstream iss(data);

		while (iss && !finishedCmd && ros::ok() && connected){
			std::string sub;
			iss >> sub;
			if(sub.compare("}") == 0){
				std::cout << "(Command system) Command complete" << std::endl;
				finishedCmd = 1;
			} else {
				commandStr += sub + " ";
			}
		}
		std::cout << "Received :" << commandStr << std::endl;

		command.push_back(std::string(1, commandStr.at(0)));

		std::list<std::string> l;
		boost::regex_split(std::back_inserter(l), commandStr, cmd_regex);
		while(l.size()) {
			std::string s = *(l.begin());
			l.pop_front();
			command.push_back(s);
		}

		if(finishedCmd){
			std::cout << "(Command system) Executing command : " << std::endl;
			if(command.size() < 10){
				for(int i = 0; i < command.size(); i++){
					std::cout << "'" << command.at(i) << "'" << std::endl;
				}
			} else {
				std::cout << "(Command system) Too many arguments to display (" << command.size() << ")" << std::endl;
			}

			execCommand(n, command);
			std::cout << "getPorts done" << std::endl;
		
		}
}

void session(boost::shared_ptr<tcp::socket> sock, ros::NodeHandle n){
	std::cout << "(Command system) Waiting for a command" << std::endl;
	try{
		std::vector<std::string> command;
		std::string commandStr = "";
		char data[max_length];
		bool finishedCmd = 0;

		getPorts(sock, n);

		startRobotPos(n);
		startMetadata(n);

		while(ros::ok() && connected){
			char data[max_length];

			boost::system::error_code error;
			size_t length = sock->read_some(boost::asio::buffer(data), error);
			std::cout << length << " byte(s) received" << std::endl;
			if ((error == boost::asio::error::eof) || (error == boost::asio::error::connection_reset)){
				std::cout << "(Command system) Connection closed" << std::endl;
				connected = false;
				return;
        	} else if (error) {
				throw boost::system::system_error(error); // Some other error.
        	}

			std::istringstream iss(data);

			while (iss && !finishedCmd && ros::ok() && connected){
				std::string sub;
				iss >> sub;
				if(sub.compare("}") == 0){
					std::cout << "(Command system) Command complete" << std::endl;
					finishedCmd = 1;
				} else {
					commandStr += sub + " ";
				}
			}

			command.push_back(std::string(1, commandStr.at(0)));

   			std::list<std::string> l;
			boost::regex_split(std::back_inserter(l), commandStr, cmd_regex);
			while(l.size()) {
				std::string s = *(l.begin());
				l.pop_front();
				command.push_back(s);
			}

			if(finishedCmd){
				std::cout << "(Command system) Executing command : " << std::endl;
				if(command.size() < 10){
					for(int i = 0; i < command.size(); i++){
						std::cout << "'" << command.at(i) << "'" << std::endl;
					}
				} else {
					std::cout << "(Command system) Too many arguments to display (" << command.size() << ")" << std::endl;
				}

				std::string msg = command.at(0);
				if(execCommand(n, command)){
					msg += " done";
				} else {
					msg += " failed";
				}
				sendMessageToPc(sock, msg);
				command.clear();
				finishedCmd = 0;
				commandStr = "";
			}
		}
	} catch (std::exception& e) {
		std::cerr << "(Command system) Exception in thread: " << e.what() << "\n";
	}
}

bool sendMessageToPc(boost::shared_ptr<tcp::socket> sock, std::string message){
	std::cout << "(Command system) Sending message : " << message << std::endl;

	try {
		boost::system::error_code ignored_error;
		boost::asio::write(*sock, boost::asio::buffer(message, message.length()), boost::asio::transfer_all(), ignored_error);
		
		std::cout << "(Command system) Message sent succesfully" << std::endl;
		return true;
	} catch (std::exception& e) {
		std::cerr << "(Command system) Message not sent" << std::endl;
		std::cerr << e.what() << std::endl;
		return false;
	}
}

void asyncAccept(boost::shared_ptr<boost::asio::io_service> io_service, boost::shared_ptr<tcp::acceptor> m_acceptor, ros::NodeHandle n){
	std::cout << "(Command system) Waiting for connection" << std::endl;

	boost::shared_ptr<tcp::socket> sock = boost::shared_ptr<tcp::socket>(new tcp::socket(*io_service));

	m_acceptor->accept(*sock);
	std::cout << "(Command system) Command socket connected to " << sock->remote_endpoint().address().to_string() << std::endl;
	connected = true;
	waiting = false;
	boost::thread t(boost::bind(session, sock, n));

	/// Send a message to the PC to tell we are connected
	sendMessageToPc(sock, "Connected");
}

void server(unsigned short port, ros::NodeHandle n){

	boost::shared_ptr<boost::asio::io_service> io_service = boost::shared_ptr<boost::asio::io_service>(new boost::asio::io_service());
	io_service->run();

	boost::shared_ptr<tcp::endpoint> m_endpoint = boost::shared_ptr<tcp::endpoint>(new tcp::endpoint(tcp::v4(), port));
	boost::shared_ptr<tcp::acceptor> m_acceptor = boost::shared_ptr<tcp::acceptor>(new tcp::acceptor(*io_service, *m_endpoint));

	m_acceptor->set_option(tcp::acceptor::reuse_address(true));

	while(ros::ok()){
		if(!connected && !waiting){
			std::cout << "(Command system) Ready to connect" << std::endl;
			boost::thread t(boost::bind(asyncAccept, io_service, m_acceptor, n));

			waiting = true;
		}
		ros::spinOnce();
	}
}

void serverDisconnected(const std_msgs::String::ConstPtr& msg){
	std::cout << "(Command system) I heard " << std::endl;
	connected = false;
	stopRobotPos();
	stopMetadata();
}


int main(int argc, char* argv[]){

	try{
		ros::init(argc, argv, "command_system");
		ros::NodeHandle n;
  		ros::Subscriber sub = n.subscribe("server_disconnected", 1000, serverDisconnected);
		
		startRobotPosClient = n.serviceClient<gobot_software::Port>("start_robot_pos_sender");
		stopRobotPosClient = n.serviceClient<gobot_software::Port>("stop_robot_pos_sender");
		
		startMetadataClient = n.serviceClient<gobot_software::Port>("start_map_metadata_sender");
		stopMetadataClient = n.serviceClient<gobot_software::Port>("stop_map_metadata_sender");
		
		startMapClient = n.serviceClient<gobot_software::Port>("start_map_sender");
		stopMapClient = n.serviceClient<gobot_software::Port>("stop_map_sender");

		playPathClient = n.serviceClient<std_srvs::Empty>("play_path");
		pausePathClient = n.serviceClient<std_srvs::Empty>("pause_path");
		stopPathClient = n.serviceClient<std_srvs::Empty>("stop_path");

		go_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);

		server(CMD_PORT, n);
		ros::spin();
	} catch (std::exception& e) {
		std::cerr << "(Command system) Exception: " << e.what() << std::endl;
	}

	return 0;
}
