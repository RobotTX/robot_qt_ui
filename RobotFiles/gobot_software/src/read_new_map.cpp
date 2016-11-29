#include "read_new_map.hpp"

using boost::asio::ip::tcp;

const int max_length = 1024;
bool waiting = false;
bool connected = false;

boost::asio::io_service io_service;
tcp::socket socket_new_map(io_service);
tcp::acceptor m_acceptor(io_service);

std::string path_computer_software = "/home/gtdollar/computer_software/";
std::string path_gobot_move = "/home/gtdollar/catkin_ws/src/gobot_move/";
ros::Publisher map_pub;


void session(boost::shared_ptr<tcp::socket> sock, ros::NodeHandle n){
    std::cout << "(New Map) session launched" << std::endl;
    int gotMapData = 0;
    std::string mapId = "";
    std::string mapMetadata = "";
    std::string mapDate = "";
    std::vector<uint8_t> map;
    std::string message = "done";

    while(ros::ok() && connected){
        char data[max_length];

        boost::system::error_code error;
        size_t length = sock->read_some(boost::asio::buffer(data), error);
        if ((error == boost::asio::error::eof) || (error == boost::asio::error::connection_reset)){
            std::cout << "(New Map) Connection closed" << std::endl;
            connected = false;
            return;
        } else if (error) {
            throw boost::system::system_error(error); // Some other error.
            return;
        }


        /*std::cout << "(New Map) Last data :" << (int) data[length-5] << " " << (int) data[length-4] << " "
         << (int) data[length-3] << " " << (int) data[length-2] << " " << (int) data[length-1] << " compared to "
         << (uint8_t) -2 << " or " << -2 << std::endl;*/
       /// Parse the data as we are supposed to receive : "mapId ; mapDate ; metadata ; map"
        for(int i = 0; i < length; i++){
            if(data[i] == ';' && gotMapData <= 2){
                /// The first ; means we got the mapId
                /// the second means we got the metadata
                /// the third means we got the map date
                std::cout << "(New Map) ';' found" << std::endl;
                gotMapData++;
            } else {
                if(gotMapData > 2){
                    map.push_back((uint8_t) data[i]);
                } else if(gotMapData == 0){
                    mapId += data[i];
                } else if(gotMapData == 1){
                    mapDate += data[i];
                } else {
                    mapMetadata += data[i];
                }
            }
        }

        //std::cout << "(New Map) Size of the map received : " << map.size() << std::endl;

        if(map.size() > 4 && (int) map.at(map.size()-5) == 254 && (int) map.at(map.size()-4) == 254
             && (int) map.at(map.size()-3) == 254 && (int) map.at(map.size()-2) == 254 && (int) map.at(map.size()-1) == 254){
            std::cout << "(New Map) Last separator found" << std::endl;

            /// Save the id of the new map
            std::cout << "(New Map) Id of the new map : " << mapId << std::endl;
            std::cout << "(New Map) Date of the new map : " << mapDate << std::endl;
            std::ofstream ofs;
            ofs.open(path_computer_software + "Robot_Infos/mapId.txt", std::ofstream::out | std::ofstream::trunc);

            if(ofs.is_open()){
                ofs << mapId << std::endl << mapDate << std::endl;
                ofs.close();
                std::cout << "(New Map) Map id updated : " << mapId << " with date " << mapDate << std::endl;

                /// Set the medatada of the new map
                std::cout << "(New Map) Map metadata before split : " << mapMetadata << std::endl;
                int width = 0;
                int height = 0;
                float resolution = 0;
                float originX = 0;
                float originY = 0;

                std::istringstream iss(mapMetadata);
                iss >> width >> height >> resolution >> originX >> originY;
                std::cout << "(New Map) Map metadata after split : " << width << " " << height << " " << resolution << " " << originX << " " << originY << std::endl;

                /// We remove the 5 last byte as they are only there to identify the end of the map
                map.erase(map.end() - 5, map.end());
                std::cout << "(New Map) Size of the map received : " << map.size() << std::endl;


                /// We save the file in a the pgm file used by amcl
                std::string mapFile = path_gobot_move + "maps/new_map.pgm";
                ofs.open(mapFile, std::ofstream::out | std::ofstream::trunc);

                if(ofs.is_open()){
                    ofs << "P5" << std::endl << width << " " << height << std::endl << "255" << std::endl;

                    /// We set each pixel of the image, the data received being
                    for(int i = 0; i < map.size(); i+=5){
                        uint8_t color = static_cast<uint8_t> (map.at(i));

                        uint32_t count2 = static_cast<uint32_t> (static_cast<uint8_t> (map.at(i+1)) << 24) + static_cast<uint32_t> (static_cast<uint8_t> (map.at(i+2)) << 16)
                                        + static_cast<uint32_t> (static_cast<uint8_t> (map.at(i+3)) << 8) + static_cast<uint32_t> (static_cast<uint8_t> (map.at(i+4)));

                        for(int j = 0; j < (int) count2; j++){
                            ofs << color;
                        }
                    }
                    ofs << std::endl;
                    ofs.close();

                    std::cout << "(New Map) New map pgm file created in " << mapFile << std::endl;

                    /// Kill gobot move so that we'll restart it with the new map
                    std::string cmd = "rosnode kill /move_base";
                    system(cmd.c_str());

                    sleep(5);
                    std::cout << "(New Map) We killed gobot_move" << std::endl;

                    /// We delete the old path
                    ofs.open(path_computer_software + "Robot_Infos/path.txt", std::ofstream::out | std::ofstream::trunc);
                    ofs.close();
                    std::cout << "(New Map) Path deleted" << std::endl;

                    /// We delete the old home
                    ofs.open(path_computer_software + "Robot_Infos/home.txt", std::ofstream::out | std::ofstream::trunc);
                    ofs.close();
                    std::cout << "(New Map) Home deleted" << std::endl;

                    /// Relaunch gobot_move
                    cmd = "roslaunch gobot_move slam.launch &";
                    system(cmd.c_str());
                    std::cout << "(New Map) We relaunched gobot_move" << std::endl;

                } else {
                    std::cout << "(New Map) Could not open the file to create a new pgm file " << mapFile << std::endl;
                    message = "failed";
                }
            } else {
                std::cout << "(New Map) Map id could not be updated : " << mapId << " with date " << mapDate << std::endl;
                message = "failed";
            }

            /// Clear the used variables
            gotMapData = 0;
            mapId = "";
            mapMetadata = "";
            map.clear();


            /// Send a message to the software to tell we finished
            boost::asio::write(*sock, boost::asio::buffer(message, message.length()), boost::asio::transfer_all(), error);

            if(error) {
                std::cout << "(New Map) Error : " << error.message() << std::endl;
            } else {
                std::cout << "(New Map) Message sent succesfully : " << message.length() << " bytes sent" << std::endl;
            }
        }
    }
}

void asyncAccept(boost::shared_ptr<boost::asio::io_service> io_service, boost::shared_ptr<tcp::acceptor> m_acceptor, ros::NodeHandle n){
    std::cout << "(New Map) Waiting for connection" << std::endl;

    boost::shared_ptr<tcp::socket> sock = boost::shared_ptr<tcp::socket>(new tcp::socket(*io_service));

    m_acceptor->accept(*sock);
    std::cout << "(New Map) Command socket connected to " << sock->remote_endpoint().address().to_string() << std::endl;
    connected = true;
    waiting = false;
    boost::thread t(boost::bind(session, sock, n));
}

void serverDisconnected(const std_msgs::String::ConstPtr& msg){
    if(connected){
        std::cout << "(New Map) I heard " << std::endl;
        connected = false;
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "read_new_map");
    ros::NodeHandle n;
    
    std::cout << "(New Map) Ready to be launched." << std::endl;

    /// Subscribe to know when we disconnected from the server
    ros::Subscriber sub = n.subscribe("server_disconnected", 1000, serverDisconnected);

    /// Advertise that we are going to publish to /map
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1000);

    boost::shared_ptr<boost::asio::io_service> io_service = boost::shared_ptr<boost::asio::io_service>(new boost::asio::io_service());
    io_service->run();

    boost::shared_ptr<tcp::endpoint> m_endpoint = boost::shared_ptr<tcp::endpoint>(new tcp::endpoint(tcp::v4(), PORT));
    boost::shared_ptr<tcp::acceptor> m_acceptor = boost::shared_ptr<tcp::acceptor>(new tcp::acceptor(*io_service, *m_endpoint));

    m_acceptor->set_option(tcp::acceptor::reuse_address(true));

    ros::Rate r(10);
    while(ros::ok()){
        if(!connected && !waiting){
            std::cout << "(New Map) Ready to connect" << std::endl;
            boost::thread t(boost::bind(asyncAccept, io_service, m_acceptor, n));

            waiting = true;
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
