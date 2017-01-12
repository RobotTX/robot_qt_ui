#include "teleop.hpp"
#include <utility>

using boost::asio::ip::tcp;

const int max_length = 1024;
bool waiting = false;
bool connected = false;

boost::asio::io_service io_service;
tcp::socket socket_teleop(io_service);
tcp::acceptor m_acceptor(io_service);

ros::Publisher teleop_pub;
ros::Publisher stop_pub;

void session(boost::shared_ptr<tcp::socket> sock, ros::NodeHandle n){
    std::cout << "(Teleop) session launched" << std::endl;

    while(ros::ok() && connected){
        char data[max_length];

        boost::system::error_code error;
        size_t length = sock->read_some(boost::asio::buffer(data), error);
        if ((error == boost::asio::error::eof) || (error == boost::asio::error::connection_reset)){
            std::cout << "(Teleop) Connection closed" << std::endl;
            connected = false;
            return;
        } else if (error) {
            throw boost::system::system_error(error); // Some other error.
            return;
        }
        teleop(static_cast<int8_t>(data[0]));
   /// data[i]

    }
}

void asyncAccept(boost::shared_ptr<boost::asio::io_service> io_service, boost::shared_ptr<tcp::acceptor> m_acceptor, ros::NodeHandle n){
    std::cout << "(Teleop) Waiting for connection" << std::endl;

    boost::shared_ptr<tcp::socket> sock = boost::shared_ptr<tcp::socket>(new tcp::socket(*io_service));

    m_acceptor->accept(*sock);
    std::cout << "(Teleop) Command socket connected to " << sock->remote_endpoint().address().to_string() << std::endl;
    connected = true;
    waiting = false;
    boost::thread t(boost::bind(session, sock, n));
}

void serverDisconnected(const std_msgs::String::ConstPtr& msg){
    if(connected){
        std::cout << "(Teleop) I heard " << std::endl;
        teleop(4);
        connected = false;
    }
}

void teleop(const int8_t val){
    std::cout << "(Teleop) got data " << (int) val << std::endl;
    if(connected){
/*
0:std::pair<int8_t, int8_t>(1,1);
1:std::pair<int8_t, int8_t>(1,0);
2:std::pair<int8_t, int8_t>(1,-1);
3:std::pair<int8_t, int8_t>(0,1);
5:std::pair<int8_t, int8_t>(0,-1);
6:std::pair<int8_t, int8_t>(-1,-1);
7:std::pair<int8_t, int8_t>(-1,0);
8:std::pair<int8_t, int8_t>(-1,1);*/
        float speed = 0.2;
        float turnSpeed = 1.0;
        int x, th;
        switch(val){
            case 0:
                x = 1;
                th = 1;
            break;
            case 1:
                x = 1;
                th = 0;
            break;
            case 2:
                x = 1;
                th = -1;
            break;
            case 3:
                x = 0;
                th = 1;
            break;
            case 5:
                x = 0;
                th = -1;
            break;
            case 6:
                x = -1;
                th = -1;
            break;
            case 7:
                x = -1;
                th = 0;
            break;
            case 8:
                x = -1;
                th = 1;
            break;
            default:
                x = 0;
                th = 0;
            break;
        }

        actionlib_msgs::GoalID cancel;
        cancel.stamp = ros::Time::now();
        cancel.id = "map";

        stop_pub.publish(cancel);

        ros::spinOnce();
        

        geometry_msgs::Twist twist;
        twist.linear.x = x*speed;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = th*turnSpeed;

        teleop_pub.publish(twist);
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "teleop");
    ros::NodeHandle n;
    
    std::cout << "(Teleop) Ready to be launched." << std::endl;

    /// Subscribe to know when we disconnected from the server
    ros::Subscriber sub = n.subscribe("server_disconnected", 1000, serverDisconnected);

    /// Advertise that we are going to publish to /cmd_vel
    teleop_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    stop_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1000);

    boost::shared_ptr<boost::asio::io_service> io_service = boost::shared_ptr<boost::asio::io_service>(new boost::asio::io_service());
    io_service->run();

    boost::shared_ptr<tcp::endpoint> m_endpoint = boost::shared_ptr<tcp::endpoint>(new tcp::endpoint(tcp::v4(), PORT));
    boost::shared_ptr<tcp::acceptor> m_acceptor = boost::shared_ptr<tcp::acceptor>(new tcp::acceptor(*io_service, *m_endpoint));

    m_acceptor->set_option(tcp::acceptor::reuse_address(true));

    ros::Rate r(10);
    while(ros::ok()){
        if(!connected && !waiting){
            std::cout << "(Teleop) Ready to connect" << std::endl;
            boost::thread t(boost::bind(asyncAccept, io_service, m_acceptor, n));

            waiting = true;
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
