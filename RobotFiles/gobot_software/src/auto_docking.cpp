#include "auto_docking.hpp"

/// we want the robot to be at most at 0.15 metres of its goal
#define ROBOT_POS_TOLERANCE 0.15

move_base_msgs::MoveBaseGoal currentGoal;
std::shared_ptr<MoveBaseClient> ac(0);

bool docking = false;
bool landingPointReached = false;
int attempt = 0;
bool collision = false;
bool charging = false;

bool leftFlag = false;
bool rightFlag = false;
bool rearFlag = false;

std::chrono::system_clock::time_point timer;

ros::Subscriber goalStatusSub;
ros::Subscriber robotPoseSub;
ros::Subscriber bumperSub;
ros::Subscriber irSub;
ros::Subscriber batterySub;

/****************************************** STEP 1 : Go 1.5 meters in front of the charging station *********************************************************/

/// Service to start docking
bool startDocking(void){
    ros::NodeHandle nh;

    landingPointReached = false;
    leftFlag = false;
    rightFlag = false;
    rearFlag = false;

    /// Get the charging station position from the home file
    std::string homeFile;
    if(nh.hasParam("home_file")){
        nh.getParam("home_file", homeFile);
        std::cout << "(auto_docking::startDocking) home file path : " << homeFile << std::endl;
        std::ifstream ifs(homeFile);

        if(ifs.is_open()){
            tfScalar x, y, oriX, oriY, oriZ, oriW;
            ifs >> x >> y >> oriX >> oriY >> oriZ >> oriW;
            ifs.close();

            std::cout << "(auto_docking::startDocking) home found : " << x << " " << y << " " << oriX << " " << oriY << " " << oriZ << " " << oriW << std::endl;

            /// Got a quaternion and want an orientation in radian
            tf::Matrix3x3 matrix = tf::Matrix3x3(tf::Quaternion(oriX , oriY , oriZ, oriW));

            tfScalar roll;
            tfScalar pitch;
            tfScalar yaw;

            matrix.getRPY(roll, pitch, yaw);
            double homeOri = -(yaw*180/3.14159);//-(orientation+90)*3.14159/180);

            /// We want to go 1.5 metres in front of the charging station
            double landingPointX = x + 1.5 * std::cos(homeOri);
            double landingPointY = y + 1.5 * std::sin(homeOri);
            std::cout << "(auto_docking::startDocking) landing point : " << landingPointX << " " << landingPointY << " " << homeOri << std::endl;

            /// Create the goal
            currentGoal.target_pose.header.stamp = ros::Time::now();
            currentGoal.target_pose.pose.position.x = landingPointX;
            currentGoal.target_pose.pose.position.y = landingPointY;
            currentGoal.target_pose.pose.position.z = 0;
            currentGoal.target_pose.pose.orientation.x = oriX;
            currentGoal.target_pose.pose.orientation.y = oriY;
            currentGoal.target_pose.pose.orientation.z = oriZ;
            currentGoal.target_pose.pose.orientation.w = oriW;
            
            /// send the goal
            if(ac->isServerConnected()) {
                ac->sendGoal(currentGoal);

                docking = true;

                /// will allow us to check that we arrived at our destination
                goalStatusSub = nh.subscribe("/move_base/status", 1, goalStatus);
                robotPoseSub = nh.subscribe("/robot_pose", 1, getRobotPos);

                std::cout << "(auto_docking::startDocking) service called succesfully" << std::endl;

                return true;
            } else 
                std::cout << "(auto_docking::startDocking) no action server" << std::endl;
        } else
            std::cout << "(auto_docking::startDocking) could not open the file " << homeFile << std::endl;
    } else
        std::cout << "(auto_docking::startDocking) could not find the param home_file " << homeFile << std::endl;

    return false;
}

void getRobotPos(const geometry_msgs::Pose::ConstPtr& robotPos){
    /// if we are docking
    if(docking){
        /// we check if the robot is close enough to its goal
        if(std::abs(robotPos->position.x - currentGoal.target_pose.pose.position.x) < ROBOT_POS_TOLERANCE && 
           std::abs(robotPos->position.y - currentGoal.target_pose.pose.position.y) < ROBOT_POS_TOLERANCE){
            /// if the robot has already arrived, we want to wait for the next goal instead of repeating the same "success" functions
            if(!landingPointReached){
                std::cout << "(auto_docking::getRobotPos) getRobotPos robot close enough to the goal" << std::endl;
                std::cout << "(auto_docking::getRobotPos) robot position " << robotPos->position.x << " " << robotPos->position.y << std::endl;
                std::cout << "(auto_docking::getRobotPos) robot goal " << currentGoal.target_pose.pose.position.x << " " << currentGoal.target_pose.pose.position.y << std::endl;
                landingPointReached = true;
                findChargingStation();
            }
        }
    }
}

// to get the status of the robot (completion of the path towards its goal, SUCCEEDED = reached its goal, ACTIVE = currently moving towards its goal)
void goalStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& goalStatusArray){
    /// TODO reset or ignore goal status after docking at least once or we'll receive the last message from the previous attempt
    if(docking){
        if(goalStatusArray->status_list[0].status == 3){
            /// if we reached the goal for the fisrt time
            if(!landingPointReached){
                landingPointReached = true;
                findChargingStation();
            }
        } else if(goalStatusArray->status_list[0].status == 4 || goalStatusArray->status_list[0].status == 5){
            /// if the goal could not be reached
            if(!landingPointReached)
                failedDocking();
            
        } else {
            //std::cout << "(auto_docking::goalStatus) status : " << (int) goalStatusArray->status_list[0].status << std::endl;
            /// NOTE not sure it's useful
            //landingPointReached = false;
        }
    }
}

/****************************************** STEP 2 : Use the IR to guide the robot to the charging station *********************************************************/


void findChargingStation(void){
    std::cout << "(auto_docking::findChargingStation) called" << std::endl;

    /// we don't need this subscriber anymore
    goalStatusSub.shutdown();
    robotPoseSub.shutdown();

    /// if action server is up -> cancel any goal
    if(ac->isServerConnected())
        ac->cancelAllGoals();

    /// TODO subscribe to battery info + ir signals + bumpers
}

bool setSpeed(const char directionR, const int velocityR, const char directionL, const int velocityL){
    std::cout << "(auto_docking::setSpeed) " << directionR << " " << velocityR << " " << directionL << " " << velocityL << std::endl;
    wheel::SetSpeeds speed; 
    speed.request.directionR = std::string(1, directionR);
    speed.request.velocityR = velocityR;
    speed.request.directionL = std::string(1, directionL);
    speed.request.velocityL = velocityL;

    return ros::service::call("setSpeeds", speed);
}

/***************************************************************************************************/

void failedDocking(void){
    attempt++;
    if(attempt <= 3)
        startDocking();
    else {
        stopDocking();
        /// TODO tell command_system that we stopped trying to dock
    }
}

void stopDocking(void){
    std::cout << "(auto_docking::stopDocking) called" << std::endl;
    docking = false;
    landingPointReached = false;

    /// unsubscribe so we don't receive messages for nothing
    goalStatusSub.shutdown();
    robotPoseSub.shutdown();

    /// if action server is up -> cancel
    if(ac->isServerConnected())
        ac->cancelAllGoals();
}

bool stopDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::cout << "(auto_docking::stopDockingService) service called" << std::endl;

    stopDocking();

    return true;
}

bool startDockingService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::cout << "(auto_docking::startDockingService) service called" << std::endl;

    docking = false;
    attempt = 0;

    return startDocking();
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "auto_docking");
    ros::NodeHandle nh;

    ac = std::shared_ptr<MoveBaseClient> (new MoveBaseClient("move_base", true));

    ros::ServiceServer startDockingSrv = nh.advertiseService("startDocking", startDockingService);
    ros::ServiceServer stopDockingSrv = nh.advertiseService("stopDocking", stopDockingService);

    ros::spin();
}