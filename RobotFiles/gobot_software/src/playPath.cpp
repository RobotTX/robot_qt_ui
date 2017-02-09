#include "playPath.hpp"

#define PLAY_PATH_PORT 8333
#define ROBOT_POS_TOLERANCE 0.5

const std::string PATH_STAGE_FILE = "/home/gtdollar/computer_software/Robot_Infos/path_stage.txt";
const std::string IS_HOME_FILE = "/home/gtdollar/computer_software/Robot_Infos/is_home.txt";

std::shared_ptr<MoveBaseClient> ac(0);

// the stage of the robot within the path (if path going from first point to second point, stage is 0, if going from point before last point to last point stage is #points-1)
int stage = 0;

// holds whether the robot is ready to accept a new goal or not (already moving towards one)
bool waitingForNextGoal = false;

std::vector<Point> path;
Point currentGoal;

ros::Publisher nextPoint;
ros::Publisher cancelPublisher;

ros::Subscriber statusSuscriber;
ros::Subscriber sub_robot;

ros::ServiceServer _playPathService;
ros::ServiceServer _pausePathService;
ros::ServiceServer _stopPathService;
ros::ServiceServer _goHomeService;
ros::ServiceServer _stopGoingHomeService;


void getRobotPos(const geometry_msgs::Pose::ConstPtr& msg){
	// if there is a currentGoal
	if(currentGoal.x != -1){
		/// we check if the robot is close enough to its goal
		if(std::abs(msg->position.x - currentGoal.x) < ROBOT_POS_TOLERANCE && std::abs(msg->position.y - currentGoal.y) < ROBOT_POS_TOLERANCE){
			/// if the robot has already arrived, we want to wait for the next goal instead of repeating the same "success" functions
			if(!waitingForNextGoal){
				std::cout << "(PlayPath) getRobotPos robot close enough to the goal" << std::endl;
				std::cout << "(PlayPath) robot position " << msg->position.x << " " << msg->position.y 
				<< "\n(PlayPath) robot goal " << currentGoal.x << " " << currentGoal.y
				<< std::endl;
				waitingForNextGoal = true;
				goalReached();
			}
		}
	}
}

// to get the status of the robot (completion of the path towards its next goal, SUCCEEDED = reached its goal, ACTIVE = currently moving towards its goal)
void getStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& goalStatusArray){
	if(currentGoal.x != -1){
		if(goalStatusArray->status_list[0].status == 3){
			// if we reached the goal
			if(!waitingForNextGoal){
				waitingForNextGoal = true;
				goalReached();
			}
		} else if(goalStatusArray->status_list[0].status == 4 || goalStatusArray->status_list[0].status == 5){
			// if the goal could not be reached
			if(!waitingForNextGoal){
				waitingForNextGoal = true;
				// we use -stage to tell where on the path we blocked
				setStageInFile(-stage);
			}
		} else {
			waitingForNextGoal = false;
		}
	}
}

// called when the last goal has been reached
void goalReached(){
	if(currentGoal.isHome){
		std::cout << "(PlayPath) home reached" << std::endl;
	} else {
		std::cout << "(PlayPath) path point reached" << std::endl;
		stage++;
		if(stage >= path.size()){
			// the robot successfully reached all its goals
			std::cout << "I have completed my journey Master Joda, what will you have me do ?" << std::endl;
			// resets the stage of the path to be able to play the path from the start again
			stage = 0;
			// resets the current goal
			currentGoal.x = -1;
		} else {
			// reached a normal/path goal so we sleep the given time
			if(currentGoal.waitingTime > 0){
				std::cout << "(PlayPath) goalReached going to sleep for " << currentGoal.waitingTime << " seconds" << std::endl;
				sleep(currentGoal.waitingTime);
			}
			goNextPoint();
		}
		setStageInFile(stage);
	}
}

bool stopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	std::cout << "(PlayPath) stopPathService called" << std::endl;
	ac->cancelAllGoals();
	currentGoal.x = -1;
	stage = 0;
	setStageInFile(stage);
	return true;
}

bool pausePathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	std::cout << "(PlayPath) pausePathService called" << std::endl;
	ac->cancelAllGoals();
	currentGoal.x = -1;
	return true;
}

void goNextPoint(){
	std::cout << "(PlayPath) goNextPoint called" << std::endl;
	// get the next point in the path list and tell the robot to go there

	if(path.size()-1 == stage)
		std::cout << "(PlayPath) This is my final destination" << std::endl;

    Point point;
    point.x = path.at(stage).x;
    point.y = path.at(stage).y;
    point.waitingTime = path.at(stage).waitingTime;
    point.isHome = false;

    goToPoint(point);
}

// sends the next goal to the robot
void goToPoint(const Point& point){
	std::cout << "(PlayPath) goToPoint " << point.x << " " << point.y << std::endl;
	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = point.x;
    goal.target_pose.pose.position.y = point.y;
    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0;
    goal.target_pose.pose.orientation.w = 1;

	currentGoal = point;

    ac->sendGoal(goal);
}

void setStageInFile(const int _stage){

	std::ofstream path_stage_file(PATH_STAGE_FILE, std::ios::out | std::ios::trunc);
	// when the stage is < 0 it means the robot was blocked at stage -stage (which is > 0)
	if(path_stage_file){
		std::cout << "(PlayPath) setStageInFile " << _stage << std::endl;
		path_stage_file << _stage;
		path_stage_file.close();
	} else {
		std::cout << "(PlayPath) Sorry we were not able to find the file play_path.txt in order to keep track of the stage of the path to be played" << std::endl;
	}
}

bool playPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	std::cout << "(PlayPath) playPathService called" << std::endl;

	path = std::vector<Point>();

	ros::NodeHandle n;

	std::ifstream file("/home/gtdollar/computer_software/Robot_Infos/path.txt", std::ios::in);

	// we recreate the path to follow from the file
	if(file){

        std::string line;

        while(getline(file, line)){
        	std::istringstream iss(line);
            Point pathPoint;
            iss >> pathPoint.x >> pathPoint.y >> pathPoint.waitingTime;
            pathPoint.isHome = false;
            path.push_back(pathPoint);
        }

	} else {
		std::cerr << "sorry could not find the path file on the robot, returning false to the cmd system";
		return false;
	}

	if(path.empty()){
		std::cerr << "the path is empty, returning false to cmd system";
		return false;
	}

	for(size_t i = 0; i < path.size(); i++)
		std::cout << "(PlayPath) Stage " << i << " " << path.at(i).x << " " << path.at(i).y << " " << path.at(i).waitingTime << std::endl;


	if(currentGoal.x == -1){
		stage = 0;
		setStageInFile(stage);
	}

	goNextPoint();

	return true;	
}

bool goHomeService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	std::cout << "(PlayPath) goHomeService called" << std::endl;
	
	std::ifstream file("/home/gtdollar/computer_software/Robot_Infos/path.txt", std::ios::in);

	Point home;
	home.isHome = true;

	// we recreate the path to follow from the file
	if(file){

        std::string line;

        while(getline(file, line)){
        	std::istringstream iss(line);
            iss >> home.x >> home.y;
       		home.waitingTime = 0;
        }

        goToPoint(home);

		return true; 

	} else {
		std::cerr << "sorry could not find the home file on the robot, returning false to the cmd system";
		return false;
	}
}

bool stopGoingHomeService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	std::cout << "(PlayPath) stopGoingHomeService called" << std::endl;
	ac->cancelAllGoals();
	currentGoal.x = -1;

	return true;
}

int main(int argc, char* argv[]){

	std::cout << "(PlayPath) play path main running..." << std::endl;

	try {

		ros::init(argc, argv, "play_path");

		currentGoal.x = -1;
		currentGoal.y = -1;
		currentGoal.waitingTime = -1;
		currentGoal.isHome = false;
		
		ros::NodeHandle n;

		// service to play the robot's path
		_playPathService = n.advertiseService("play_path", playPathService);

		// service to pause the robot's path
		_pausePathService = n.advertiseService("pause_path", pausePathService);

		// service to stop the robot's path
		_stopPathService = n.advertiseService("stop_path", stopPathService);

		// service to send the robot home
		_goHomeService = n.advertiseService("go_home", goHomeService);

		// stops the robot on its way home
		_stopGoingHomeService = n.advertiseService("stop_going_home", stopGoingHomeService);

		// to cancel a goal
		cancelPublisher = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1000);

		// to publish the goals as the robot plays its path
		nextPoint = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);

		// tell the action client that we want to spin a thread by default
		ac = std::shared_ptr<MoveBaseClient> (new MoveBaseClient("move_base", true));

		// get the current status of the goal 
		statusSuscriber = n.subscribe("/move_base/status", 1, getStatus);

		// get the position of the robot to compare with the goal
		sub_robot = n.subscribe("/robot_pose", 1, getRobotPos);

		// wait for the action server to come up
		while(!ac->waitForServer(ros::Duration(5.0)))
			ROS_INFO("Waiting for the move_base action server to come up");

		ros::spin();

	} catch (std::exception& e) {
		std::cerr << "(Play path) Exception: " << e.what() << std::endl;
	}

	return 0;
}

/*
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server
*/
