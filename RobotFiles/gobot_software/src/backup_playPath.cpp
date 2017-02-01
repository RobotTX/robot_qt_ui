#include "playPath.hpp"

#define PLAY_PATH_PORT 8333

const std::string PATH_STAGE_FILE = "/home/gtdollar/computer_software/Robot_Infos/path_stage.txt";
const std::string IS_HOME_FILE = "/home/gtdollar/computer_software/Robot_Infos/is_home.txt";

std::shared_ptr<MoveBaseClient> ac(0);

// the stage of the robot within the path (if path going from first point to second point, stage is 0, if going from point before last point to last point stage is #points-1)
int stage = 0;

uint8_t status = 0;

std::vector<PathPoint> path; 

ros::Publisher nextPoint;
ros::Publisher cancelPublisher;

ros::Subscriber statusSuscriber;

ros::ServiceServer _playPathService;
ros::ServiceServer _pausePathService;
ros::ServiceServer _stopPathService;
ros::ServiceServer _goHomeService;
ros::ServiceServer _stopGoingHomeService;

bool greenLight = false;

int counter = 0;

// to get the status of the robot (completion of the path towards its next goal, SUCCEEDED = reached its goal, ACTIVE = currently moving towards its goal)
void getStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& _status){
	if(! _status->status_list.empty()){
		/// if our status changed from something to SUCCEEDED it means we have reached another point
		if(_status->status_list[0].status != status){
			std::cout << "New status : " << _status->status_list[0].text << " whose code is : " << std::to_string(static_cast<int>(_status->status_list[0].status)) << std::endl;
			status = _status->status_list[0].status;
		}
	}
}

bool stopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	std::cout << "stop path service called" << std::endl;

	ros::NodeHandle n;

	// the thread executes the service so that true can be returned to the command system
	boost::thread _thread(boost::bind(stopPath, n, ac));

	return true;
}

bool stopPath(ros::NodeHandle n, std::shared_ptr<MoveBaseClient> ac){
	
	// notifies the robot that it cannot move anymore
	greenLight = false;

	stage = 0;

	status = 0;
	
	// reset the path stage in the file
	std::ofstream path_stage_file(PATH_STAGE_FILE, std::ios::out | std::ios::trunc);

	if(path_stage_file){
		path_stage_file << "0";
		path_stage_file.close();
	} else {
		std::cout << "Sorry we were not able to find the file play_path.txt in order to keep track of the stage of the path to be played" << std::endl;
		return false;	
	}

	ac->cancelGoal();
	return true;
}

bool pausePathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	std::cout << "pause path service called" << std::endl;
	
	ros::NodeHandle n;

	// the thread executes the service so that true can be returned to the command system
	boost::thread _thread(boost::bind(pausePath, n, ac));

	return true;
}

void pausePath(ros::NodeHandle n, std::shared_ptr<MoveBaseClient> ac){
	// notifies the robot that it cannot move anymore
	std::cout << "Ready to cancel the next goal !" << std::endl;
	greenLight = false;
	ac->cancelGoal();
}

void startPath(ros::NodeHandle n, std::shared_ptr<MoveBaseClient> ac){

	// notifies the robot that it can go 
	greenLight = true;

	statusSuscriber = n.subscribe("/move_base/status", 1, getStatus);

	while(ros::ok() && (stage != path.size()) && greenLight){

		ROS_INFO("Heading towards target %d", stage+1);

		if(path.size()-1 == stage)
			ROS_INFO("This is my final destination");

		move_base_msgs::MoveBaseGoal goal;

		goal.target_pose.header.frame_id = "map";
	    goal.target_pose.header.stamp = ros::Time::now();

	    goal.target_pose.pose.position.x = path.at(stage).x;
	    goal.target_pose.pose.position.y = path.at(stage).y;
	    goal.target_pose.pose.orientation.x = 0;
	    goal.target_pose.pose.orientation.y = 0;
	    goal.target_pose.pose.orientation.z = 0;
	    goal.target_pose.pose.orientation.w = 1;

	    if(stage > 0 && path.at(stage-1).waitingTime >= 0){
	    	std::cout << "Hey I gotta wait for " << path.at(stage-1).waitingTime << std::endl;
			sleep(path.at(stage-1).waitingTime);
	    }

		// sends the robot's goal to the server 
	    ac->sendGoal(goal);

	    // waits for the answer of the server regarding the status of the robot
	    ac->waitForResult();

	    if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	    	ROS_INFO("Hooray, the base reached target %d", stage+1);
	    	stage++;
	    	// updates the path stage in the file
			std::ofstream path_stage_file(PATH_STAGE_FILE, std::ios::out | std::ios::trunc);

			if(path_stage_file){
				path_stage_file << stage;
				path_stage_file.close();
			} else 
				std::cout << "Sorry we were not able to find the file play_path.txt in order to keep track of the stage of the path to be played" << std::endl;	
	    } else
	    	ROS_INFO("The base failed to reach target %d", stage+1);
	}
	if(stage == path.size() && ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		// the robot successfully reached all its goals in order 
		std::cout << "I have completed my journey Master Joda, what will you have me do ?" << std::endl;
		// resets the stage of the path to be able to play the path from the start again
		stage = 0;
	}
	else
		std::cout << "I was stopped trying to reach " << path.at(stage).x << " " << path.at(stage).y << std::endl;
}

bool playPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	
	std::cout << "play path service called" << std::endl;

	path = std::vector<PathPoint>();

	ros::NodeHandle n;

	std::ifstream file("/home/gtdollar/computer_software/Robot_Infos/path.txt", std::ios::in);

	// we recreate the path to follow from the file
	if(file){

        std::string line;

        while(getline(file, line)){
        	std::istringstream iss(line);
            PathPoint pathPoint;
            iss >> pathPoint.x >> pathPoint.y >> pathPoint.waitingTime;
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
		std::cout << "Stage " << i << " " << path.at(i).x << " " << path.at(i).y << " " << path.at(i).waitingTime << std::endl;

	// writes stage 0 in the file path_stage.txt, this is sent later to the software
	std::ofstream path_stage_file(PATH_STAGE_FILE, std::ios::out | std::ios::trunc);

	if(path_stage_file){
		path_stage_file << "0";
		path_stage_file.close();
	} else {
		std::cerr << "Sorry we were not able to find the file play_path.txt in order to keep track of the stage of the path to be played" << std::endl;
		return false;	
	}

	// another thread executes the service so that true can be returned to the command system
	boost::thread _thread(boost::bind(startPath, n, ac));

	return true;	
}

bool goHomeService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	std::cout << "Go Home service called" << std::endl;

	ros::NodeHandle n;

	std::ifstream file("/home/gtdollar/computer_software/Robot_Infos/path.txt", std::ios::in);

	PathPoint home;

	// we recreate the path to follow from the file
	if(file){

        std::string line;

        while(getline(file, line)){
        	std::istringstream iss(line);
            iss >> home.x >> home.y;
       		home.waitingTime = 0;
        }

	} else {
		std::cerr << "sorry could not find the home file on the robot, returning false to the cmd system";
		return false;
	}
	
	boost::thread _thread(boost::bind(goHome, n, ac, home));

	return true; 
}

void goHome(ros::NodeHandle n, std::shared_ptr<MoveBaseClient> ac, const PathPoint& home){
	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = home.x;
    goal.target_pose.pose.position.y = home.y;
    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0;
    goal.target_pose.pose.orientation.w = 1;

    // sends the robot home
    ac->sendGoal(goal);

    // waits for the answer of the server regarding the status of the robot
    ac->waitForResult();

    if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    	std::ofstream ofs(IS_HOME_FILE);

    	(ofs) ? ofs << 1 : std::cout << "Sorry could not update my status in is_home.txt" << std::endl;

    	ROS_INFO("Awesome we are finally home !");
    }
    else
    	ROS_INFO("Oh noooo, I was not able to go home !");
}

bool stopGoingHomeService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	std::cout << "Stop going home service called" << std::endl;

	ros::NodeHandle n;

	boost::thread _thread(boost::bind(stopGoingHome, n, ac));

	return true;
}

void stopGoingHome(ros::NodeHandle n, std::shared_ptr<MoveBaseClient> ac){
	ac->cancelGoal();
	if(ac->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
		ROS_INFO("Successfully cancelled my trip home");
}

int main(int argc, char* argv[]){

	std::cout << "play path main running..." << std::endl;

	try {

		ros::init(argc, argv, "play_path");
		
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

		//tell the action client that we want to spin a thread by default
		ac = std::shared_ptr<MoveBaseClient> (new MoveBaseClient("move_base", true));

		//wait for the action server to come up
		while(!ac->waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}

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
