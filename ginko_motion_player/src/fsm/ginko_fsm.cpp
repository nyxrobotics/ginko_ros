#include "ginko_fsm.h"
using namespace ginko_fsm;

GinkoTimer ginko_timer_;
GinkoPlayer ginko_player_;

enum MOTIN_NUM
{
    TORQUE_OFF = 0,
	TORQUE_ON,
	STANDING,

	WAKEUP_FRONT,
	WAKEUP_BACK,
	WALK_FRONT,
	WALK_BACK,
	WALK_RIGHT,
	WALK_LEFT,
	TURN_RIGHT,
	TURN_LEFT,

	ATK_RIGHT_1,
	ATK_RIGHT_2,
	ATK_RIGHT_BACK_1,
	ATK_RIGHT_BACK_2,

	ATK_LEFT_1,
	ATK_LEFT_2,
	ATK_LEFT_BACK_1,
	ATK_LEFT_BACK_2,

	MOVE_URG,

};

volatile bool leftBumper, rightBumper, wallSensor;

int _motiomCommand = TORQUE_OFF;
unsigned char _motiomCommandChanged = 0;

ros::Subscriber _motiomCommandSub;
ros::Publisher _PosePublisher;

ros::Subscriber leftBumperSub;
ros::Subscriber rightBumperSub;
ros::Subscriber wallSensorSub;

EventQueue* mainEventQueue;
struct MainEventQueue{
	MainEventQueue(){ mainEventQueue = new RosEventQueue();	}
	~MainEventQueue(){ delete mainEventQueue; }
};

FSM(Ginko)
{
	enum STATES
	{
		torqueOff,
		torqueOn,
		standing,
		wakeupFront,wakeupBack,
		walkFront,walkBack,
		turnRight,turnLeft,
		attackRight1,attackRight2,attackRightBack1,attackRightBack2,
		attackLeft1,attackLeft2,attackLeftBack1,attackLeftBack2,
		moveUrg

	}
	FSM_START(torqueOff);
	FSM_BGN
	{
        FSM_STATE(torqueOff){
            FSM_CALL_TASK(torqueOffTask)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_ON , FSM_NEXT(torqueOn));
				FSM_ON_CONDITION(_motiomCommand == STANDING , FSM_NEXT(standing));
            }
        }
        FSM_STATE(torqueOn){
            FSM_CALL_TASK(torqueOnTask)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
				FSM_ON_CONDITION(_motiomCommand == STANDING , FSM_NEXT(standing));
            }
        }
        FSM_STATE(standing){
            FSM_CALL_TASK(standingTask)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
				FSM_ON_CONDITION(_motiomCommand == WALK_FRONT , FSM_NEXT(walkFront));
				FSM_ON_CONDITION(_motiomCommand == WALK_FRONT , FSM_NEXT(walkBack));
				FSM_ON_CONDITION(_motiomCommand == WALK_FRONT , FSM_NEXT(turnRight));
				FSM_ON_CONDITION(_motiomCommand == WALK_FRONT , FSM_NEXT(turnLeft));
				FSM_ON_CONDITION(_motiomCommand == WALK_FRONT , FSM_NEXT(attackRight1));
				FSM_ON_CONDITION(_motiomCommand == WALK_FRONT , FSM_NEXT(attackRight2));
				FSM_ON_CONDITION(_motiomCommand == WALK_FRONT , FSM_NEXT(attackRightBack1));
				FSM_ON_CONDITION(_motiomCommand == WALK_FRONT , FSM_NEXT(attackRightBack2));
				FSM_ON_CONDITION(_motiomCommand == WALK_FRONT , FSM_NEXT(attackLeft1));
				FSM_ON_CONDITION(_motiomCommand == WALK_FRONT , FSM_NEXT(attackLeft2));
				FSM_ON_CONDITION(_motiomCommand == WALK_FRONT , FSM_NEXT(attackLeftBack1));
				FSM_ON_CONDITION(_motiomCommand == WALK_FRONT , FSM_NEXT(attackLeftBack2));
				FSM_ON_CONDITION(_motiomCommand == WAKEUP_FRONT && _motiomCommandChanged ==1 , FSM_NEXT(wakeupFront));
				FSM_ON_CONDITION(_motiomCommand == WAKEUP_BACK && _motiomCommandChanged ==1 , FSM_NEXT(wakeupBack));
				FSM_ON_CONDITION(_motiomCommand == MOVE_URG && _motiomCommandChanged ==1 , FSM_NEXT(moveUrg));
            }
        }
        FSM_STATE(wakeupFront){
            FSM_CALL_TASK(wakeupFrontTask)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(wakeupBack){
            FSM_CALL_TASK(wakeupBackTask)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));

            }
        }

        FSM_STATE(walkFront){
            FSM_CALL_TASK(walkFrontTask)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(walkBack){
            FSM_CALL_TASK(walkBackTask)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(turnRight){
            FSM_CALL_TASK(turnRightTask)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(turnLeft){
            FSM_CALL_TASK(turnLeftTask)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }

        FSM_STATE(attackRight1){
            FSM_CALL_TASK(attackRight1Task)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(attackRight2){
            FSM_CALL_TASK(attackRight2Task)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(attackRightBack1){
            FSM_CALL_TASK(attackRightBack1Task)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(attackRightBack2){
            FSM_CALL_TASK(attackRightBack2Task)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(attackLeft1){
            FSM_CALL_TASK(attackLeft1Task)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(attackLeft2){
            FSM_CALL_TASK(attackLeft2Task)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(attackLeftBack1){
            FSM_CALL_TASK(attackLeftBack1Task)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(attackLeftBack2){
            FSM_CALL_TASK(attackLeftBack2Task)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(moveUrg){
            FSM_CALL_TASK(moveUrgTask)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }



	}
	FSM_END
}


//Subscribers callback =======================================================
void motionCommandCallback(const std_msgs::String::ConstPtr& msg)
{
	string tmpCommandString = msg->data.c_str();
	static string preCommandString = "TORQUE_OFF";
	//	static int prev_command = TORQUE_OFF;

	if(tmpCommandString  == "TORQUE_OFF" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = TORQUE_OFF;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "TORQUE_ON" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = TORQUE_ON;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "STANDING" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = STANDING;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "WAKEUP_FRONT" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = WAKEUP_FRONT;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "WAKEUP_BACK" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = WAKEUP_BACK;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "WALK_FRONT" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = WALK_FRONT;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "WALK_BACK" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = WALK_BACK;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "TURN_RIGHT" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = TURN_RIGHT;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "TURN_LEFT" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = TURN_LEFT;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "ATK_RIGHT_1" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = ATK_RIGHT_1;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "ATK_RIGHT_2" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = ATK_RIGHT_2;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "ATK_RIGHT_BACK_1" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = ATK_RIGHT_BACK_1;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "ATK_RIGHT_BACK_2" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = ATK_RIGHT_BACK_2;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "ATK_LEFT_1" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = ATK_LEFT_1;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "ATK_LEFT_2" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = ATK_LEFT_2;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "ATK_LEFT_BACK_1" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = ATK_LEFT_BACK_1;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "ATK_LEFT_BACK_2" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = ATK_LEFT_BACK_2;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}

	else if(tmpCommandString  == "MOVE_URG" && preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = MOVE_URG;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}

	else if(preCommandString != tmpCommandString){
		_motiomCommandChanged = 1;
		_motiomCommand = STANDING;
	}

//	prev_command = _motiomCommand;
	preCommandString = tmpCommandString;
}


//tasks
decision_making::TaskResult torqueOnCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("torqueOnTask...");
    ginko_player_.torqueEnable(1);
    usleep(100000);
    ginko_player_.torqueEnable(0);
//    usleep(100000);
    sleep(1);
    ginko_player_.torqueEnable(1);
    usleep(100000);
//    sleep(1);
//    ginko_player_.playPose(standing_Motion_Start,0);
    ginko_player_.playPose(standing_Motion_Loop,0);
//    ginko_player_.playPose(walkFront_Motion_Start,0);
//    ginko_player_.torqueEnable(1);
    eventQueue.riseEvent("/MOTION_FINISH");
    return TaskResult::SUCCESS();
}
decision_making::TaskResult torqueOffCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("torqueOffTask...");
//    sleep(1);
    ginko_player_.torqueEnable(0);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult standingCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("standingTask...");
//    sleep(1);
//    ginko_player_.playPose(standing_Motion_Start,0);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult wakeupFrontCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("wakeupFrontTask...");
    ginko_player_.playMotion(wakeupFront_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult wakeupBackCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("wakeupBackTask...");
    ginko_player_.playMotion(wakeupBack_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult walkFrontCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("walkFrontStart...");
    ginko_player_.playMotion(walkFront_Motion_Start);
    while(_motiomCommand == WALK_FRONT){
        ROS_INFO("walkFrontLoop...");
        ginko_player_.playMotion(walkFront_Motion_Loop);
    }
    if(_motiomCommand != TORQUE_OFF){
        ROS_INFO("walkFrontEnd...");
        ginko_player_.playMotion(walkFront_Motion_End);
    }
    ginko_timer_.msleepCyclic(1000);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult walkBackCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("walkBackStart...");
    ginko_player_.playMotion(walkBack_Motion_Start);
    while(_motiomCommand == WALK_BACK){
        ROS_INFO("walkBackLoop...");
        ginko_player_.playMotion(walkBack_Motion_Loop);
    }
    if(_motiomCommand != TORQUE_OFF){
        ROS_INFO("walkBackEnd...");
        ginko_player_.playMotion(walkBack_Motion_End);
    }
    ginko_timer_.msleepCyclic(1000);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}

decision_making::TaskResult turnRightCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("turnRightTask...");
    ginko_player_.playMotion(turnRight_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult turnLeftCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("turnLeftTask...");
    ginko_player_.playMotion(turnLeft_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult attackRight1Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("attackRight1Task...");
    ginko_player_.playMotion(attackRight1_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult attackRight2Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("attackRight2Task...");
    ginko_player_.playMotion(attackRight2_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult attackRightBack1Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("attackRightBack1Task...");
    ginko_player_.playMotion(attackRightBack1_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult attackRightBack2Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("attackRightBack2Task...");
    ginko_player_.playMotion(attackRightBack2_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult attackLeft1Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("attackLeft1Task...");
    ginko_player_.playMotion(attackLeft1_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult attackLeft2Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("attackLeft2Task...");
    ginko_player_.playMotion(attackLeft2_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult attackLeftBack1Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("attackLeftBack1Task...");
    ginko_player_.playMotion(attackLeftBack1_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult attackLeftBack2Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("attackLeftBack2Task...");
    ginko_player_.playMotion(attackLeftBack2_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}
/*
decision_making::TaskResult moveUrgCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("moveUrgStart...");
    ginko_player_.playMotion(move_urg3_Motion_Loop);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}
*/

decision_making::TaskResult moveUrgCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("moveUrgStart...");
//    ginko_player_.playMotion(walkFront_Motion_Start);
    while(_motiomCommand == MOVE_URG){
        ROS_INFO("moveUrgLoop...");
//        ginko_player_.playMotion(move_urg2_Motion_Loop);
        ginko_player_.playMotion(move_urg3_Motion_Loop);
    }
    if(_motiomCommand != TORQUE_OFF){
        ROS_INFO("moveUrgEnd...");
//        ginko_player_.playMotion(walkFront_Motion_End);
    }
    ginko_timer_.msleepCyclic(1000);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motiomCommandChanged = 0;
    return TaskResult::SUCCESS();
}







//main
int main(int argc, char** argv){

	ros::init(argc, argv, "fsm_roomba");
	MainEventQueue meq;
	ros_decision_making_init(argc, argv);

	ros::NodeHandle node;
	ros::TransportHints transport_hints;
	transport_hints.tcpNoDelay(true);
	_motiomCommandSub = node.subscribe("/motion_command",10,&motionCommandCallback,transport_hints);
	//init timer
    ginko_timer_.usecStart();
    ginko_player_.initPublisher();
	//Tasks registration
    LocalTasks::registrate("torqueOnTask",	torqueOnCallback);
    LocalTasks::registrate("torqueOffTask",	torqueOffCallback);
    LocalTasks::registrate("standingTask",	standingCallback);
    LocalTasks::registrate("wakeupFrontTask",	wakeupFrontCallback);
    LocalTasks::registrate("wakeupBackTask",	wakeupBackCallback);
    LocalTasks::registrate("walkFrontTask",	walkFrontCallback);
    LocalTasks::registrate("walkBackTask",	walkBackCallback);
    LocalTasks::registrate("moveUrgTask",	moveUrgCallback);
    LocalTasks::registrate("attackRight1Task",	attackRight1Callback);
    LocalTasks::registrate("attackRight2Task",	attackRight2Callback);
    LocalTasks::registrate("attackRightBack1Task",	attackRightBack1Callback);
    LocalTasks::registrate("attackRightBack2Task",	attackRightBack2Callback);
    LocalTasks::registrate("attackLeft1Task",	attackLeft1Callback);
    LocalTasks::registrate("attackLeft2Task",	attackLeft2Callback);
    LocalTasks::registrate("attackLeftBack1Task",	attackLeftBack1Callback);
    LocalTasks::registrate("attackLeftBack2Task",	attackLeftBack2Callback);
//    LocalTasks::registrate("moveURG2Task",	moveURG2Callback);
//    LocalTasks::registrate("moveURG3Task",	moveURG3Callback);

	ros::AsyncSpinner spinner(2);
	spinner.start();

	ROS_INFO("Starting roobma...");

	/**
	 * Blocking call
	 */
	mainEventQueue->async_spin();
	FsmGinko(NULL, mainEventQueue);
	mainEventQueue->close();

	spinner.stop();
	ROS_INFO("Roomba done");

	return 0;
}

