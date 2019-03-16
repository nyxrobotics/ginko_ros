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
	TURN_LIGHT,

	ATK_FRONT_RHIGH,
	ATK_FRONT_RLOW,
	ATK_FRONT_LHIGH,
	ATK_FRONT_LLOW,

	ATK_BACK_RHIGH,
	ATK_BACK_RLOW,
	ATK_BACK_LHIGH,
	ATK_BACK_LLOW,

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
		walkFront,
		moveUrg1
//		moveUrg2,
//		moveUrg3

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
				FSM_ON_CONDITION(_motiomCommand == WAKEUP_FRONT && _motiomCommandChanged ==1 , FSM_NEXT(wakeupFront));
				FSM_ON_CONDITION(_motiomCommand == WAKEUP_BACK && _motiomCommandChanged ==1 , FSM_NEXT(wakeupBack));
				FSM_ON_CONDITION(_motiomCommand == MOVE_URG && _motiomCommandChanged ==1 , FSM_NEXT(moveUrg1));
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
        FSM_STATE(moveUrg1){
            FSM_CALL_TASK(moveUrg1Task)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
//        FSM_STATE(moveUrg2){
//            FSM_CALL_TASK(moveURG2Task)
//            FSM_TRANSITIONS{
//				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
//                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
//            }
//        }
//        FSM_STATE(moveUrg3){
//            FSM_CALL_TASK(moveURG3Task)
//            FSM_TRANSITIONS{
//				FSM_ON_CONDITION(_motiomCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
//                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
//            }
//        }

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
	}else if(tmpCommandString  == "MOVE_URG" && preCommandString != tmpCommandString){
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
    usleep(100000);
    ginko_player_.torqueEnable(1);
//    sleep(1);
    ginko_player_.playPose(standing_Motion_Start,0);
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
    sleep(1);
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
decision_making::TaskResult moveUrg1Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
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
//decision_making::TaskResult moveURG2Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
//    ROS_INFO("walkFrontStart...");
//    ginko_player_.playMotion(walkFront_Motion_Start);
//    while(_motiomCommand == WALK_FRONT){
//        ROS_INFO("walkFrontLoop...");
//        ginko_player_.playMotion(walkFront_Motion_Loop);
//    }
//    if(_motiomCommand != TORQUE_OFF){
//        ROS_INFO("walkFrontEnd...");
//        ginko_player_.playMotion(walkFront_Motion_End);
//    }
//    ginko_timer_.msleepCyclic(1000);
//    eventQueue.riseEvent("/MOTION_FINISH");
//    _motiomCommandChanged = 0;
//    return TaskResult::SUCCESS();
//}
//
//decision_making::TaskResult moveURG3Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
//    ROS_INFO("walkFrontStart...");
//    ginko_player_.playMotion(walkFront_Motion_Start);
//    while(_motiomCommand == WALK_FRONT){
//        ROS_INFO("walkFrontLoop...");
//        ginko_player_.playMotion(walkFront_Motion_Loop);
//    }
//    if(_motiomCommand != TORQUE_OFF){
//        ROS_INFO("walkFrontEnd...");
//        ginko_player_.playMotion(walkFront_Motion_End);
//    }
//    ginko_timer_.msleepCyclic(1000);
//    eventQueue.riseEvent("/MOTION_FINISH");
//    _motiomCommandChanged = 0;
//    return TaskResult::SUCCESS();
//}

//main
int main(int argc, char** argv){

	ros::init(argc, argv, "fsm_roomba");
	MainEventQueue meq;
	ros_decision_making_init(argc, argv);

	ros::NodeHandle node;
	_motiomCommandSub = node.subscribe("/motion_command",1,&motionCommandCallback);
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
    LocalTasks::registrate("moveUrg1Task",	moveUrg1Callback);
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

