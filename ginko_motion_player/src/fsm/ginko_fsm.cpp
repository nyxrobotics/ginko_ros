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

    ATK_FRONT,
    ATK_BACK,
    ATK_R1,
    ATK_R2,
	ATK_RB1,
	ATK_RB2,
	ATK_L1,
	ATK_L2,
	ATK_LB1,
	ATK_LB2,

	MOVE_URG,

};

volatile bool leftBumper, rightBumper, wallSensor;

int _motionCommand = TORQUE_OFF;
unsigned char _motionCommandChanged = 0;

ros::Subscriber _motionCommandSub;
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
		walkFront,walkBack,walkRight,walkLeft,
		turnRight,turnLeft,
        atkFront,atkBack,
		atkRight1,atkRight2,atkRightBack1,atkRightBack2,
		atkLeft1,atkLeft2,atkLeftBack1,atkLeftBack2,
		moveUrg

	}
	FSM_START(torqueOff);
	FSM_BGN
	{
        FSM_STATE(torqueOff){
            FSM_CALL_TASK(torqueOffTask)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motionCommand == TORQUE_ON , FSM_NEXT(torqueOn));
				FSM_ON_CONDITION(_motionCommand == STANDING , FSM_NEXT(standing));
            }
        }
        FSM_STATE(torqueOn){
            FSM_CALL_TASK(torqueOnTask)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
				FSM_ON_CONDITION(_motionCommand == STANDING , FSM_NEXT(standing));
            }
        }
        FSM_STATE(standing){
            FSM_CALL_TASK(standingTask)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
				FSM_ON_CONDITION(_motionCommand == WALK_FRONT , FSM_NEXT(walkFront));
				FSM_ON_CONDITION(_motionCommand == WALK_BACK  , FSM_NEXT(walkBack));
                FSM_ON_CONDITION(_motionCommand == WALK_RIGHT , FSM_NEXT(walkRight));
                FSM_ON_CONDITION(_motionCommand == WALK_LEFT  , FSM_NEXT(walkLeft));
                FSM_ON_CONDITION(_motionCommand == TURN_RIGHT , FSM_NEXT(turnRight));
                FSM_ON_CONDITION(_motionCommand == TURN_LEFT  , FSM_NEXT(turnLeft));
                FSM_ON_CONDITION(_motionCommand == ATK_FRONT  , FSM_NEXT(atkFront));
                FSM_ON_CONDITION(_motionCommand == ATK_BACK   , FSM_NEXT(atkBack));
                FSM_ON_CONDITION(_motionCommand == ATK_R1     , FSM_NEXT(atkRight1));
                FSM_ON_CONDITION(_motionCommand == ATK_R2     , FSM_NEXT(atkRight2));
				FSM_ON_CONDITION(_motionCommand == ATK_RB1    , FSM_NEXT(atkRightBack1));
				FSM_ON_CONDITION(_motionCommand == ATK_RB2    , FSM_NEXT(atkRightBack2));
				FSM_ON_CONDITION(_motionCommand == ATK_L1     , FSM_NEXT(atkLeft1));
				FSM_ON_CONDITION(_motionCommand == ATK_L2     , FSM_NEXT(atkLeft2));
				FSM_ON_CONDITION(_motionCommand == ATK_LB1    , FSM_NEXT(atkLeftBack1));
				FSM_ON_CONDITION(_motionCommand == ATK_LB2    , FSM_NEXT(atkLeftBack2));
				FSM_ON_CONDITION(_motionCommand == WAKEUP_FRONT && _motionCommandChanged ==1 , FSM_NEXT(wakeupFront));
				FSM_ON_CONDITION(_motionCommand == WAKEUP_BACK && _motionCommandChanged ==1 , FSM_NEXT(wakeupBack));
				FSM_ON_CONDITION(_motionCommand == MOVE_URG && _motionCommandChanged ==1 , FSM_NEXT(moveUrg));
            }
        }
        FSM_STATE(wakeupFront){
            FSM_CALL_TASK(wakeupFrontTask)
            FSM_TRANSITIONS{
                FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(wakeupBack){
            FSM_CALL_TASK(wakeupBackTask)
            FSM_TRANSITIONS{
                FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));

            }
        }


        FSM_STATE(walkFront){
            FSM_CALL_TASK(walkFrontTask)
            FSM_TRANSITIONS{
                FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(walkBack){
            FSM_CALL_TASK(walkBackTask)
            FSM_TRANSITIONS{
                FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(walkRight){
            FSM_CALL_TASK(walkRightTask)
            FSM_TRANSITIONS{
                FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(walkLeft){
            FSM_CALL_TASK(walkLeftTask)
            FSM_TRANSITIONS{
                FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }

        FSM_STATE(turnRight){
            FSM_CALL_TASK(turnRightTask)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(turnLeft){
            FSM_CALL_TASK(turnLeftTask)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }

        FSM_STATE(atkFront){
            FSM_CALL_TASK(atkFrontTask)
            FSM_TRANSITIONS{
                FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(atkBack){
            FSM_CALL_TASK(atkBackTask)
            FSM_TRANSITIONS{
                FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }

        FSM_STATE(atkRight1){
            FSM_CALL_TASK(atkRight1Task)
            FSM_TRANSITIONS{
                FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(atkRight2){
            FSM_CALL_TASK(atkRight2Task)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(atkRightBack1){
            FSM_CALL_TASK(atkRightBack1Task)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(atkRightBack2){
            FSM_CALL_TASK(atkRightBack2Task)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(atkLeft1){
            FSM_CALL_TASK(atkLeft1Task)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(atkLeft2){
            FSM_CALL_TASK(atkLeft2Task)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(atkLeftBack1){
            FSM_CALL_TASK(atkLeftBack1Task)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(atkLeftBack2){
            FSM_CALL_TASK(atkLeftBack2Task)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
                FSM_ON_EVENT("/MOTION_FINISH", FSM_NEXT(standing));
            }
        }
        FSM_STATE(moveUrg){
            FSM_CALL_TASK(moveUrgTask)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION(_motionCommand == TORQUE_OFF , FSM_NEXT(torqueOff));
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
    //ここであえて連続でコマンドを送れるようにした
    if(_motionCommand = STANDING){
        preCommandString = "STANDING";
    }

	if(tmpCommandString  == "TORQUE_OFF" && preCommandString != tmpCommandString){
		_motionCommandChanged = 1;
		_motionCommand = TORQUE_OFF;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "TORQUE_ON" && preCommandString != tmpCommandString){
		_motionCommandChanged = 1;
		_motionCommand = TORQUE_ON;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "STANDING" && preCommandString != tmpCommandString){
		_motionCommandChanged = 1;
		_motionCommand = STANDING;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "WAKEUP_FRONT" && preCommandString != tmpCommandString){
		_motionCommandChanged = 1;
		_motionCommand = WAKEUP_FRONT;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "WAKEUP_BACK" && preCommandString != tmpCommandString){
		_motionCommandChanged = 1;
		_motionCommand = WAKEUP_BACK;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "WALK_FRONT" && preCommandString != tmpCommandString){
        _motionCommandChanged = 1;
        _motionCommand = WALK_FRONT;
        ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
    }else if(tmpCommandString  == "WALK_BACK" && preCommandString != tmpCommandString){
        _motionCommandChanged = 1;
        _motionCommand = WALK_BACK;
        ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
    }else if(tmpCommandString  == "WALK_RIGHT" && preCommandString != tmpCommandString){
        _motionCommandChanged = 1;
        _motionCommand = WALK_RIGHT;
        ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
    }else if(tmpCommandString  == "WALK_LEFT" && preCommandString != tmpCommandString){
        _motionCommandChanged = 1;
        _motionCommand = WALK_LEFT;
        ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
    }else if(tmpCommandString  == "TURN_RIGHT" && preCommandString != tmpCommandString){
		_motionCommandChanged = 1;
		_motionCommand = TURN_RIGHT;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "TURN_LEFT" && preCommandString != tmpCommandString){
		_motionCommandChanged = 1;
		_motionCommand = TURN_LEFT;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "ATK_FRONT" && preCommandString != tmpCommandString){
        _motionCommandChanged = 1;
        _motionCommand = ATK_FRONT;
        ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
    }else if(tmpCommandString  == "ATK_BACK" && preCommandString != tmpCommandString){
        _motionCommandChanged = 1;
        _motionCommand = ATK_BACK;
        ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
    }else if(tmpCommandString  == "ATK_R1" && preCommandString != tmpCommandString){
        _motionCommandChanged = 1;
        _motionCommand = ATK_R1;
        ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
    }else if(tmpCommandString  == "ATK_R2" && preCommandString != tmpCommandString){
        _motionCommandChanged = 1;
        _motionCommand = ATK_R2;
        ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
    }else if(tmpCommandString  == "ATK_RB1" && preCommandString != tmpCommandString){
		_motionCommandChanged = 1;
		_motionCommand = ATK_RB1;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "ATK_RB2" && preCommandString != tmpCommandString){
		_motionCommandChanged = 1;
		_motionCommand = ATK_RB2;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "ATK_L1" && preCommandString != tmpCommandString){
		_motionCommandChanged = 1;
		_motionCommand = ATK_L1;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "ATK_L2" && preCommandString != tmpCommandString){
		_motionCommandChanged = 1;
		_motionCommand = ATK_L2;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "ATK_LB1" && preCommandString != tmpCommandString){
		_motionCommandChanged = 1;
		_motionCommand = ATK_LB1;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}else if(tmpCommandString  == "ATK_LB2" && preCommandString != tmpCommandString){
		_motionCommandChanged = 1;
		_motionCommand = ATK_LB2;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}

	else if(tmpCommandString  == "MOVE_URG" && preCommandString != tmpCommandString){
		_motionCommandChanged = 1;
		_motionCommand = MOVE_URG;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}

	else if(preCommandString != tmpCommandString){
		_motionCommandChanged = 1;
		_motionCommand = STANDING;
	}

	preCommandString = tmpCommandString;
}


//tasks
decision_making::TaskResult torqueOnCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("torqueOnTask...");
    ginko_player_.torqueEnable(1);
    usleep(100000);
    ginko_player_.torqueEnable(0);
    //usleep(100000);
    sleep(1);
    ginko_player_.torqueEnable(1);
    usleep(100000);

    ginko_player_.playPose(standing_Motion_Start,0);

    eventQueue.riseEvent("/MOTION_FINISH");
    _motionCommand = STANDING;
    _motionCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult torqueOffCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("torqueOffTask...");
//    sleep(1);
    ginko_player_.torqueEnable(0);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motionCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult standingCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("standingTask...");
//    sleep(1);
//    ginko_player_.playPose(standing_Motion_Start,0);
    ginko_player_.playPose(standing_Motion_Loop,0);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motionCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult wakeupFrontCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("wakeupFrontTask...");
    ginko_player_.playMotion(wakeupFront_Motion_Start);
    ginko_timer_.msleepCyclic(1000);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motionCommand = STANDING;
    _motionCommandChanged = 1;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult wakeupBackCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("wakeupBackTask...");
    ginko_player_.playMotion(wakeupBack_Motion_Start);
    ginko_timer_.msleepCyclic(1000);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motionCommand = STANDING;
    _motionCommandChanged = 1;
    return TaskResult::SUCCESS();
}


decision_making::TaskResult walkFrontCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("walkFrontStart...");
    ginko_player_.playMotion(walkFront_Motion_Start);
    while(_motionCommand == WALK_FRONT){
        ROS_INFO("walkFrontLoop...");
        ginko_player_.playMotion(walkFront_Motion_Loop);
    }
    if(_motionCommand != TORQUE_OFF){
        ROS_INFO("walkFrontEnd...");
        ginko_player_.playMotion(walkFront_Motion_End);
    }
    ginko_timer_.msleepCyclic(1000);
    eventQueue.riseEvent("/MOTION_FINISH");
    // _motionCommand = STANDING;
    _motionCommandChanged = 1;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult walkBackCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("walkBackStart...");
    ginko_player_.playMotion(walkBack_Motion_Start);
    while(_motionCommand == WALK_BACK){
        ROS_INFO("walkBackLoop...");
        ginko_player_.playMotion(walkBack_Motion_Loop);
    }
    if(_motionCommand != TORQUE_OFF){
        ROS_INFO("walkBackEnd...");
        ginko_player_.playMotion(walkBack_Motion_End);
    }
    ginko_timer_.msleepCyclic(1000);
    eventQueue.riseEvent("/MOTION_FINISH");
    // _motionCommand = STANDING;
    _motionCommandChanged = 1;
    return TaskResult::SUCCESS();
}

decision_making::TaskResult walkRightCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("walkRightStart...");
    ginko_player_.playMotion(walkRight_Motion_Start);
    while(_motionCommand == WALK_RIGHT){
        ROS_INFO("walkRightLoop...");
        ginko_player_.playMotion(walkRight_Motion_Loop);
    }
    if(_motionCommand != TORQUE_OFF){
        ROS_INFO("walkRightEnd...");
        ginko_player_.playMotion(walkRight_Motion_End);
    }
    ginko_timer_.msleepCyclic(1000);
    eventQueue.riseEvent("/MOTION_FINISH");
    // _motionCommand = STANDING;
    _motionCommandChanged = 1;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult walkLeftCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("walkLeftStart...");
    ginko_player_.playMotion(walkLeft_Motion_Start);
    while(_motionCommand == WALK_LEFT){
        ROS_INFO("walkLeftLoop...");
        ginko_player_.playMotion(walkLeft_Motion_Loop);
    }
    if(_motionCommand != TORQUE_OFF){
        ROS_INFO("walkLeftEnd...");
        ginko_player_.playMotion(walkLeft_Motion_End);
    }
    ginko_timer_.msleepCyclic(1000);
    eventQueue.riseEvent("/MOTION_FINISH");
    // _motionCommand = STANDING;
    _motionCommandChanged = 1;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult turnRightCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("turnRightStart...");
    ginko_player_.playMotion(turnRight_Motion_Start);
    while(_motionCommand == TURN_RIGHT){
        ROS_INFO("turnRightLoop...");
        ginko_player_.playMotion(turnRight_Motion_Loop);
    }
    if(_motionCommand != TORQUE_OFF){
        ROS_INFO("turnRightEnd...");
        ginko_player_.playMotion(turnRight_Motion_End);
    }
    ginko_timer_.msleepCyclic(1000);
    eventQueue.riseEvent("/MOTION_FINISH");
    // _motionCommand = STANDING;
    _motionCommandChanged = 1;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult turnLeftCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("turnLeftStart...");
    ginko_player_.playMotion(turnLeft_Motion_Start);
    while(_motionCommand == TURN_LEFT){
        ROS_INFO("turnLeftLoop...");
        ginko_player_.playMotion(turnLeft_Motion_Loop);
    }
    if(_motionCommand != TORQUE_OFF){
        ROS_INFO("turnLeftEnd...");
        ginko_player_.playMotion(turnLeft_Motion_End);
    }
    ginko_timer_.msleepCyclic(1000);
    eventQueue.riseEvent("/MOTION_FINISH");
    // _motionCommand = STANDING;
    _motionCommandChanged = 1;
    return TaskResult::SUCCESS();
}

decision_making::TaskResult atkFrontCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("atkFrontTask...");
    ginko_player_.playMotion(atkFront_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motionCommand = STANDING;
    _motionCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult atkBackCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("atkBackTask...");
    ginko_player_.playMotion(atkBack_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motionCommand = STANDING;
    _motionCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult atkRight1Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("atkRight1Task...");
    ginko_player_.playMotion(atkRight1_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motionCommand = STANDING;
    _motionCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult atkRight2Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("atkRight2Task...");
    ginko_player_.playMotion(atkRight2_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motionCommand = STANDING;
    _motionCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult atkRightBack1Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("atkRightBack1Task...");
    ginko_player_.playMotion(atkRightBack1_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motionCommand = STANDING;
    _motionCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult atkRightBack2Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("atkRightBack2Task...");
    ginko_player_.playMotion(atkRightBack2_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motionCommand = STANDING;
    _motionCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult atkLeft1Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("atkLeft1Task...");
    ginko_player_.playMotion(atkLeft1_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motionCommand = STANDING;
    _motionCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult atkLeft2Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("atkLeft2Task...");
    ginko_player_.playMotion(atkLeft2_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motionCommand = STANDING;
    _motionCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult atkLeftBack1Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("atkLeftBack1Task...");
    ginko_player_.playMotion(atkLeftBack1_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motionCommand = STANDING;
    _motionCommandChanged = 0;
    return TaskResult::SUCCESS();
}
decision_making::TaskResult atkLeftBack2Callback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("atkLeftBack2Task...");
    ginko_player_.playMotion(atkLeftBack2_Motion_Start);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motionCommand = STANDING;
    _motionCommandChanged = 0;
    return TaskResult::SUCCESS();
}
/*
decision_making::TaskResult moveUrgCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("moveUrgStart...");
    ginko_player_.playMotion(move_urg3_Motion_Loop);
    eventQueue.riseEvent("/MOTION_FINISH");
    _motionCommandChanged = 0;
    return TaskResult::SUCCESS();
}
*/

decision_making::TaskResult moveUrgCallback(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("moveUrgStart...");
//    ginko_player_.playMotion(walkFront_Motion_Start);
    while(_motionCommand == MOVE_URG){
        ROS_INFO("moveUrgLoop...");
//        ginko_player_.playMotion(move_urg2_Motion_Loop);
        ginko_player_.playMotion(move_urg3_Motion_Loop);
    }
    if(_motionCommand != TORQUE_OFF){
        ROS_INFO("moveUrgEnd...");
//        ginko_player_.playMotion(walkFront_Motion_End);
    }
    ginko_timer_.msleepCyclic(1000);
    eventQueue.riseEvent("/MOTION_FINISH");
    // _motionCommand = STANDING;
    _motionCommandChanged = 1;
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
	_motionCommandSub = node.subscribe("/motion_command",10,&motionCommandCallback,transport_hints);
	//init timer
    ginko_timer_.usecStart();
    ginko_player_.initPublisher();
	//Tasks registration
    LocalTasks::registrate("torqueOnTask",	torqueOnCallback);
    LocalTasks::registrate("torqueOffTask",	torqueOffCallback);
    LocalTasks::registrate("standingTask",	standingCallback);
    LocalTasks::registrate("wakeupFrontTask",	wakeupFrontCallback);
    LocalTasks::registrate("wakeupBackTask",	wakeupBackCallback);
    LocalTasks::registrate("walkFrontTask", walkFrontCallback);
    LocalTasks::registrate("walkBackTask",  walkBackCallback);
    LocalTasks::registrate("walkRightTask", walkRightCallback);
    LocalTasks::registrate("walkLeftTask",  walkLeftCallback);
    LocalTasks::registrate("turnRightTask", turnRightCallback);
    LocalTasks::registrate("turnLeftTask",  turnLeftCallback);
    LocalTasks::registrate("moveUrgTask",	moveUrgCallback);
    LocalTasks::registrate("atkFrontTask", atkFrontCallback);
    LocalTasks::registrate("atkBackTask", atkBackCallback);
    LocalTasks::registrate("atkRight1Task", atkRight1Callback);
    LocalTasks::registrate("atkRight2Task", atkRight2Callback);
    LocalTasks::registrate("atkRightBack1Task",	atkRightBack1Callback);
    LocalTasks::registrate("atkRightBack2Task",	atkRightBack2Callback);
    LocalTasks::registrate("atkLeft1Task",	atkLeft1Callback);
    LocalTasks::registrate("atkLeft2Task",	atkLeft2Callback);
    LocalTasks::registrate("atkLeftBack1Task",	atkLeftBack1Callback);
    LocalTasks::registrate("atkLeftBack2Task",	atkLeftBack2Callback);
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

