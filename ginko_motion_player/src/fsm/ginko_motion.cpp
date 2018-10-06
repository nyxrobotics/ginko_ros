/*******************************************************************************
 * Copyright 2016 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include "ginko_motion.h"

//using namespace ginko_motion;
string _motiomCommand = "TorqueFree";
unsigned char _motiomCommandChanged = 0;

ros::Subscriber _motiomCommandSub;
ros::Publisher _PosePublisher;

//random_numbers::RandomNumberGenerator _randomizer;
//ros::Publisher _velocityPublisher;
/*************************************************************************************************
*** Final state machine
**************************************************************************************************/

FSM(Motion){
    FSM_STATES{
    	TorqueFree,
		TorqueOn
//		Standing,
//		WakeupFront,
//		WakeupBack,
//
//		WalkFront,
//		WalkBack,
//		WalkRiht,
//		WalkLeft,
//
//		TurnRiht,
//		TurnLeft,
//
//		AtkFrontRHigh,
//		AtkFrontRLow,
//		AtkFrontLHigh,
//		AtkFrontLLow,
//
//		AtkBackRHigh,
//		AtkBackRLow,
//		AtkBackRLHigh,
//		AtkBackRLLow,

    }
    FSM_START(TorqueFree)
    FSM_BGN{
        FSM_STATE(TorqueFree){
            FSM_CALL_TASK(TorqueFree)
            FSM_TRANSITIONS{
              FSM_ON_EVENT("/TORQUE_ENABLE", FSM_NEXT(TorqueOn));

				FSM_ON_CONDITION( getMotionCommand() == "TorqueOn" && getCommandChanged() == 1 , FSM_NEXT(TorqueOn));
//				FSM_ON_EVENT("/STANDING", FSM_NEXT(Standing));
            }
        }
        FSM_STATE(TorqueOn){
            FSM_CALL_TASK(TorqueOn)
            FSM_TRANSITIONS{
				FSM_ON_CONDITION( getMotionCommand() == "TorqueFree" && getCommandChanged() == 1 , FSM_NEXT(TorqueFree));
            }
        }

    }
    FSM_END
}


/*************************************************************************************************
*** ROS Subscriptions
**************************************************************************************************/
//void initSubscriber() {
//	_motiomCommandSub = node_handle_.subscribe("/motion_command", 100,&motionCommandCallback);
//}

void motionCommandCallback(const std_msgs::String::ConstPtr& msg)
{
	static string prevCommand = "TorqueFree";
	_motiomCommand = msg->data.c_str();
	if(prevCommand != _motiomCommand){
		_motiomCommandChanged = 1;
	    ROS_INFO("GetMotionCommand: [%s]", msg->data.c_str());
	}

}
string getMotionCommand(void){
	return _motiomCommand;
}

unsigned char getCommandChanged(void){
	return _motiomCommandChanged;
}

/*************************************************************************************************
*** Task implementations
**************************************************************************************************/

decision_making::TaskResult torqueFreeTask(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("torqueFreeTask...");

    for (int i = 0; i < 100; ++i) {
        if (eventQueue.isTerminated()) {
            ROS_INFO("Obstacle!");
            return TaskResult::TERMINATED();
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }

//    eventQueue.riseEvent("/DRIVE_TIMEOUT");
    return TaskResult::SUCCESS();
}

decision_making::TaskResult torqueOnTask(string name, const FSMCallContext& context, EventQueue& eventQueue) {
    ROS_INFO("torqueOnTask...");

    for (int i = 0; i < 100; ++i) {
        if (eventQueue.isTerminated()) {
            ROS_INFO("Obstacle!");
            return TaskResult::TERMINATED();
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }

//    eventQueue.riseEvent("/DRIVE_TIMEOUT");
    return TaskResult::SUCCESS();
}


/*************************************************************************************************
*** The Main
**************************************************************************************************/

int main(int argc, char **argv) {
    /**
     * Initialization
     */
    ros::init(argc, argv, "fsm_ginko");
    ros_decision_making_init(argc, argv);
    ros::NodeHandle node_handle_("~");
    RosEventQueue eventQueue;


    /**
     * Tasks registration
     */
    LocalTasks::registrate("TorqueFree", torqueFreeTask);
    LocalTasks::registrate("TorqueOn", torqueOnTask);

    /**
     * Subscription for the laser topic and velocity publisher creation
     */
    _motiomCommandSub = node_handle_.subscribe("/motion_command", 100,&motionCommandCallback);
//    initSubscriber();



    /**
     * ROS Spinner for topic subscriptions
     */
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /**
     * Execution of the FSM
     */
    ROS_INFO("Starting wandering machine...");
    FsmMotion(NULL, &eventQueue);

    /**
     * Cleanup
     */
	return 0;
}
