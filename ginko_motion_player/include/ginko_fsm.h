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

#ifndef GINKO_MOTION_H
#define GINKO_MOTION_H


#include <iostream>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <random_numbers/random_numbers.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>



#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>


#include "ginko_timer.h"
#include "ginko_player.h"
#include "standing.h"
#include "walkFront.h"
#include "walkBack.h"
#include "turnRight.h"
#include "turnLeft.h"
#include "wakeupFront.h"
#include "wakeupBack.h"

#include "attackRight1.h"
#include "attackRight2.h"
#include "attackRightBack1.h"
#include "attackRightBack2.h"
#include "attackLeft1.h"
#include "attackLeft2.h"
#include "attackLeftBack1.h"
#include "attackLeftBack2.h"

#include "moveUrg1.h"
#include "moveUrg2.h"
#include "moveUrg3.h"

namespace ginko_fsm {
using namespace std;
using namespace decision_making;
using namespace ginko_timer;
using namespace ginko_player;


#define foreach BOOST_FOREACH




//ginko_joint_controllerのGinkoControllerと同じ
#define LOOP_FREQUENCY  (30)
#define SERVO_NUM     25


}




#endif //GINKO_MOTION_H
