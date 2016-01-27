/*
 *    Copyright (C) 2015 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <Aria.h>
#include <ArNetworking.h>
#include <cmath>
#include <string>




void handleGoalList(ArNetPacket *packet);
void handleGetRobotPosition(ArNetPacket *packet);
void handleGoalPositionList(ArNetPacket *packet);
void handlePathPlannerStatus(ArNetPacket *packet);
float distanciaPuntos(float pxa, float pya, float pxb, float pyb);

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	int checkGoalDone();
	int checkPathPlannerStatus();
	int getGoalPositions();
	int setSafeDrive(const bool &activate);
	int getInfo();
	int getRobotPosition();
	int wanderMode(const bool &activate);
	int stopRobot();
	int getGoals();
	int goToGoal(const string &name);
	int goToPose(const personPose &p);
	

public slots:
	void compute(); 

private:
	
};

#endif

