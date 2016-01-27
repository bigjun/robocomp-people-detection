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
#include "specificworker.h"


bool isConnectedToServer;	//Si ESTE servidor esta conectado al Servidor ArNetworking del robot.
ArClientBase client;		//Cliente ArNetworking, punto de enlace con el robot
mapPose robotPosition;
mapPose goalPosition;
personPose personPosition;
MapInfo mapInfo;
GoalInfo goalInfo;


void handleGoalList(ArNetPacket *packet)
{
	cout << "goalListHandler" << endl;
	mapInfo.numGoals = 0;
	std::cout << ".. Server has these goals:" << std::endl;
	char goal[256];
	for(int i = 0; packet->getReadLength() < packet->getLength(); i++)
	{
		packet->bufToStr(goal, 255);
		if(strlen(goal) == 0)
			return;
		mapInfo.listGoals.push_back(goal);
		std::cout << goal << std::endl;
		mapInfo.numGoals++;
		std::cout << mapInfo.numGoals << std::endl;
	}
}


void handleGetRobotPosition(ArNetPacket *packet)
{
	//cout << "getRobotPositionHandler" << endl;
	double robAux;

	//std::cout << ".. Robot position:" << std::endl;

	robAux = packet->bufToDouble();
	robotPosition.X = robAux;

	robAux = packet->bufToDouble();
	robotPosition.Y = robAux;

	robAux = packet->bufToDouble();
	robotPosition.Th = robAux;

	//std::cout << robotPosition.getX() << ", " << robotPosition.getY() << ", " << robotPosition.getTh() << std::endl;
}


void handleGoalPositionList(ArNetPacket *packet)
{
	cout << "goalPositionListHandler" << endl;
	double goal;

	std::cout << ".. Goal position:" << std::endl;
	for(int i = 0; i < mapInfo.numGoals; i++)
	{
		goal = packet->bufToDouble();
		goalPosition.X = goal;
		goal = packet->bufToDouble();
		goalPosition.Y = goal;
		mapInfo.goalPosition.push_back(goalPosition);


		std::cout << mapInfo.goalPosition[i].X << ", " << mapInfo.goalPosition[i].Y << std::endl;
	}
}



void handlePathPlannerStatus(ArNetPacket *packet)
{
	cout << "pathPlannerStatusHandler" << endl;
	goalInfo.goalState = packet->bufToDouble();
	cout << "State: " << goalInfo.goalState << endl;
}



float distanciaPuntos(float pxa, float pya, float pxb, float pyb)
{
	return ( abs((max(pxa,pxb) - min(pxa,pxb))) + abs((max(pya,pyb) - min(pya,pyb))) );
}



/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	bool errors = false;//Variable usada para controlar errores. Evitamos asi que el servidor robocomp se cierre debido a Aria
	isConnectedToServer = false;

	Aria::init();

	//Mejorar para pedir estos datos al usuario (excepto el primer parametro)
	char *parameters[] = {"robocomp","-host","localhost"};
	int numParameters = 3;

	ArArgumentParser parser(&numParameters,parameters);

	ArClientSimpleConnector clientConnector(&parser);

	parser.loadDefaultArguments();

	if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed()){
		Aria::logOptions();
		errors=true;
	}

	if (!errors && !clientConnector.connectClient(&client)){
		if (client.wasRejected())
			printf("Server '%s' rejected connection\n", client.getHost());
		else
			printf("Could not connect to server '%s'\n", client.getHost());
		errors=true;
	}

	client.setRobotName(client.getHost());

	//NEW
	client.addHandler("getGoals", new ArGlobalFunctor1<ArNetPacket*>(&handleGoalList));
	client.addHandler("getRobotPosition", new ArGlobalFunctor1<ArNetPacket*>(&handleGetRobotPosition));
	client.addHandler("getGoalPositions", new ArGlobalFunctor1<ArNetPacket*>(&handleGoalPositionList));
	client.addHandler("goalReached", new ArGlobalFunctor1<ArNetPacket*>(&handlePathPlannerStatus));

	client.runAsync();

	isConnectedToServer = true;

	printf("Connected to server.\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{

}


int SpecificWorker::checkGoalDone()
{
	ArNetPacket packet;
	(&client)->requestOnce("getRobotPosition", &packet);
	if(distanciaPuntos(robotPosition.X, robotPosition.Y, mapInfo.goalPosition[goalInfo.currentGoal].X, mapInfo.goalPosition[goalInfo.currentGoal].Y) < 50){
		std::cout << "HA LLEGADO!!!!!!" << std::endl;
		return 1;
	}else{
		return 0;
	}
}

int SpecificWorker::checkPathPlannerStatus()
{
	return (int)goalInfo.goalState;
}

int SpecificWorker::getGoalPositions()
{
	ArNetPacket packet;
	(&client)->requestOnce("getGoalPositions", &packet);
	return 0;
}

int SpecificWorker::setSafeDrive(const bool &activate)
{
	int value=1;
	if(!activate)value=0;
	ArNetPacket packet;
	packet.byteToBuf(value);
	(&client)->requestOnce("setSafeDrive",&packet);
	return 0;
}

int SpecificWorker::getInfo()
{
	ArNetPacket packet;
	(&client)->requestOnce("getGoals", &packet);
	(&client)->requestOnce("getGoalPositions", &packet);
	(&client)->request("goalReached", 500, &packet);
	ArUtil::sleep(500);
	ariamapinformation_proxy->mapInformation(mapInfo);
	return 0;	
}

int SpecificWorker::getRobotPosition()
{
	ArNetPacket packet;
	(&client)->requestOnce("getRobotPosition", &packet);
	return 0;
}

int SpecificWorker::wanderMode(const bool &activate)
{
	int value=1;
	if(!activate)value=0;
	ArNetPacket packet;
	packet.byteToBuf(value);
	(&client)->requestOnce("wander",&packet);
	return 0;
}

int SpecificWorker::stopRobot()
{
	//Change this because the robot is only setting the safe drive mode
	(&client)->requestOnce("setSafeDrive");
	return 0;
}

int SpecificWorker::getGoals()
{
	ArNetPacket packet;
	(&client)->requestOnce("getGoals", &packet);
	return 0;
}

int SpecificWorker::goToGoal(const string &name)
{
	ArNetPacket packet;
	const char *cstr = name.c_str();
	packet.strToBuf(cstr);
	std::cout << cstr;
	(&client)->requestOnce("gotoGoal", &packet);
	for(int i = 0; i < mapInfo.numGoals; i++){
		if(strcmp(mapInfo.listGoals[i].c_str(),cstr) == 0){
			goalInfo.currentGoal = i;
			break;
		}
	}
	cout << "Current goal: " << mapInfo.listGoals[goalInfo.currentGoal] << endl;
	return 0;
}

int SpecificWorker::goToPose(const personPose &p)
{		
	ArNetPacket packet;
	packet.byte4ToBuf(p.x);
    packet.byte4ToBuf(p.y);
    packet.byte4ToBuf(p.th);
	(&client)->requestOnce("gotoPose", &packet);
	
	cout << "Going to the pose: (" << p.x << ", " << p.y << ")" << endl;
	return 0;
}






