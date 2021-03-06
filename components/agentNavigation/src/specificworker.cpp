/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	#ifdef USE_QTGUI	
		//Buttons with Qt	
		QWidget * wdg = new QWidget(this);
		QVBoxLayout *vlay = new QVBoxLayout(wdg);
		QPushButton *btn1 = new QPushButton("goToPerson",this);
		vlay->addWidget(btn1);		
		QPushButton *btn2 = new QPushButton("reachPerson",this);
		vlay->addWidget(btn2);
		
		//Connection of the buttons with the functionalities
	    connect(btn1, SIGNAL (clicked()), this, SLOT (goToPerson()));
	    connect(btn2, SIGNAL (clicked()), this, SLOT (reachPerson()));
		wdg->setLayout(vlay);
		//setCentralWidget(wdg);
		
		setupUi(this);
		show();
	#endif

	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

#ifdef USE_QTGUI
void SpecificWorker::goToGoal()
{
	//AGMModelPrinter::printWorld(worldModel);
	
	bool edgeYes = false, edgeStopped = false, edgeUnvisited = false;		
	int robotId, homePointId, robotStateId, goalId;
	AGMModelSymbol::SPtr node1, node2;
	
	robotId = worldModel->getIdentifierByType("robot");
	homePointId =  worldModel->getIdentifierByType("homePoint");
	robotStateId =  worldModel->getIdentifierByType("robotState");
	goalId =  worldModel->getIdentifierByType("goal");
	/*printf("robotId: %d \n",robotId);
	printf("homePointId: %d \n",homePointId);
	printf("robotStateId: %d \n",robotStateId);
	printf("goalId: %d \n",goalId);*/
	
	if(robotId>0 && homePointId>0 && goalId>0){
		for (std::vector<AGMModelEdge>::iterator it = worldModel->edges.begin() ; it != worldModel->edges.end(); ++it)
		{
			if ((*it)->getLabel() == "yes"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == homePointId) || (node2->identifier == robotId && node1->identifier == homePointId)){
					edgeYes = true;
				}
			}
			else if((*it)->getLabel() == "stopped"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == robotStateId) || (node2->identifier == robotId && node1->identifier == robotStateId)){
					edgeStopped = true;
				}
			}
			else if((*it)->getLabel() == "unvisited"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == goalId) || (node2->identifier == robotId && node1->identifier == goalId)){
					goalDestination = (*it)->getAttribute("goal");
					//cout << "GOAL DESTINATION: " << goalDestination << endl;
					edgeUnvisited = true;
				}
			}
		}
	}
		
	
	if(edgeYes && edgeUnvisited && edgeStopped && waiting)
	{
		
		if(waiting){
			arianavigation_proxy->goToGoal(goalDestination);
			waiting = false;
			sleep(2);	
			
			/*************************NOTE****************
			* Comprobar si no puede ir a ese goal, ¿qué hacer?:
			*	- Volvere al estado inicial?
			*	- Ir a un estado del mundo intermedio
			*	- Limbo
			*********************************************/
			QMutexLocker locker(mutex);
		
			AGMModel::SPtr newModel(new AGMModel(worldModel));			

			//Editar el edge
			AGMModelSymbol::SPtr nodeRobot = newModel->getSymbol(robotId);
			AGMModelSymbol::SPtr nodeHomePoint = newModel->getSymbol(homePointId);
			AGMModelSymbol::SPtr nodeRobotState = newModel->getSymbol(robotStateId);
			newModel->removeEdge(nodeRobot, nodeHomePoint, "yes");
			newModel->removeEdge(nodeRobot, nodeRobotState, "stopped");
			newModel->addEdgeByIdentifiers(robotId,homePointId,"no");
			newModel->addEdgeByIdentifiers(robotId,robotStateId,"moving");
		 	
	 		//AGMModelPrinter::printWorld(newModel);
	
			//qDebug()<<"----------- MODIFICATION -----------------------";
			sendModificationProposal(worldModel, newModel);			
		
	 		//AGMModelPrinter::printWorld(worldModel);	
		}
	}
}

void SpecificWorker::reachGoal()
{
	//AGMModelPrinter::printWorld(worldModel);
	
	bool edgeNo = false, edgeMoving = false, edgeUnvisited = false;		
	int robotId, homePointId, robotStateId, goalId;
	AGMModelSymbol::SPtr node1, node2;
	
	robotId = worldModel->getIdentifierByType("robot");
	homePointId =  worldModel->getIdentifierByType("homePoint");
	robotStateId =  worldModel->getIdentifierByType("robotState");
	goalId =  worldModel->getIdentifierByType("goal");
	/*printf("robotId: %d \n",robotId);
	printf("homePointId: %d \n",homePointId);
	printf("robotStateId: %d \n",robotStateId);
	printf("goalId: %d \n",goalId);*/
	
	if(robotId>0 && homePointId>0 && robotStateId>0 && goalId>0){
		for (std::vector<AGMModelEdge>::iterator it = worldModel->edges.begin() ; it != worldModel->edges.end(); ++it)
		{
			if ((*it)->getLabel() == "no"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == homePointId) || (node2->identifier == robotId && node1->identifier == homePointId)){
					edgeNo = true;
				}
			}
			else if((*it)->getLabel() == "moving"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == robotStateId) || (node2->identifier == robotId && node1->identifier == robotStateId)){
					edgeMoving = true;
				}
			}
			else if((*it)->getLabel() == "unvisited"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == goalId) || (node2->identifier == robotId && node1->identifier == goalId)){
					edgeUnvisited = true;
				}
			}
		}
	}
		
	
	if(edgeNo && edgeUnvisited && edgeMoving && !goalReached)
	{
		checkGoalReached();
		
		if(goalReached){
			waiting = true;
			QMutexLocker locker(mutex);
		
			AGMModel::SPtr newModel(new AGMModel(worldModel));			

			//Editar el edge
			AGMModelSymbol::SPtr nodeRobot = newModel->getSymbol(robotId);
			AGMModelSymbol::SPtr nodeGoal = newModel->getSymbol(goalId);
			AGMModelSymbol::SPtr nodeRobotState = newModel->getSymbol(robotStateId);
			newModel->removeEdge(nodeRobot, nodeGoal, "unvisited");
			newModel->addEdgeByIdentifiers(robotId,goalId,"visited");
			newModel->removeEdge(nodeRobot, nodeRobotState, "moving");
			newModel->addEdgeByIdentifiers(robotId,robotStateId,"stopped");
		 	
	 		//AGMModelPrinter::printWorld(newModel);
	
			//qDebug()<<"----------- MODIFICATION -----------------------";
			sendModificationProposal(worldModel, newModel);	
			goalReached = false;		
		
	 		//AGMModelPrinter::printWorld(worldModel);
		}			
	}
}


void SpecificWorker::goToHome()
{
	//AGMModelPrinter::printWorld(worldModel);
	
	bool edgeNo = false, edgeStopped = false, edgeVisited = false;		
	int robotId, homePointId, robotStateId, goalId;
	AGMModelSymbol::SPtr node1, node2;
	
	robotId = worldModel->getIdentifierByType("robot");
	homePointId =  worldModel->getIdentifierByType("homePoint");
	robotStateId =  worldModel->getIdentifierByType("robotState");
	goalId =  worldModel->getIdentifierByType("goal");
	/*printf("robotId: %d \n",robotId);
	printf("homePointId: %d \n",homePointId);
	printf("robotStateId: %d \n",robotStateId);
	printf("goalId: %d \n",goalId);*/
	
	if(robotId>0 && homePointId>0 && robotStateId>0 && goalId>0){
		for (std::vector<AGMModelEdge>::iterator it = worldModel->edges.begin() ; it != worldModel->edges.end(); ++it)
		{
			if ((*it)->getLabel() == "no"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == homePointId) || (node2->identifier == robotId && node1->identifier == homePointId)){
					edgeNo = true;
				}
			}
			else if((*it)->getLabel() == "stopped"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == robotStateId) || (node2->identifier == robotId && node1->identifier == robotStateId)){
					edgeStopped = true;
				}
			}
			else if((*it)->getLabel() == "visited"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == goalId) || (node2->identifier == robotId && node1->identifier == goalId)){
					edgeVisited = true;
				}
			}
		}
	}
		
	
	if(edgeNo && edgeVisited && edgeStopped && waiting)
	{
		
		if(waiting){
			arianavigation_proxy->goToGoal("homePoint");
			waiting = false;
			sleep(2);	
			cout << "GOTOHOME - goalReached: " << goalReached << endl; 
			/*************************NOTE****************
			* Comprobar si no puede ir al home, ¿qué hacer?:
			*	- Volvere al estado inicial?
			*	- Ir a un estado del mundo intermedio
			*	- Limbo
			*********************************************/
		
			QMutexLocker locker(mutex);
		
			AGMModel::SPtr newModel(new AGMModel(worldModel));			

			//Editar el edge
			AGMModelSymbol::SPtr nodeRobot = newModel->getSymbol(robotId);
			AGMModelSymbol::SPtr nodehomePoint = newModel->getSymbol(homePointId);
			AGMModelSymbol::SPtr nodeRobotState = newModel->getSymbol(robotStateId);
			newModel->removeEdge(nodeRobot, nodeRobotState, "stopped");
			newModel->addEdgeByIdentifiers(robotId,robotStateId,"moving");		
		 	
	 		//AGMModelPrinter::printWorld(newModel);
	
			//qDebug()<<"----------- MODIFICATION -----------------------";
			sendModificationProposal(worldModel, newModel);			
		
	 		//AGMModelPrinter::printWorld(worldModel);
	 	}
	}
}

void SpecificWorker::reachHome()
{
	//AGMModelPrinter::printWorld(worldModel);
	
	bool edgeNo = false, edgeMoving = false, edgeVisited = false;		
	int robotId, homePointId, robotStateId, goalId;
	AGMModelSymbol::SPtr node1, node2;
	
	robotId = worldModel->getIdentifierByType("robot");
	homePointId =  worldModel->getIdentifierByType("homePoint");
	robotStateId =  worldModel->getIdentifierByType("robotState");
	goalId =  worldModel->getIdentifierByType("goal");
	/*printf("robotId: %d \n",robotId);
	printf("homePointId: %d \n",homePointId);
	printf("robotStateId: %d \n",robotStateId);
	printf("goalId: %d \n",goalId);*/
	
	if(robotId>0 && homePointId>0 && robotStateId>0 && goalId>0){
		for (std::vector<AGMModelEdge>::iterator it = worldModel->edges.begin() ; it != worldModel->edges.end(); ++it)
		{
			if ((*it)->getLabel() == "no"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == homePointId) || (node2->identifier == robotId && node1->identifier == homePointId)){
					edgeNo = true;
				}
			}
			else if((*it)->getLabel() == "moving"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == robotStateId) || (node2->identifier == robotId && node1->identifier == robotStateId)){
					edgeMoving = true;
				}
			}
			else if((*it)->getLabel() == "visited"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == goalId) || (node2->identifier == robotId && node1->identifier == goalId)){
					edgeVisited = true;
				}
			}
		}
	}
		
	
	if(edgeNo && edgeVisited && edgeMoving && !goalReached)
	{
		checkGoalReached();
		
		if(goalReached){		
			cout << "REACHHOME - goalReached: " << goalReached << endl; 
			goalReached = false;
			waiting = true;
			QMutexLocker locker(mutex);
		
			AGMModel::SPtr newModel(new AGMModel(worldModel));			

			//Editar el edge
			AGMModelSymbol::SPtr nodeRobot = newModel->getSymbol(robotId);
			AGMModelSymbol::SPtr nodeHomePoint = newModel->getSymbol(homePointId);
			AGMModelSymbol::SPtr nodeRobotState = newModel->getSymbol(robotStateId);
			newModel->removeEdge(nodeRobot, nodeHomePoint, "no");
			newModel->addEdgeByIdentifiers(robotId,homePointId,"yes");
			newModel->removeEdge(nodeRobot, nodeRobotState, "moving");
			newModel->addEdgeByIdentifiers(robotId,robotStateId,"stopped");
		 	
	 		//AGMModelPrinter::printWorld(newModel);
	
			//qDebug()<<"----------- MODIFICATION -----------------------";
			sendModificationProposal(worldModel, newModel);			
		
	 		//AGMModelPrinter::printWorld(worldModel);
		}			
	}
}

void SpecificWorker::backToInitial(){
	//AGMModelPrinter::printWorld(worldModel);
	
	bool edgeYes = false, edgeStopped = false, edgeVisited = false;		
	int robotId, homePointId, robotStateId, goalId;
	AGMModelSymbol::SPtr node1, node2;
	
	robotId = worldModel->getIdentifierByType("robot");
	homePointId =  worldModel->getIdentifierByType("homePoint");
	robotStateId =  worldModel->getIdentifierByType("robotState");
	goalId =  worldModel->getIdentifierByType("goal");
	/*printf("robotId: %d \n",robotId);
	printf("homePointId: %d \n",homePointId);
	printf("goalId: %d \n",goalId);*/
	
	if(robotId>0 && homePointId>0 && robotStateId>0 && goalId>0){
		for (std::vector<AGMModelEdge>::iterator it = worldModel->edges.begin() ; it != worldModel->edges.end(); ++it)
		{
			if ((*it)->getLabel() == "yes"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == homePointId) || (node2->identifier == robotId && node1->identifier == homePointId)){
					edgeYes = true;
				}
			}
			else if((*it)->getLabel() == "stopped"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == robotStateId) || (node2->identifier == robotId && node1->identifier == robotStateId)){
					edgeStopped = true;
				}
			}
			else if((*it)->getLabel() == "visited"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == goalId) || (node2->identifier == robotId && node1->identifier == goalId)){
					edgeVisited = true;
				}
			}
		}
	}
		
	
	if(edgeYes && edgeVisited && edgeStopped)
	{
		//goalReached = false;
		QMutexLocker locker(mutex);
		
		AGMModel::SPtr newModel(new AGMModel(worldModel));			

		//Editar el edge
		AGMModelSymbol::SPtr nodeRobot = newModel->getSymbol(robotId);
		AGMModelSymbol::SPtr nodeGoal = newModel->getSymbol(goalId);	
		newModel->removeEdge(nodeRobot, nodeGoal, "visited");
		newModel->removeSymbol(nodeGoal);
	 	
 		//AGMModelPrinter::printWorld(newModel);
	
		//qDebug()<<"----------- MODIFICATION -----------------------";
		sendModificationProposal(worldModel, newModel);			
		
 		//AGMModelPrinter::printWorld(worldModel);
	}
}

void SpecificWorker::goToPerson()
{
	//AGMModelPrinter::printWorld(worldModel);
	
	bool edgeYes = false, edgeStopped = false, edgeFoundPerson = false;		
	int robotId, homePointId, robotStateId, personId, personPositionId;
	AGMModelSymbol::SPtr node1, node2;
	
	robotId = worldModel->getIdentifierByType("robot");
	homePointId =  worldModel->getIdentifierByType("homePoint");
	robotStateId =  worldModel->getIdentifierByType("robotState");
	personId =  worldModel->getIdentifierByType("person");
	
	if(robotId>0 && homePointId>0 && robotStateId>0 && personId>0){
		for (std::vector<AGMModelEdge>::iterator it = worldModel->edges.begin() ; it != worldModel->edges.end(); ++it)
		{	
			if ((*it)->getLabel() == "yes"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == homePointId) || (node2->identifier == robotId && node1->identifier == homePointId)){
					edgeYes = true;
				}
			}
			else if((*it)->getLabel() == "stopped"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == robotStateId) || (node2->identifier == robotId && node1->identifier == robotStateId)){
					edgeStopped = true;
				}
			}
			else if((*it)->getLabel() == "found"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == personId) || (node2->identifier == robotId && node1->identifier == personId)){
					personPosition.x = std::stoi((*it)->getAttribute("x"));
					personPosition.y = std::stoi((*it)->getAttribute("y"));
					personPosition.th = std::stoi((*it)->getAttribute("th"));
					//cout << "GOAL DESTINATION: " << goalDestination << endl;
					edgeFoundPerson = true;
				}
			}
		}
	}
		
	
	if(edgeYes && edgeStopped && edgeFoundPerson && waiting)
	{
		
		if(waiting){
			arianavigation_proxy->goToPose(personPosition);
			waiting = false;
			sleep(2);	
			
			/*************************NOTE****************
			* Comprobar si no puede ir a ese goal, ¿qué hacer?:
			*	- Volvere al estado inicial?
			*	- Ir a un estado del mundo intermedio
			*	- Limbo
			*********************************************/
			QMutexLocker locker(mutex);
		
			AGMModel::SPtr newModel(new AGMModel(worldModel));			
			
			AGMModelSymbol::SPtr nodePersonPosition =newModel->newSymbol("personPosition");		 // Crear desde 0	
			personPositionId = nodePersonPosition->identifier;
			newModel->addEdgeByIdentifiers(robotId,personPositionId,"going");

			//Editar el edge
			AGMModelSymbol::SPtr nodeRobot = newModel->getSymbol(robotId);
			AGMModelSymbol::SPtr nodeHomePoint = newModel->getSymbol(homePointId);
			AGMModelSymbol::SPtr nodeRobotState = newModel->getSymbol(robotStateId);
			newModel->removeEdge(nodeRobot, nodeHomePoint, "yes");
			newModel->removeEdge(nodeRobot, nodeRobotState, "stopped");
			newModel->addEdgeByIdentifiers(robotId,homePointId,"no");
			newModel->addEdgeByIdentifiers(robotId,robotStateId,"moving");
		 	
	 		//AGMModelPrinter::printWorld(newModel);
	
			//qDebug()<<"----------- MODIFICATION -----------------------";
			sendModificationProposal(worldModel, newModel);			
		
	 		//AGMModelPrinter::printWorld(worldModel);	
		}
	}
}

void SpecificWorker::reachPerson()
{
	//AGMModelPrinter::printWorld(worldModel);
	
	bool edgeNo = false, edgeMoving = false, edgeFound = false, edgeGoing = false;		
	int robotId, homePointId, robotStateId, personId, personPositionId;
	AGMModelSymbol::SPtr node1, node2;
	
	robotId = worldModel->getIdentifierByType("robot");
	homePointId =  worldModel->getIdentifierByType("homePoint");
	robotStateId =  worldModel->getIdentifierByType("robotState");
	personId =  worldModel->getIdentifierByType("person");
	personPositionId =  worldModel->getIdentifierByType("personPosition");
	/*printf("robotId: %d \n",robotId);
	printf("homePointId: %d \n",homePointId);
	printf("robotStateId: %d \n",robotStateId);
	printf("goalId: %d \n",goalId);*/
	
	if(robotId>0 && homePointId>0 && robotStateId>0 && personId>0 && personPositionId>0){
		for (std::vector<AGMModelEdge>::iterator it = worldModel->edges.begin() ; it != worldModel->edges.end(); ++it)
		{
			if ((*it)->getLabel() == "no"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == homePointId) || (node2->identifier == robotId && node1->identifier == homePointId)){
					edgeNo = true;
				}
			}
			else if((*it)->getLabel() == "moving"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == robotStateId) || (node2->identifier == robotId && node1->identifier == robotStateId)){
					edgeMoving = true;
				}
			}
			else if((*it)->getLabel() == "found"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == personId) || (node2->identifier == robotId && node1->identifier == personId)){
					edgeFound = true;
				}
			}
			else if((*it)->getLabel() == "going"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == personPositionId) || (node2->identifier == robotId && node1->identifier == personPositionId)){
					edgeGoing = true;
				}
			}
		}
	}
		
	
	if(edgeNo && edgeFound && edgeMoving && edgeGoing && !goalReached)
	{
		checkGoalReached();
		
		if(goalReached){
			waiting = true;
			QMutexLocker locker(mutex);
		
			AGMModel::SPtr newModel(new AGMModel(worldModel));			

			//Editar el edge
			AGMModelSymbol::SPtr nodeRobot = newModel->getSymbol(robotId);
			AGMModelSymbol::SPtr nodeRobotState = newModel->getSymbol(robotStateId);
			AGMModelSymbol::SPtr nodePersonPosition = newModel->getSymbol(personPositionId);
			newModel->removeEdge(nodeRobot, nodePersonPosition, "going");
			newModel->addEdgeByIdentifiers(robotId,personPositionId,"reached");
			newModel->removeEdge(nodeRobot, nodeRobotState, "moving");
			newModel->addEdgeByIdentifiers(robotId,robotStateId,"stopped");
		 	
	 		//AGMModelPrinter::printWorld(newModel);
	
			//qDebug()<<"----------- MODIFICATION -----------------------";
			sendModificationProposal(worldModel, newModel);	
			goalReached = false;		
		
	 		//AGMModelPrinter::printWorld(worldModel);
		}			
	}
}

void SpecificWorker::goToHome2()
{
	//AGMModelPrinter::printWorld(worldModel);
	
	bool edgeNo = false, edgeStopped = false, edgeFound = false, edgeDetected = false, edgeReached = false;		
	int robotId, homePointId, robotStateId, personId, personPositionId;
	AGMModelSymbol::SPtr node1, node2;
	
	robotId = worldModel->getIdentifierByType("robot");
	homePointId =  worldModel->getIdentifierByType("homePoint");
	robotStateId =  worldModel->getIdentifierByType("robotState");
	personId =  worldModel->getIdentifierByType("person");
	personPositionId =  worldModel->getIdentifierByType("personPosition");
	
	if(robotId>0 && homePointId>0 && robotStateId>0 && personId>0 && personPositionId>0){
		for (std::vector<AGMModelEdge>::iterator it = worldModel->edges.begin() ; it != worldModel->edges.end(); ++it)
		{
			if ((*it)->getLabel() == "no"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == homePointId) || (node2->identifier == robotId && node1->identifier == homePointId)){
					edgeNo = true;
				}
			}
			else if((*it)->getLabel() == "stopped"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == robotStateId) || (node2->identifier == robotId && node1->identifier == robotStateId)){
					edgeStopped = true;
				}
			}
			else if((*it)->getLabel() == "found"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == personId) || (node2->identifier == robotId && node1->identifier == personId)){
					edgeFound = true;
				}
			}
			else if((*it)->getLabel() == "detected"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == personId) || (node2->identifier == robotId && node1->identifier == personId)){
					edgeDetected = true;
				}
			}
			else if((*it)->getLabel() == "reached"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == personPositionId) || (node2->identifier == robotId && node1->identifier == personPositionId)){
					edgeReached = true;
				}
			}
		}
	}
		
	
	if(edgeNo && edgeStopped && edgeFound && edgeDetected && edgeReached && waiting)
	{
		
		if(waiting){
			arianavigation_proxy->goToGoal("homePoint");
			waiting = false;
			sleep(2);	
			cout << "GOTOHOME - goalReached: " << goalReached << endl; 
			/*************************NOTE****************
			* Comprobar si no puede ir al home, ¿qué hacer?:
			*	- Volvere al estado inicial?
			*	- Ir a un estado del mundo intermedio
			*	- Limbo
			*********************************************/
		
			QMutexLocker locker(mutex);
		
			AGMModel::SPtr newModel(new AGMModel(worldModel));			

			//Editar el edge
			AGMModelSymbol::SPtr nodeRobot = newModel->getSymbol(robotId);
			AGMModelSymbol::SPtr nodehomePoint = newModel->getSymbol(homePointId);
			AGMModelSymbol::SPtr nodeRobotState = newModel->getSymbol(robotStateId);
			newModel->removeEdge(nodeRobot, nodeRobotState, "stopped");
			newModel->addEdgeByIdentifiers(robotId,robotStateId,"moving");		
		 	
	 		//AGMModelPrinter::printWorld(newModel);
	
			//qDebug()<<"----------- MODIFICATION -----------------------";
			sendModificationProposal(worldModel, newModel);			
		
	 		//AGMModelPrinter::printWorld(worldModel);
	 	}
	}
}

void SpecificWorker::reachHome2()
{
	//AGMModelPrinter::printWorld(worldModel);
	
	bool edgeNo = false, edgeMoving = false, edgeFound = false, edgeDetected = false, edgeReached = false;		
	int robotId, homePointId, robotStateId, personId, personPositionId;
	AGMModelSymbol::SPtr node1, node2;
	
	robotId = worldModel->getIdentifierByType("robot");
	homePointId =  worldModel->getIdentifierByType("homePoint");
	robotStateId =  worldModel->getIdentifierByType("robotState");
	personId =  worldModel->getIdentifierByType("person");
	personPositionId =  worldModel->getIdentifierByType("personPosition");
	
	if(robotId>0 && homePointId>0 && robotStateId>0 && personId>0 && personPositionId>0){
		for (std::vector<AGMModelEdge>::iterator it = worldModel->edges.begin() ; it != worldModel->edges.end(); ++it)
		{
			if ((*it)->getLabel() == "no"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == homePointId) || (node2->identifier == robotId && node1->identifier == homePointId)){
					edgeNo = true;
				}
			}
			else if((*it)->getLabel() == "moving"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == robotStateId) || (node2->identifier == robotId && node1->identifier == robotStateId)){
					edgeMoving = true;
				}
			}
			else if((*it)->getLabel() == "found"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == personId) || (node2->identifier == robotId && node1->identifier == personId)){
					edgeFound = true;
				}
			}
			else if((*it)->getLabel() == "detected"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == personId) || (node2->identifier == robotId && node1->identifier == personId)){
					edgeDetected = true;
				}
			}
			else if((*it)->getLabel() == "reached"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == personPositionId) || (node2->identifier == robotId && node1->identifier == personPositionId)){
					edgeReached = true;
				}
			}
		}
	}
		
	
	if(edgeNo && edgeMoving && edgeFound && edgeDetected && edgeReached && !goalReached)
	{
		checkGoalReached();
		
		cout << "ALMENDRAS" << endl;
		if(goalReached){		
			cout << "REACHHOME - goalReached: " << goalReached << endl; 
			goalReached = false;
			waiting = true;
			QMutexLocker locker(mutex);
		
			AGMModel::SPtr newModel(new AGMModel(worldModel));			

			//Editar el edge
			AGMModelSymbol::SPtr nodeRobot = newModel->getSymbol(robotId);
			AGMModelSymbol::SPtr nodeHomePoint = newModel->getSymbol(homePointId);
			AGMModelSymbol::SPtr nodeRobotState = newModel->getSymbol(robotStateId);			
			AGMModelSymbol::SPtr nodePerson = newModel->getSymbol(personId);						
			AGMModelSymbol::SPtr nodePersonPosition = newModel->getSymbol(personPositionId);
			newModel->removeEdge(nodeRobot, nodeHomePoint, "no");
			newModel->addEdgeByIdentifiers(robotId,homePointId,"sleeping");
			newModel->removeEdge(nodeRobot, nodeRobotState, "moving");
			newModel->addEdgeByIdentifiers(robotId,robotStateId,"stopped");
			newModel->removeEdge(nodeRobot, nodePerson, "detected");			
			newModel->removeEdge(nodeRobot, nodePerson, "found");
			newModel->addEdgeByIdentifiers(robotId,personId,"notFound");		
			newModel->removeEdge(nodeRobot, nodePersonPosition, "reached");
			newModel->removeSymbol(nodePersonPosition);
			
			
		 	
	 		//AGMModelPrinter::printWorld(newModel);
	
			//qDebug()<<"----------- MODIFICATION -----------------------";
			sendModificationProposal(worldModel, newModel);			
		
	 		//AGMModelPrinter::printWorld(worldModel);
		}			
	}
}
#endif

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{


//       THE FOLLOWING IS JUST AN EXAMPLE for AGENTS
// 	try
// 	{
// 		RoboCompCommonBehavior::Parameter par = params.at("NameAgent.InnerModel") ;
// 		if( QFile(QString::fromStdString(par.value)).exists() == true)
// 		{
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
// 			innerModel = new InnerModel(par.value);
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file read OK!" ;
// 		}
// 		else
// 		{
// 			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file " << QString::fromStdString(par.value) << " does not exists";
// 			qFatal("Exiting now.");
// 		}
// 	}
// 	catch(std::exception e)
// 	{
// 		qFatal("Error reading config params");
// 	}

	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
	//qDebug()<<"Current action: "<<QString::fromStdString(action);
	if(pc == 0)
	{
		arianavigation_proxy->getInfo();
		pc++;
	}
	else
	{	
		/*goToGoal();
		reachGoal();
		goToHome();
		reachHome();*/
		goToPerson();
		reachPerson();
		goToHome2();
		reachHome2();
	}
}


bool SpecificWorker::reloadConfigAgent()
{
	return true;
}

bool SpecificWorker::activateAgent(const ParameterMap &prs)
{
	bool activated = false;
	if (setParametersAndPossibleActivation(prs, activated))
	{
		if (not activated)
		{
			return activate(p);
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool SpecificWorker::setAgentParameters(const ParameterMap &prs)
{
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

ParameterMap SpecificWorker::getAgentParameters()
{
	return params;
}

void SpecificWorker::killAgent()
{

}

int SpecificWorker::uptimeAgent()
{
	return 0;
}

bool SpecificWorker::deactivateAgent()
{
	return deactivate();
}

StateStruct SpecificWorker::getAgentState()
{
	StateStruct s;
	if (isActive())
	{
		s.state = Running;
	}
	else
	{
		s.state = Stopped;
	}
	s.info = p.action.name;
	return s;
}

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::Event &modification)
{
	mutex->lock();
 	AGMModelConverter::fromIceToInternal(modification.newModel, worldModel);
 
	delete innerModel;
	innerModel = agmInner.extractInnerModel(worldModel);
	mutex->unlock();
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{

}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	mutex->lock();
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	delete innerModel;
	innerModel = agmInner.extractInnerModel(worldModel);
	mutex->unlock();
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	mutex->lock();
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	delete innerModel;
	innerModel = agmInner.extractInnerModel(worldModel);
	mutex->unlock();
}

void SpecificWorker::robotInformation(const mapPose &robotPositionA)
{
	robotPosition = robotPositionA;
}

void SpecificWorker::goalInformation(const GoalInfo &goalA)
{
	currentGoal = goalA.currentGoal;
	goalState = goalA.goalState;
}

void SpecificWorker::mapInformation(const MapInfo &infoA)
{
	numGoals = infoA.numGoals;
	listGoals = infoA.listGoals;
	goalPosition = infoA.goalPosition;
}



bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	printf("<<< setParametersAndPossibleActivation\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	try
	{
		action = params["action"].value;
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);
		//TYPE YOUR ACTION NAME
		if (action == "actionname")
		{
			active = true;
		}
		else
		{
			active = true;
		}
	}
	catch (...)
	{
		printf("exception in setParametersAndPossibleActivation %d\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}

	printf("setParametersAndPossibleActivation >>>\n");

	return true;
}
void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	try
	{
		AGMModelPrinter::printWorld(newModel);
		AGMMisc::publishModification(newModel, agmagenttopic_proxy, worldModel,"agentNavigationAgent");
	}
	catch(...)
	{
		exit(1);
	}
}


void SpecificWorker::checkPathPlanner(){
	pathPlannerState = arianavigation_proxy->checkPathPlannerStatus();
	switch(pathPlannerState){
		case 0:
			//pathPlanner no inicializado
			cout << "El estado del pathPlanner es " << pathPlannerState << endl;
			break;
		case 3:
			//Se ha llegado al goal
			cout << "El estado del pathPlanner es " << pathPlannerState << endl;
			pathPlannerState = -1;
			waiting = true;
			break;
		case 4:
			//Ha fallado la planificación
			cout << "El estado del pathPlanner es " << pathPlannerState << endl;
			pathPlannerState = -1;
			waiting = true;
			break;
		case 5:
			//Ha fallado el movimiento
			cout << "El estado del pathPlanner es " << pathPlannerState << endl;
			pathPlannerState = -1;
			waiting = true;
			break;
		case 6:
			//Ha abortado la búsqueda del camino
			cout << "El estado del pathPlanner es " << pathPlannerState << endl;
			pathPlannerState = -1;
			waiting = true;
			break;
		case 7:
			//Goal inválido
			cout << "El estado del pathPlanner es " << pathPlannerState << endl;
			pathPlannerState = -1;
			waiting = true;
			break;

		default:
			//El path planner se está inicializando (1) o va de camino al goal (2)
			cout << "El estado del pathPlanner es " << pathPlannerState << endl;
	}
}

void SpecificWorker::checkGoalReached(){
	pathPlannerState = arianavigation_proxy->checkPathPlannerStatus();
	switch(pathPlannerState){
		case 3:
			//Se ha llegado al goal
			//cout << "El estado del pathPlanner es " << pathPlannerState << endl;
			goalReached = true;
			break;
		//default:
			//El path planner se está inicializando (1) o va de camino al goal (2)
			//cout << "El estado del pathPlanner es " << pathPlannerState << endl;
	}
}




