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
		QPushButton *btn1 = new QPushButton("detectPerson",this);
		vlay->addWidget(btn1);
		
		//Connection of the buttons with the functionalities
	    connect(btn1, SIGNAL (clicked()), this, SLOT (detectPerson()));
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
void SpecificWorker::detectPerson()
{
	//AGMModelPrinter::printWorld(worldModel);
	
	bool edgeNo = false, edgeStopped = false, edgeFound = false, edgeReached = false, edgeDetected = false;		
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
			else if((*it)->getLabel() == "reached"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == personPositionId) || (node2->identifier == robotId && node1->identifier == personPositionId)){
					edgeReached = true;
				}
			}
			else if((*it)->getLabel() == "detected"){
				node1 = worldModel->getSymbol((*it)->getSymbolPair().first);
				node2 = worldModel->getSymbol((*it)->getSymbolPair().second);
				if((node1->identifier == robotId && node2->identifier == personId) || (node2->identifier == robotId && node1->identifier == personId)){
					edgeDetected = true;
				}
			}
		}
	}
		
	
	if(edgeNo && edgeStopped && edgeFound && edgeReached && !edgeDetected)
	{		
			personPositionReached = true;
			/*************************NOTE****************
			* Comprobar si no puede ir a ese goal, ¿qué hacer?:
			*	- Volvere al estado inicial?
			*	- Ir a un estado del mundo intermedio
			*	- Limbo
			*********************************************/
			if(personDetected){
				QMutexLocker locker(mutex);
		
				AGMModel::SPtr newModel(new AGMModel(worldModel));			

				//Editar el edge
				AGMModelSymbol::SPtr nodeRobot = newModel->getSymbol(robotId);
				AGMModelSymbol::SPtr nodePerson = newModel->getSymbol(personId);
				newModel->addEdge(nodeRobot, nodePerson,"detected");
			 	
		 		//AGMModelPrinter::printWorld(newModel);
	
				//qDebug()<<"----------- MODIFICATION -----------------------";
				sendModificationProposal(worldModel, newModel);			
		
		 		//AGMModelPrinter::printWorld(worldModel);	
		 		personPositionReached = false;
		 		personDetected = false;
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
	detectPerson();
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

void SpecificWorker::peopleDetected(const bool &people)
{
	if(people && personPositionReached){
		printf("PERSONA DETECTADA!!!!!!!!!!!\n");
		personDetected = true;
	}
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
		AGMMisc::publishModification(newModel, agmagenttopic_proxy, worldModel,"agentsrCompAgent");
	}
	catch(...)
	{
		exit(1);
	}
}




