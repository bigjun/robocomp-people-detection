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

/**
       \brief
       @author authorname
*/



// THIS IS AN AGENT




#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <string>
#include <regex>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	bool reloadConfigAgent();
	bool activateAgent(const ParameterMap &prs);
	bool setAgentParameters(const ParameterMap &prs);
	ParameterMap getAgentParameters();
	void killAgent();
	int uptimeAgent();
	bool deactivateAgent();
	StateStruct getAgentState();
	void structuralChange(const RoboCompAGMWorldModel::Event &modification);
	void edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications);
	void edgeUpdated(const RoboCompAGMWorldModel::Edge &modification);
	void symbolUpdated(const RoboCompAGMWorldModel::Node &modification);
	void robotInformation(const mapPose &robotPosition);
	void goalInformation(const GoalInfo &goal);
	void mapInformation(const MapInfo &info);

	int pathPlannerState;
	void checkPathPlanner();
	void checkGoalReached();
	bool waiting = true;
	int pc = 0;
	string myAction;
	bool activeTour = false;
	int goalIndex;
	
	int numGoals = 0;
	stringList listGoals;
	mapPoseList goalPosition;
	mapPose robotPosition;
	int currentGoal;
	double goalState;
	string goalDestination;
	bool goalReached;
	personPose personPosition;

public slots:
	void compute();  	
	#ifdef USE_QTGUI
		void goToGoal();
		void reachGoal();
		void goToHome();
		void reachHome();
		void backToInitial();
		void goToPerson();
		void reachPerson();
		void goToHome2();
		void reachHome2();
	#endif		

private:
	std::string action;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	InnerModel *innerModel;
	bool active;
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);
	
};

#endif

