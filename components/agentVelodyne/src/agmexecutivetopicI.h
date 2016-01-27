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
#ifndef AGMEXECUTIVETOPIC_H
#define AGMEXECUTIVETOPIC_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <AGMExecutive.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompAGMExecutive;

class AGMExecutiveTopicI : public QObject , public virtual RoboCompAGMExecutive::AGMExecutiveTopic
{
Q_OBJECT
public:
	AGMExecutiveTopicI( GenericWorker *_worker, QObject *parent = 0 );
	~AGMExecutiveTopicI();
	
	void structuralChange(const RoboCompAGMWorldModel::Event  &modification, const Ice::Current&);
	void edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence  &modifications, const Ice::Current&);
	void edgeUpdated(const RoboCompAGMWorldModel::Edge  &modification, const Ice::Current&);
	void symbolUpdated(const RoboCompAGMWorldModel::Node  &modification, const Ice::Current&);

	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif
