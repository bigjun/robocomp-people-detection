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
#ifndef PEOPLEDETECTION_H
#define PEOPLEDETECTION_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <PeopleDetection.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompPeopleDetection;

class PeopleDetectionI : public QObject , public virtual RoboCompPeopleDetection::PeopleDetection
{
Q_OBJECT
public:
	PeopleDetectionI( GenericWorker *_worker, QObject *parent = 0 );
	~PeopleDetectionI();
	
	void peopleDetected( bool  people, const Ice::Current&);

	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif
