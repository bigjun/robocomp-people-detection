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
#ifndef ARIAMAPINFORMATION_H
#define ARIAMAPINFORMATION_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <AriaMapInformation.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompAriaMapInformation;

class AriaMapInformationI : public QObject , public virtual RoboCompAriaMapInformation::AriaMapInformation
{
Q_OBJECT
public:
	AriaMapInformationI( GenericWorker *_worker, QObject *parent = 0 );
	~AriaMapInformationI();
	
	void robotInformation(const mapPose  &robotPosition, const Ice::Current&);
	void goalInformation(const GoalInfo  &goal, const Ice::Current&);
	void mapInformation(const MapInfo  &info, const Ice::Current&);

	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif
