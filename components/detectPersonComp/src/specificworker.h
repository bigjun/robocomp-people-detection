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







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/exceptions.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/people/people_detector.h>
#include <pcl/gpu/people/colormap.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <boost/filesystem.hpp>

#include <iostream>

namespace pc = pcl::console;
using namespace pcl::visualization;
using namespace pcl::gpu;
using namespace pcl;
using namespace std;


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	
	typedef pcl::gpu::people::PeopleDetector PeopleDetector;

    enum { COLS = 640, ROWS = 480 };
    
    //pcl::Grabber& capture_;
    boost::shared_ptr<pcl::Grabber> capture;
    boost::signals2::connection c;
    
    boost::mutex data_ready_mutex_;
    boost::condition_variable data_ready_cond_;

    bool cloud_cb_;
    bool exit_;
    bool has_data;
    int time_ms_;
    int counter_;
    int process_return_;
    PeopleDetector people_detector_;
    PeopleDetector::Image cmap_device_;
    pcl::PointCloud<pcl::RGB> cmap_host_;

    PeopleDetector::Depth depth_device_;
    PeopleDetector::Image image_device_;

    pcl::PointCloud<unsigned short> depth_host_;
    pcl::PointCloud<pcl::RGB> rgba_host_;
    std::vector<unsigned char> rgb_host_;

    PointCloud<PointXYZRGBA> cloud_host_;

    ImageViewer final_view_;
    ImageViewer depth_view_;

    DeviceArray<pcl::RGB> color_map_;
    
public slots:
	void compute();
	void visualizeAndWrite();
	void source_cb1(const boost::shared_ptr<const PointCloud<PointXYZRGBA> >& cloud);
	void source_cb2(const boost::shared_ptr<openni_wrapper::Image>& image_wrapper, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_wrapper, float);
	

private:
	
};

#endif

