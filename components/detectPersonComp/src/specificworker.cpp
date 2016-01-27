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
//#include "PeoplePCApp.cpp"

/**
* \brief Default constructor
*/

struct SampledScopeTime : public StopWatch
{
  enum { EACH = 33 };
  SampledScopeTime(int& time_ms) : time_ms_(time_ms) {}
  ~SampledScopeTime()
  {
    static int i_ = 0;
    time_ms_ += getTime ();
    if (i_ % EACH == 0 && i_)
    {
      cout << "Average frame time = " << time_ms_ / EACH << "ms ( " << 1000.f * EACH / time_ms_ << "fps )" << endl;
      time_ms_ = 0;
    }
    ++i_;
  }
  private:
  int& time_ms_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

string 
make_name(int counter, const char* suffix)
{
  char buf[4096];
  sprintf (buf, "./people%04d_%s.png", counter, suffix);
  return buf;
}

template<typename T> void 
savePNGFile(const std::string& filename, const pcl::gpu::DeviceArray2D<T>& arr)
{
  int c;
  pcl::PointCloud<T> cloud(arr.cols(), arr.rows());
  arr.download(cloud.points, c);
  pcl::io::savePNGFile(filename, cloud);
}

template <typename T> void
savePNGFile (const std::string& filename, const pcl::PointCloud<T>& cloud)
{
  pcl::io::savePNGFile(filename, cloud);
}

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx), exit_(false), time_ms_(0), cloud_cb_(true), counter_(0), final_view_("Final labeling"), depth_view_("Depth")
{

  // selecting data source
  capture.reset( new pcl::OpenNIGrabber() );
  pcl::Grabber& capture_ = *capture;

  //selecting tree files
  vector<string> tree_files;
  tree_files.push_back("Data/forest1/tree_20.txt");
  tree_files.push_back("Data/forest2/tree_20.txt");
  tree_files.push_back("Data/forest3/tree_20.txt");

  try
  {
    // loading trees
    typedef pcl::gpu::people::RDFBodyPartsDetector RDFBodyPartsDetector;
    RDFBodyPartsDetector::Ptr rdf(new RDFBodyPartsDetector(tree_files));
    PCL_INFO("Loaded files into rdf");
    
    final_view_.setSize (COLS, ROWS);
	depth_view_.setSize (COLS, ROWS);

	final_view_.setPosition (0, 0);
	depth_view_.setPosition (650, 0);

	cmap_device_.create(ROWS, COLS);
	cmap_host_.points.resize(COLS * ROWS);
	depth_device_.create(ROWS, COLS);
	image_device_.create(ROWS, COLS);

	depth_host_.points.resize(COLS * ROWS);

	rgba_host_.points.resize(COLS * ROWS);
	rgb_host_.resize(COLS * ROWS * 3);

	people::uploadColorMap(color_map_);
	
	people_detector_.rdf_detector_ = rdf;
	
	
	
	cloud_cb_ = false;

    PCDGrabberBase* ispcd = dynamic_cast<pcl::PCDGrabberBase*>(&capture_);
    if (ispcd)
      cloud_cb_= true;

    typedef boost::shared_ptr<openni_wrapper::DepthImage> DepthImagePtr;
    typedef boost::shared_ptr<openni_wrapper::Image> ImagePtr;

    boost::function<void (const boost::shared_ptr<const PointCloud<PointXYZRGBA> >&)> func1 = boost::bind (&SpecificWorker::source_cb1, this, _1);
    boost::function<void (const ImagePtr&, const DepthImagePtr&, float constant)> func2 = boost::bind (&SpecificWorker::source_cb2, this, _1, _2, _3);                  
    c = cloud_cb_ ? capture_.registerCallback (func1) : capture_.registerCallback (func2);
          
      
      boost::unique_lock<boost::mutex> lock(data_ready_mutex_);   
	  capture_.start ();

  }
  catch (const pcl::PCLException& e) { cout << "PCLException: " << e.detailedMessage() << endl; }  
  catch (const std::runtime_error& e) { cout << e.what() << endl; }
  catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; }
  catch (const std::exception& /*e*/) { cout << "Exception" << endl; }
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
	boost::unique_lock<boost::mutex> lock(data_ready_mutex_);
    try
    {
      
	  if(!exit_ && !final_view_.wasStopped())
	  {
			has_data = data_ready_cond_.timed_wait(lock, boost::posix_time::millisec(100));
			if(has_data)
			{
			  SampledScopeTime fps(time_ms_);

			  if (cloud_cb_)
				process_return_ = people_detector_.process(cloud_host_.makeShared());
			  else
				process_return_ = people_detector_.process(depth_device_, image_device_);

			  ++counter_;
			}

			if(has_data && (process_return_ == 2)){
			  peopledetection_proxy->peopleDetected(true);		  
			  printf("PERSONA DETECTADA!!!!!!!!!!!\n");
			  visualizeAndWrite();
			}
	   }
	   //final_view_.spinOnce (3);
    }
    catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; }
    catch (const std::exception& /*e*/) { cout << "Exception" << endl; }
}

void SpecificWorker::visualizeAndWrite()
{
  const PeopleDetector::Labels& labels = people_detector_.rdf_detector_->getLabels();
  people::colorizeLabels(color_map_, labels, cmap_device_);

  int c;
  cmap_host_.width = cmap_device_.cols();
  cmap_host_.height = cmap_device_.rows();
  cmap_host_.points.resize(cmap_host_.width * cmap_host_.height);
  cmap_device_.download(cmap_host_.points, c);

  final_view_.showRGBImage<pcl::RGB>(cmap_host_);
  final_view_.spinOnce(1, true);

  if (cloud_cb_)
  {
    depth_host_.width = people_detector_.depth_device1_.cols();
    depth_host_.height = people_detector_.depth_device1_.rows();
    depth_host_.points.resize(depth_host_.width * depth_host_.height);
    people_detector_.depth_device1_.download(depth_host_.points, c);
  }

  depth_view_.showShortImage(&depth_host_.points[0], depth_host_.width, depth_host_.height, 0, 5000, true);
  depth_view_.spinOnce(1, true);
}

void SpecificWorker::source_cb1(const boost::shared_ptr<const PointCloud<PointXYZRGBA> >& cloud)
{
  {
    boost::mutex::scoped_lock lock(data_ready_mutex_);
    if (exit_)
      return;

    pcl::copyPointCloud(*cloud, cloud_host_);
  }
  data_ready_cond_.notify_one();
}

void SpecificWorker::source_cb2(const boost::shared_ptr<openni_wrapper::Image>& image_wrapper, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_wrapper, float)
{
  {
    boost::mutex::scoped_try_lock lock(data_ready_mutex_);

    if (exit_ || !lock)
      return;

    //getting depth
    int w = depth_wrapper->getWidth();
    int h = depth_wrapper->getHeight();
    int s = w * PeopleDetector::Depth::elem_size;
    const unsigned short *data = depth_wrapper->getDepthMetaData().Data();
    depth_device_.upload(data, s, h, w);

    depth_host_.points.resize(w *h);
    depth_host_.width = w;
    depth_host_.height = h;
    std::copy(data, data + w * h, &depth_host_.points[0]);

    //getting image
    w = image_wrapper->getWidth();
    h = image_wrapper->getHeight();
    s = w * PeopleDetector::Image::elem_size;

    //fill rgb array
    rgb_host_.resize(w * h * 3);
    image_wrapper->fillRGB(w, h, (unsigned char*)&rgb_host_[0]);

    // convert to rgba, TODO image_wrapper should be updated to support rgba directly
    rgba_host_.points.resize(w * h);
    rgba_host_.width = w;
    rgba_host_.height = h;
    for(int i = 0; i < rgba_host_.size(); ++i)
    {
      const unsigned char *pixel = &rgb_host_[i * 3];
      RGB& rgba = rgba_host_.points[i];
      rgba.r = pixel[0];
      rgba.g = pixel[1];
      rgba.b = pixel[2];
    }
    image_device_.upload(&rgba_host_.points[0], s, h, w);
  }
  data_ready_cond_.notify_one();
}
