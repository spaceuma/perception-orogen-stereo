/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace dense_stereo;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
}

Task::~Task()
{
}

//RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> leftFrame, rightFrame;//besser?

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    //initialize dense stereo
    dense_stereo = new DenseStereo();

    //configure dense stereo
    dense_stereo->setCalibrationAndLibElasConfiguration(_stereoCameraCalibration.get(), _libElas_conf.get());

    if (! TaskBase::configureHook())
        return false;
    return true;
}
// bool Task::startHook()
// {
//     if (! TaskBase::startHook())
//         return false;    
//     return true;
// }
void Task::updateHook()
{
    TaskBase::updateHook();
    
    base::samples::frame::Frame leftFrame, rightFrame;
    
    if(_left_frame.read(leftFrame) == RTT::NewData && _right_frame.read(rightFrame) == RTT::NewData)
    {
      //check if something is connected to the outputports otherwise do not calculate anything
      if(_distance_frame.connected())
      {
      const size_t 
	  width = leftFrame.getSize().width, 
	  height = leftFrame.getSize().height, 
	  size = width * height;

	//rotate frames by 180 deg and switch them
	//flip images is faster by one magnitude then rotation
	//cv::flip(leftFrame.convertToCvMat(),rightCvFrame,-1);
	//cv::flip(rightFrame.convertToCvMat(),leftCvFrame,-1);
	
	//for wide angle lens flip is not needed
	cv::Mat rightCvFrame = leftFrame.convertToCvMat();
	cv::Mat leftCvFrame = rightFrame.convertToCvMat();

	// pre-allocate the memory for the output disparity map, so we don't
	// have to copy it. This means we have to assert that the width and
	// height is the same for input and resulting disparity images
	distance_image distanceFrame;
	distanceFrame.data.resize( size );	
	distanceFrame.height = height;
	distanceFrame.width = width;	
	distanceFrame.time = rightFrame.time;

	// get calibration to extract scales, baseline and focal length
	StereoCameraCalibration calibration = _stereoCameraCalibration.get();
	
	// set scale x and scale y
	// p_x = (x / focal length) * pixel width, same for y
	// after this transform p_x is in meters and a point on a plane with focal length 1.0
	distanceFrame.scale_x = (float)(calibration.CamLeft.fx / _cameraPixelWidth.get());
	distanceFrame.scale_y = (float)(calibration.CamLeft.fy / _cameraPixelHeight.get());
	
	// set principal point (center_x and center_y) in meters
	distanceFrame.center_x = (float)(calibration.CamLeft.cx * _cameraPixelWidth.get());
	distanceFrame.center_x = (float)(calibration.CamLeft.cy * _cameraPixelHeight.get());
	
	// cv wrappers for the resulting disparity images. 
	// only store the left image, discard the right one
	cv::Mat leftCvDisparityFrame( width, height, cv::DataType<float>::type,
		reinterpret_cast<uint8_t *>( &distanceFrame.data[0] ) );
	cv::Mat rightCvDisparityFrame;

	//calculate the disparities
	dense_stereo->process_FramePair(leftCvFrame, rightCvFrame, leftCvDisparityFrame, rightCvDisparityFrame);

	// set distance factor
	// use focal length in x-direction, because we measure disparities in x-direction. 
	// Only relevant for cameras with rectangular pixel
	const float dist_factor = (float)(calibration.extrinsic.tx * calibration.CamLeft.fx); // baseline * focal length
	// calculate distance as inverse of disparity
	for( size_t i=0; i<size; i++ )
	    distanceFrame.data[i] = dist_factor / distanceFrame.data[i];

	//write to outputs
	_distance_frame.write(distanceFrame);
      }
    }
}
// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }
// void Task::stopHook()
// {
//     TaskBase::stopHook();
// }
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    delete dense_stereo;
}

