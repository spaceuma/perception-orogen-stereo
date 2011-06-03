/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <limits>

#include <base/time.h>

using namespace dense_stereo;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state), leftFrameValid(false), rightFrameValid(false)
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

    while( _left_frame.read(leftFrame) == RTT::NewData ) leftFrameValid = true;
    while( _right_frame.read(rightFrame) == RTT::NewData ) rightFrameValid = true;
    
    // check conditions which must be met so we can/should calculate the distance image
    if( ( std::abs((leftFrame.time - rightFrame.time).toMilliseconds()) < 5) 
	    && leftFrameValid && rightFrameValid 
	    && (_distance_frame.connected() || _disparity_frame.connected())
	    )
    {
	//if sub-sampling is activated image dimensions should be width/2 x height/2 (rounded towards zero)
	const bool subsampling = _libElas_conf.get().subsampling;
      const size_t 
	  width = subsampling ? leftFrame.getSize().width / 2.0f : leftFrame.getSize().width, 
	  height = subsampling ? leftFrame.getSize().height / 2.0f : leftFrame.getSize().height, 
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
	const StereoCameraCalibration &calibration = _stereoCameraCalibration.get();
	
	// scale and center parameters of the distance image are the inverse of the
	// f and c values from the camera calibration. 
	//
	// so analogous for x and y we get scale = 1/f and offset = -c/f
	// 
	distanceFrame.scale_x = 1.0f / calibration.CamLeft.fx;
	distanceFrame.scale_y = 1.0f / calibration.CamLeft.fy;
	distanceFrame.center_x = -calibration.CamLeft.cx / calibration.CamLeft.fx; 
	distanceFrame.center_y = -calibration.CamLeft.cy / calibration.CamLeft.fy; 
	
	// sub-sampling reduces the image width and height by a factor of 2.
	// this leads to an aditional division by 2 for focal length and center,
	// but as both are appearing in the center parameter calculation only the
	// scale parameters need multiplication with 2
	if( subsampling )
	{
	  distanceFrame.scale_x *= 2.0f;
	  distanceFrame.scale_y *= 2.0f;
	}
	
	// cv wrappers for the resulting disparity images. 
	// only store the left image, discard the right one
	cv::Mat leftCvDisparityFrame( width, height, cv::DataType<float>::type,
		reinterpret_cast<uint8_t *>( &distanceFrame.data[0] ) );
	cv::Mat rightCvDisparityFrame;

	//calculate the disparities
	dense_stereo->process_FramePair(leftCvFrame, rightCvFrame, leftCvDisparityFrame, rightCvDisparityFrame);

	// create a frame to display the disparity image (mainly for debug)
	if( _disparity_frame.connected() )
	{
	    base::samples::frame::Frame disparity_image( 
		    width, height, 8, base::samples::frame::MODE_GRAYSCALE );
	    const float scaling_factor = 255.0f / *std::max_element( distanceFrame.data.begin(), distanceFrame.data.end() );
	    uint8_t *data = disparity_image.getImagePtr();
	    
	    for( size_t i=0; i<size; i++ )
		data[i] = distanceFrame.data[i] * scaling_factor;

	    _disparity_frame.write( disparity_image );
	}

	if( _distance_frame.connected() )
	{
	    // set distance factor
	    // if sub-sampling is active focal length has to be divided by 2 same as width
	    const float focal_length = subsampling ? calibration.CamLeft.fx / 2.0f : calibration.CamLeft.fx;
	    const float dist_factor = fabs( focal_length * calibration.extrinsic.tx * 1e-3 ); // baseline in meters 

	    // calculate distance as inverse of disparity
	    for( size_t i=0; i<size; i++ )
	    {
		const float disparity = distanceFrame.data[i];
		distanceFrame.data[i] = disparity > 0 ? 
		    dist_factor / disparity : 
		    std::numeric_limits<float>::quiet_NaN();
	    }

	    //write to output
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

