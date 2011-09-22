/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <limits>

#include <stddef.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <stereo/densestereo.h>
#include <stereo/sparse_stereo.hpp>
#include <stereo/dense_stereo_types.h>
#include <base/time.h>
#include <frame_helper/Calibration.h>
#include <frame_helper/FrameHelper.h>

using namespace stereo;

namespace stereo
{
struct Task::SparseDebugImpl
{
    base::samples::frame::Frame debugFrame; 
    cv::Mat stereoFrame[2];
    stereo::StereoFeatureArray features[2];
    bool hasPrev;
    int idx;

    cv::Mat& currentFrame() { return stereoFrame[idx]; }
    cv::Mat& previousFrame() { return stereoFrame[(idx+1)%2]; }

    stereo::StereoFeatureArray& currentFeatures() { return features[idx]; }
    stereo::StereoFeatureArray& previousFeatures() { return features[(idx+1)%2]; }

    SparseDebugImpl() : hasPrev(false), idx(0) {}

    void update( Task* task )
    {
	stereo::StereoFeatures *stereo = task->sparse_stereo;

	stereo->getDebugImage().copyTo( currentFrame() );
	currentFeatures() = stereo->getStereoFeatures();

	if( hasPrev )
	{
	    stereo->calculateInterFrameCorrespondences( previousFeatures(), currentFeatures(), stereo::FILTER_ISOMETRY );
	    cv::Mat debugImage = stereo->getInterFrameDebugImage( previousFrame(), previousFeatures(), currentFrame(), currentFeatures());

	    frame_helper::FrameHelper::copyMatToFrame( 
		    debugImage, debugFrame );

	    task->_sparse_debug.write( debugFrame );
	}

	currentFrame().copyTo( previousFrame() );
	previousFeatures() = currentFeatures();
	hasPrev = true;

	idx = (idx+1)%2;
    }

};
}

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state), leftFrameValid(false), rightFrameValid(false), sparseDebug( new Task::SparseDebugImpl() )
{
}

Task::~Task()
{
    delete sparseDebug;
}

//RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> leftFrame, rightFrame;//besser?

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    // initialize dense stereo
    dense_stereo = new DenseStereo();
    sparse_stereo = new StereoFeatures();

    // configure dense stereo
    dense_stereo->setLibElasConfiguration(_libElas_conf.get());

    // configure sparse stereo
    sparse_stereo->setConfiguration( _sparse_config.get() );

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
	    )
    {
	//rotate frames by 180 deg and switch them
	//flip images is faster by one magnitude then rotation
	//cv::flip(leftFrame.convertToCvMat(),rightCvFrame,-1);
	//cv::flip(rightFrame.convertToCvMat(),leftCvFrame,-1);
	
	//for wide angle lens flip is not needed
	cv::Mat rightCvFrame = leftFrame.convertToCvMat();
	cv::Mat leftCvFrame = rightFrame.convertToCvMat();

	// see if we need to initialize the calibration still
	// this can only be done once the image size is known
	cv::Size imageSize = leftCvFrame.size();
	if( calib.getImageSize() != imageSize )
	    initCalibration( imageSize );

	// convert ot greyscale if required
	if( leftCvFrame.type() != CV_8UC1 )
	{
	    cv::Mat leftGray, rightGray; 

	    cv::cvtColor( leftCvFrame, leftGray, CV_BGR2GRAY );
	    cv::cvtColor( rightCvFrame, rightGray, CV_BGR2GRAY );

	    leftCvFrame = leftGray;
	    rightCvFrame = rightGray;
	}

	// if the images are not undistorted and rectified, we do it now
	if( !_image_rectified.value() )
	{
	    cv::Mat leftRectified, rightRectified;

	    calib.camLeft.undistortAndRectify( leftCvFrame, leftRectified );
	    calib.camRight.undistortAndRectify( rightCvFrame, rightRectified );

	    leftCvFrame = leftRectified;
	    rightCvFrame = rightRectified;
	}	    

	// perform dense stereo processing if the output ports are connected
	if( _distance_frame.connected() || _disparity_frame.connected() )
	    denseStereo( leftCvFrame, rightCvFrame );

	// same for sparse
	if( _sparse_debug.connected() || _stereo_features.connected() )
	    sparseStereo( leftCvFrame, rightCvFrame );
    }
}

void Task::initCalibration( const cv::Size imageSize )
{
    // create own cv calibration object
    calib.setCalibration( _stereoCameraCalibration.value() );
    calib.setImageSize( imageSize );
    calib.initCv();

    // initialize the the dense stereo lib calibration
    dense_stereo->setStereoCalibration( _stereoCameraCalibration.get(), 
	    imageSize.width, imageSize.height );

    sparse_stereo->setCalibration( _stereoCameraCalibration.get() );
}

void Task::denseStereo( const cv::Mat& leftCvFrame, const cv::Mat& rightCvFrame )
{
    // TODO most of this stuff should go into the library

    //if sub-sampling is activated image dimensions should be width/2 x height/2 (rounded towards zero)
    const bool subsampling = _libElas_conf.get().subsampling;
    const size_t 
	width = subsampling ? leftFrame.getSize().width / 2 : leftFrame.getSize().width, 
	height = subsampling ? leftFrame.getSize().height / 2 : leftFrame.getSize().height, 
	size = width * height;

    // pre-allocate the memory for the output disparity map, so we don't
    // have to copy it. This means we have to assert that the width and
    // height is the same for input and resulting disparity images
    base::samples::DistanceImage distanceFrame;
    distanceFrame.data.resize( size );	
    distanceFrame.height = height;
    distanceFrame.width = width;	
    distanceFrame.time = rightFrame.time;

    // get calibration to extract scales, baseline and focal length
    const frame_helper::StereoCalibration &calibration = _stereoCameraCalibration.get();

    // scale and center parameters of the distance image are the inverse of the
    // f and c values from the camera calibration. 
    //
    // so analogous for x and y we get scale = 1/f and offset = -c/f
    // 
    distanceFrame.scale_x = 1.0f / calibration.camLeft.fx;
    distanceFrame.scale_y = 1.0f / calibration.camLeft.fy;
    distanceFrame.center_x = -calibration.camLeft.cx / calibration.camLeft.fx; 
    distanceFrame.center_y = -calibration.camLeft.cy / calibration.camLeft.fy; 

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
    dense_stereo->process_FramePair(leftCvFrame, rightCvFrame, 
	    leftCvDisparityFrame, rightCvDisparityFrame, true);

    // create a frame to display the disparity image (mainly for debug)
    if( _disparity_frame.connected() )
    {
	base::samples::frame::Frame disparity_image( 
		width, height, 8, base::samples::frame::MODE_GRAYSCALE );
	disparity_image.time = rightFrame.time;
	const float scaling_factor = 255.0f / *std::max_element( distanceFrame.data.begin(), distanceFrame.data.end() );
	uint8_t *data = disparity_image.getImagePtr();

	for( size_t i=0; i<size; ++i )
	    data[i] = distanceFrame.data[i] * scaling_factor;

	_disparity_frame.write( disparity_image );
    }

    if( _distance_frame.connected() )
    {
	// set distance factor
	// if sub-sampling is active focal length has to be divided by 2 same as width
	const float focal_length = subsampling ? calibration.camLeft.fx / 2.0f : calibration.camLeft.fx;
	const float dist_factor = fabs( focal_length * calibration.extrinsic.tx * 1e-3 ); // baseline in meters 

	// calculate distance as inverse of disparity
	for( size_t i=0; i<size; ++i )
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

void Task::sparseStereo( const cv::Mat& leftImage, const cv::Mat& rightImage )
{
    sparse_stereo->processFramePair( leftImage, rightImage );
    if( _sparse_debug.connected() )
	sparseDebug->update( this );
    StereoFeatureArray &feature_array( sparse_stereo->getStereoFeatures() );
    feature_array.time = rightFrame.time;
    _stereo_features.write( feature_array ); 
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
    delete sparse_stereo;
}

