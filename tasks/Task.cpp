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
	// see if we need to initialize the calibration
	// which has to be done when the image size is 
	//
	// this can only be done once the image size is known
	cv::Size currentSize = cv::Size( leftFrame.getWidth(), leftFrame.getHeight() );
	if( currentSize != imageSize )
	{
	    imageSize = currentSize;
	    initCalibration( imageSize );
	}

	// setup buffers for conversion
	leftFrameTarget.init( leftFrame.getWidth(), leftFrame.getHeight(), 8, base::samples::frame::MODE_GRAYSCALE );
	rightFrameTarget.init( leftFrame.getWidth(), leftFrame.getHeight(), 8, base::samples::frame::MODE_GRAYSCALE );

	// see if we want to undistort and perform the conversion
	const bool undistort = !_image_rectified.value();
	leftConv.convert( leftFrame, leftFrameTarget, 0, 0, frame_helper::INTER_LINEAR, undistort );
	rightConv.convert( rightFrame, rightFrameTarget, 0, 0, frame_helper::INTER_LINEAR, undistort );
	
	// get a cv::Mat wrapper around the frames
	cv::Mat rightCvFrame = rightFrameTarget.convertToCvMat();
	cv::Mat leftCvFrame = leftFrameTarget.convertToCvMat();

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
    // initialize the the dense stereo lib calibration
    dense_stereo->setStereoCalibration( _stereoCameraCalibration.get(), 
	    imageSize.width, imageSize.height );

    sparse_stereo->setCalibration( _stereoCameraCalibration.get() );

    // setup frame helper for left and right
    leftConv.setCalibrationParameter( _stereoCameraCalibration.value().camLeft );
    rightConv.setCalibrationParameter( _stereoCameraCalibration.value().camRight );
}

void Task::denseStereo( const cv::Mat& leftCvFrame, const cv::Mat& rightCvFrame )
{
    // create the output distanceImage and register a cv::Mat on the same
    // data buffer
    base::samples::DistanceImage distanceFrame;
    distanceFrame.time = leftFrame.time;
    cv::Mat 
	leftCvResult = dense_stereo->createLeftDistanceImage( distanceFrame ),
	rightCvResult;

    // calculate the distance images
    dense_stereo->processFramePair(leftCvFrame, rightCvFrame, 
	    leftCvResult, rightCvResult, true);

    // create a frame to display the disparity image (mainly for debug)
    if( _disparity_frame.connected() )
    {
	base::samples::frame::Frame disparity_image( 
		distanceFrame.width, distanceFrame.height, 8, 
		base::samples::frame::MODE_GRAYSCALE );
	disparity_image.time = leftFrame.time;
	const float scaling_factor = 255.0f / *std::max_element( distanceFrame.data.begin(), distanceFrame.data.end() );
	uint8_t *data = disparity_image.getImagePtr();

	for( size_t i=0; i<distanceFrame.data.size(); ++i )
	    data[i] = distanceFrame.data[i] * scaling_factor;

	_disparity_frame.write( disparity_image );
    }

    // calculate distance images from disparity images
    dense_stereo->getDistanceImages( leftCvResult, rightCvResult );

    if( _distance_frame.connected() )
    {
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

