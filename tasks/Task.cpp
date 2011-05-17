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
    disparity_image disparityFrame;
    cv::Mat leftCvFrame, rightCvFrame, leftCvDisparityFrame, rightCvDisparityFrame;
    
    if(_left_frame.read(leftFrame) == RTT::NewData && _right_frame.read(rightFrame) == RTT::NewData)
    {
      //check if something is connected to the outputports otherwise do not calculate anything
      if(_disparity_frame.connected())
      {
	//rotate frames by 180 deg and switch them
	//flip images is faster by one magnitude then rotation
	//cv::flip(leftFrame.convertToCvMat(),rightCvFrame,-1);
	//cv::flip(rightFrame.convertToCvMat(),leftCvFrame,-1);
	
	//for wide angle lens flip is not needed
	rightCvFrame = leftFrame.convertToCvMat();
	leftCvFrame = rightFrame.convertToCvMat();
	
	//calculate the disparities
	dense_stereo->process_FramePair(leftCvFrame, rightCvFrame, leftCvDisparityFrame, rightCvDisparityFrame);
	
	//setup the disparity image
	disparityFrame.data = (float *)leftCvDisparityFrame.data;
	disparityFrame.height = leftCvDisparityFrame.size().height;
	disparityFrame.width = leftCvDisparityFrame.size().width;	

	//set the frame's timestamp
	disparityFrame.time = rightFrame.time; //rightFrame.time because switch of the to input frames
	
	//write to outputs
	_disparity_frame.write(disparityFrame);
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

