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
  
    //configuration?
    
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
    
    base::samples::frame::Frame leftFrame, rightFrame, leftDisparityFrame, rightDisparityFrame;
    cv::Mat leftCvFrame, rightCvFrame, leftCvDisparityFrame, rightCvDisparityFrame;
    
    if(_left_frame.read(leftFrame) == RTT::NewData && _right_frame.read(rightFrame) == RTT::NewData)
    {
      //check if something is connected to the outputports otherwise do not calculate anything
      if(_left_disparity_frame.connected() || _right_disparity_frame.connected())
      {
	//rotate frames by 180 deg and switch them
	//flip images is faster by one magnitude then rotation
	cv::flip(leftFrame.convertToCvMat(),rightCvFrame,-1);
	cv::flip(rightFrame.convertToCvMat(),leftCvFrame,-1);
	
	//for wide angle lens flip is not needed
	//rightCvFrame = leftFrame.convertToCvMat();
	//leftCvFrame = rightFrame.convertToCvMat();
	
	//calculate the disparities
	dense_stereo->process_FramePair(leftCvFrame, rightCvFrame, leftCvDisparityFrame, rightCvDisparityFrame);
	
	//save input images to filesystem
	ostringstream left_file_time, right_file_time;
	left_file_time << leftFrame.time.toMilliseconds();
	right_file_time << rightFrame.time.toMilliseconds();
	
	cv::imwrite("right_frame_" + right_file_time.str() + ".png",rightCvFrame);
	cv::imwrite("left_frame_" + left_file_time.str() + ".png",leftCvFrame);
	
	//test if output images are ok
	cv::imwrite("right_frame_disp_" + right_file_time.str() + ".png",rightCvDisparityFrame);
	cv::imwrite("left_frame_disp_" + left_file_time.str() + ".png",leftCvDisparityFrame);
	
	//leftDisparityFrame convert to Frame
	leftDisparityFrame.init(leftCvDisparityFrame.size().width, leftCvDisparityFrame.size().height, leftCvDisparityFrame.elemSize1(), base::samples::frame::MODE_GRAYSCALE);
	leftDisparityFrame.setImage((const char *)leftCvDisparityFrame.data, leftCvDisparityFrame.size().width * leftCvDisparityFrame.size().height);
	//leftDisparityFrame convert to Frame
	rightDisparityFrame.init(rightCvDisparityFrame.size().width, rightCvDisparityFrame.size().height, rightCvDisparityFrame.elemSize1(), base::samples::frame::MODE_GRAYSCALE);
	rightDisparityFrame.setImage((const char *)rightCvDisparityFrame.data, rightCvDisparityFrame.size().width * rightCvDisparityFrame.size().height);

	//set the frame's timestamp
	leftDisparityFrame.time = rightFrame.time;
	rightDisparityFrame.time = leftFrame.time;
	
	//write to outputs
	_left_disparity_frame.write(leftDisparityFrame);
	_right_disparity_frame.write(rightDisparityFrame);
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

