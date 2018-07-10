/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef DENSE_STEREO_TASK_TASK_HPP
#define DENSE_STEREO_TASK_TASK_HPP

#include <stddef.h>
#include <opencv/cv.h>
#include <base/samples/Frame.hpp>
#include <frame_helper/CalibrationCv.h>
#include <frame_helper/FrameHelper.h>

#include "stereo/TaskBase.hpp"

namespace stereo {

    class StereoFeatures;
    class DenseStereo;

    class Task : public TaskBase
    {
        friend class TaskBase;

    protected:
        //Instance for the input ports
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> leftFrame, rightFrame;
        ///Instance of dense stereo processing
        DenseStereo *dense_stereo;

        base::samples::frame::Frame leftFrameTarget, rightFrameTarget, leftFrameSync, rightFrameSync;
        frame_helper::FrameHelper leftConv, rightConv;
        bool leftFrameValid, rightFrameValid;
        cv::Size imageSize;
        double imageScalingFactor;
        frame_helper::StereoCalibration calibration;

        void initCalibration( cv::Size imageSize );
        void denseStereo( const cv::Mat& leftImage, const cv::Mat& rightImage );

    public:
        Task(std::string const& name = "stereo::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         *
         *   task_context "TaskName" do
         *     needs_configuration
         *     ...
         *   end
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recovered() to go back in the Runtime state.
         */
        // void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

