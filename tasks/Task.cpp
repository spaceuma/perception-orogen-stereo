/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <limits>

#include <stddef.h>
#include <stereo/densestereo.h>
#include <stereo/dense_stereo_types.h>
#include <base/Time.hpp>
#include <frame_helper/Calibration.h>
#include <frame_helper/CalibrationCv.h>
#include <frame_helper/FrameHelper.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <stdexcept>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> CloudType;
CloudType::Ptr cloud (new CloudType);

using namespace stereo;

Task::Task(std::string const& name): TaskBase(name), dense_stereo(0)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine): TaskBase(name, engine), dense_stereo(0)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{

    // initialize dense stereo
    if(dense_stereo)
        delete dense_stereo;

    // configure dense stereo
    dense_stereo = new DenseStereo();
    dense_stereo->setLibElasConfiguration(_libElas_conf.get());
    dense_stereo->setGaussianKernel( _gaussian_kernel.get() );

    calibration = _stereoCameraCalibration.get();

    imageScalingFactor = _image_scaling_factor.get();

    if (! TaskBase::configureHook())
        return false;

    udp_config = _udp_config.get();        

    loccam = _loccam.get();        

    navcam = _navcam.get();        

    if (udp_config)
    {
        // Ports to receive data from Vortex (server and client)
        stereo_port_c = _stereo_port_c.get();

        // Load addresses
        addr_c = _addr_c.get();
        
        if (!create_socks)
        {
            udp_stereo = new udp::UDP();

            // Creating socks to receive data from Vortex (server and client)
            stereo_sock_client = udp_stereo->createSocket();

            // Bind & connect sockets with addresses to receive data from Vortex
            udp_stereo->connectSocket(stereo_sock_client, addr_c, stereo_port_c);

            create_socks = true;

        }
    }

    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    leftFrameValid = false;
    rightFrameValid = false;

    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    while( _left_frame.read(leftFrame) == RTT::NewData ) leftFrameValid = true;
    while( _right_frame.read(rightFrame) == RTT::NewData ) rightFrameValid = true;

    // check conditions which must be met so we can/should calculate the distance image
    if( leftFrameValid && rightFrameValid && (std::abs((leftFrame->time - rightFrame->time).toMilliseconds()) < 5) )
    {
        // see if we need to initialize the calibration
        // this can only be done once the image size is known
        cv::Size currentSize = cv::Size( leftFrame->getWidth() * imageScalingFactor, leftFrame->getHeight() * imageScalingFactor );
        if( currentSize != imageSize )
        {
            imageSize = currentSize;
            initCalibration( imageSize );
        }

        // create tmp frames in case we need to flip
        base::samples::frame::Frame ltmp, rtmp;
        // flip input images
        if( _image_rotated.value() )
        {
            leftConv.rotateBy180Degrees( *leftFrame, ltmp );
            rightConv.rotateBy180Degrees( *rightFrame, rtmp );
        } else {
            ltmp.init(*leftFrame, true);
            rtmp.init(*rightFrame, true);
        }

        // setup buffers for conversion
        leftFrameTarget.init( leftFrame->getWidth() * imageScalingFactor, leftFrame->getHeight() * imageScalingFactor, 8, base::samples::frame::MODE_GRAYSCALE );
        rightFrameTarget.init( leftFrame->getWidth() * imageScalingFactor, leftFrame->getHeight() * imageScalingFactor, 8, base::samples::frame::MODE_GRAYSCALE );

        // see if we want to undistort and perform the conversion
        const bool undistort = !_image_rectified.value();
        leftConv.convert( ltmp, leftFrameTarget, 0, 0, frame_helper::INTER_LINEAR, undistort );
        rightConv.convert( rtmp, rightFrameTarget, 0, 0, frame_helper::INTER_LINEAR, undistort );

        // get a cv::Mat wrapper around the frames
        cv::Mat rightCvFrame = frame_helper::FrameHelper::convertToCvMat(rightFrameTarget);
        cv::Mat leftCvFrame = frame_helper::FrameHelper::convertToCvMat(leftFrameTarget);

        // perform dense stereo processing if the output ports are connected
        if(_distance_frame.connected() || _disparity_frame.connected() || _point_cloud.connected())
            denseStereo( leftCvFrame, rightCvFrame );

        // setup buffers for conversion
        leftFrameSync.init( leftFrame->getWidth() * imageScalingFactor, leftFrame->getHeight() * imageScalingFactor, 8, _output_format_sync.value() );
        rightFrameSync.init( leftFrame->getWidth() * imageScalingFactor, leftFrame->getHeight() * imageScalingFactor, 8, _output_format_sync.value() );
        leftConv.convert( ltmp, leftFrameSync, 0, 0, frame_helper::INTER_LINEAR, undistort );
        rightConv.convert( rtmp, rightFrameSync, 0, 0, frame_helper::INTER_LINEAR, undistort );

        // Write the images to the synced output ports
        leftFrameSync.time = base::Time::now();
        rightFrameSync.time = leftFrameSync.time;
        _left_frame_sync.write(leftFrameSync);
        _right_frame_sync.write(rightFrameSync);
    }
}

void Task::initCalibration( const cv::Size imageSize )
{
    // initialize the the dense stereo lib calibration
    dense_stereo->setStereoCalibration( calibration,
            imageSize.width, imageSize.height );

    // setup frame helper for left and right
    frame_helper::StereoCalibrationCv stereoCalib;
    stereoCalib.setCalibration(calibration);
    stereoCalib.setImageSize( imageSize );
    stereoCalib.initCv();

    leftConv.setCalibrationParameter( stereoCalib.camLeft );
    rightConv.setCalibrationParameter( stereoCalib.camRight );
}

void Task::denseStereo( const cv::Mat& leftCvFrame, const cv::Mat& rightCvFrame )
{
    // create the output distanceImage and register a cv::Mat on the same
    // data buffer
    base::samples::DistanceImage distanceFrame, rightDistanceFrame;
    distanceFrame.time = leftFrameTarget.time;
    cv::Mat
        leftCvResult = dense_stereo->createLeftDistanceImage( distanceFrame ),
                     rightCvResult = dense_stereo->createRightDistanceImage( rightDistanceFrame );

    try {
        // calculate the distance images
        dense_stereo->processFramePair(leftCvFrame, rightCvFrame,
                leftCvResult, rightCvResult, true);
    } catch (std::runtime_error &e)
    {
        return;
    }

    // create disparity frame
    if(_disparity_frame.connected() || _point_cloud.connected())
    {
        base::samples::frame::Frame disparity_image(
                distanceFrame.width, distanceFrame.height, 8,
                base::samples::frame::MODE_RGB );
        disparity_image.time = leftFrameTarget.time;
        const float scaling_factor = 255.0f / *std::max_element( distanceFrame.data.begin(), distanceFrame.data.end() );
        uint8_t *data = disparity_image.getImagePtr();

        static bool lut_initialized = false;
        static uint8_t red[256];
        static uint8_t green[256];
        static uint8_t blue[256];

        if(!lut_initialized) {
            //populate some lookup tables for grayscale depth to color depth matching
            for(int i = 0; i < 256; ++i)
                red[i] = green[i] = blue[i] = 0;
            for(int i = 0; i < 64; ++i){
                red[i+95] = std::min(4*i, 255);
                green[i+31] = std::min(4*i, 255);
                blue[i] = std::min(132 + 4*i, 255);
            }
            for(int i = 0; i < 65; ++i){
                red[i+159] = 255;
                green[i+95] = 255;
                blue[i+31] = 255;
            }
            for(int i = 0; i < 64; ++i){
                if(i < 32) red[i+224] = std::max(4*(63 - i), 0);
                green[i+160] = std::max(4*(63 - i), 0);
                blue[i+96] = std::max(4*(63 - i), 0);
            }
            lut_initialized = true;
        }

        for( size_t i=0; i<distanceFrame.data.size(); ++i ) {
            if(distanceFrame.data[i] > 0){
                *(data++) = red[(uint8_t)(distanceFrame.data[i] * scaling_factor)];
                *(data++) = green[(uint8_t)(distanceFrame.data[i] * scaling_factor)];
                *(data++) = blue[(uint8_t)(distanceFrame.data[i] * scaling_factor)];
            } else { // if depth information is not valid --> black pixel
                *(data++) = 0;
                *(data++) = 0;
                *(data++) = 0;
            }
        }

        _disparity_frame.write( disparity_image );
    }

    // calculate distance images from disparity images
    dense_stereo->getDistanceImages( leftCvResult, rightCvResult );

    if(_distance_frame.connected() || _point_cloud.connected())
    {
        //write to output
        _distance_frame.write(distanceFrame);

        distance_frame_data = distanceFrame.data; 

        if (udp_config)
        {
            if (navcam)

                udp_stereo->saveVector_float(&distance_frame_data, "/var/www/exoter.com/public_html/distance_frame_navcam");

            if (loccam)

                udp_stereo->saveVector_float(&distance_frame_data, "/var/www/exoter.com/public_html/distance_frame_loccam");

            //TODO output UDP with the distanceFrame

        }
        if (_point_cloud.connected())
        {
            /** Convert to point cloud with color **/
            base::samples::Pointcloud point_cloud;
            point_cloud.time = distanceFrame.time;
            Eigen::Vector3d point;
            cv::Mat color_image_mat = frame_helper::FrameHelper::convertToCvMat(*this->leftFrame);

            /** Organize point cloud **/
            if(_organized_output_point_cloud.value())
            {
                point_cloud.points.resize(distanceFrame.width * distanceFrame.height);
                point_cloud.colors.resize(distanceFrame.width * distanceFrame.height);

                for (std::vector<base::Point>::iterator it = point_cloud.points.begin();
                        it != point_cloud.points.end(); ++it)
                {
                    (*it).z() = std::numeric_limits<double>::quiet_NaN ();
                }

                for(size_t y = 0; y < distanceFrame.height ; ++y)
                {
                    for(size_t x = 0; x < distanceFrame.width ; ++x)
                    {
                        if (distanceFrame.getScenePoint(x, y, point))
                        {
                            point_cloud.points[distanceFrame.width*y+x] = point;

                            if (_colored_output_point_cloud.value())
                            {
                                cv::Vec3b color = color_image_mat.at<cv::Vec3b>(y, x);
                                point_cloud.colors[distanceFrame.width*y+x] = base::Vector4d(color[0]/255.0, color[1]/255.0, color[2]/255.0, 1.0);
                            }
                        }
                    }
                }

            }
            else
            {
                for(size_t y = 0; y < distanceFrame.height ; ++y)
                {
                    for(size_t x = 0; x < distanceFrame.width ; ++x)
                    {
                        if (distanceFrame.getScenePoint(x, y, point))
                        {
                            point_cloud.points.push_back(point);

                            if (_colored_output_point_cloud.value())
                            {
                                cv::Vec3b color = color_image_mat.at<cv::Vec3b>(y, x);
                                point_cloud.colors.push_back(base::Vector4d(color[0]/255.0, color[1]/255.0, color[2]/255.0, 1.0));
                            }
                        }
                    }
                }
            }

            /** write to output **/
            _point_cloud.write(point_cloud);
        }
    }
}

void Task::stopHook()
{
    delete dense_stereo;

    dense_stereo = 0;

    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

