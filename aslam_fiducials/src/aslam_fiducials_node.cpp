#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <aslam/AprilDotGridDetector.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

namespace aslam {

    class AprilGridDetectorNode
    {
    public:
        AprilGridDetectorNode(const ros::NodeHandle & nh) : _nh(nh), _it(_nh){
            // Set up publishers and subscribers

            ROS_INFO_STREAM("Setting up image subscriber");
            // This sets up a callback when a new image arrives.
            _imageSubscriber = _it.subscribe("image", 1, &AprilGridDetectorNode::imageCallback,this);
            ROS_INFO_STREAM( "Successfully set up the image subscriber" );
            std::string im = _nh.resolveName("image");
            ROS_INFO_STREAM( "image: " << im );

            // Here you would get the parameters you need to create a new detector
            // using _nh.param( ... )
            // http://www.ros.org/wiki/roscpp/Overview/Parameter%20Server
            // 

            // Then you create the detector:
            // in the future, fill in the constructor with parameters.
            _detector.reset( new AprilDotGridDetector );

        }
        virtual ~AprilGridDetectorNode(){}

        void imageCallback(const sensor_msgs::ImageConstPtr& msg){

            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            cv::Mat image = cv_ptr->image;

            cameras::GridCalibrationTargetObservation obs;
            // This calls the detector to fill in the calibration target.
            bool success = _detector->findTarget(image, obs);

            // Was the detector successful?
            ROS_INFO_STREAM("Detector, success: " << success );

            
        }

        void spin() {
            // Give control of the program over to ROS
            // so that it can call the image callback
            ros::spin();
        }

        ros::NodeHandle _nh;
        image_transport::Subscriber _imageSubscriber;
        image_transport::ImageTransport _it;
        boost::shared_ptr< AprilDotGridDetector > _detector;
    };
    
} // namespace aslam

int main(int argc, char ** argv)
{
    try {
        ros::init(argc, argv, "aslam_fiducials_node");
        
        aslam::AprilGridDetectorNode node(ros::NodeHandle("~"));
        
        node.spin();
        

    } catch( const std::exception & e ) {
        ROS_ERROR_STREAM("Exception: " << e.what());
        return 1;
    }

    return 0;
}
