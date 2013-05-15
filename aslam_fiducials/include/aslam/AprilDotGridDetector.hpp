#ifndef ASLAM_APRIL_DOT_GRID_DETECTOR_HPP
#define ASLAM_APRIL_DOT_GRID_DETECTOR_HPP

#include <aslam/cameras.hpp>
#include <aslam/cameras/GridCalibrationTarget.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
#include <opencv2/highgui/highgui.hpp>


namespace aslam {

    class AprilDotGridDetector
    {
    public:
        SM_DEFINE_EXCEPTION(Exception, std::runtime_error);


        // \todo You may need more arguments here, such as the number of dots in the pattern
        AprilDotGridDetector(AprilTags::TagCodes tagCodes = AprilTags::tagCodes36h11);
        virtual ~AprilDotGridDetector();

        // \todo fill in this function
        bool findTarget(const cv::Mat & image, cameras::GridCalibrationTargetObservation & outObservation);

        // this function is just for convenience.
        bool findTargetEigen(const Eigen::Matrix<boost::uint8_t, Eigen::Dynamic, Eigen::Dynamic> & image,
                             cameras::GridCalibrationTargetObservation & outObservation);


    private:
        boost::shared_ptr<AprilTags::TagDetector> _tagDetector;
        AprilTags::TagCodes _tagCodes;

        // \todo you may need more members here such as
        // the target that defines the grid geometry
        // see aslam_cameras/include/aslam/cameras/GridCalibrationTarget.hpp
        // boost::shared_ptr<aslam::cameras::GridCalibrationTarget> _target


    };


} // namespace aslam


#endif /* ASLAM_APRIL_DOT_GRID_DETECTOR_HPP */
