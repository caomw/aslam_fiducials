#include <aslam/AprilDotGridDetector.hpp>
#include <sm/assert_macros.hpp>

const char * WINDOW_NAME = "April Dot Grid Detector";

namespace aslam {
    
    AprilDotGridDetector::AprilDotGridDetector(AprilTags::TagCodes tagCodes) :
        _tagDetector( new AprilTags::TagDetector(tagCodes) ), _tagCodes(tagCodes)
    {
        // Create the named window needed to view images:
        // see: http://docs.opencv.org/modules/highgui/doc/user_interface.html
        cv::namedWindow(WINDOW_NAME);

    }
    
    AprilDotGridDetector::~AprilDotGridDetector()
    {

    }

    bool AprilDotGridDetector::findTarget(const cv::Mat & image, cameras::GridCalibrationTargetObservation & outObservation)
    {
        SM_ASSERT_EQ(std::runtime_error, image.type(), CV_8UC1, "This function only works for single channel grayscale images");

        bool success = false;

        // \todo Detect the april tags
        //       see aslam_fiducials/third-party/apriltags/AprilTags/TagDetector.h
        
        // \todo Search for the dots in the dot pattern
        //       see http://www.vision.ee.ethz.ch/software/calibration_toolbox//calibration_toolbox.php


        // \todo Fill in the grid observation based on (a) the id of the april tag, and (b) the grid found
        

        // DEBUG: Draw and display an image
        cv::Mat colorImage;
        cv::cvtColor( image, colorImage, CV_GRAY2RGB );
        
        // Now draw the results on the color image.
        // http://docs.opencv.org/modules/core/doc/drawing_functions.html
        
        // Display the image
        cv::imshow(WINDOW_NAME, colorImage);
        cv::waitKey(10);

        return success;
    }

    bool AprilDotGridDetector::findTargetEigen(const Eigen::Matrix<boost::uint8_t, Eigen::Dynamic, Eigen::Dynamic> & image, 
                                               cameras::GridCalibrationTargetObservation & outObservation)
    {
        // Mat(int _rows, int _cols, int _type, void* _data, size_t _step=AUTO_STEP);
        cv::Mat mat(image.rows(), image.cols(), CV_8UC1, const_cast<void*>(reinterpret_cast<const void *>(&image(0,0))));
        return findTarget(mat, outObservation);
    }



} // namespace aslam
