#include <aslam/AprilDotGridDetector.hpp>
#include <sm/assert_macros.hpp>
#include <AprilTags/TagDetector.h>
#include "/home/vp/ros_aslam/aslam_fiducials/aslam_fiducials/third-party/apriltags/AprilTags/TagDetection.h"

const char * WINDOW_NAME = "April Dot Grid Detector";
const char * G_O = "Grid Observation";

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

        cv::Mat imageCopy = image;

        // \todo Detect the april tags
        //       see aslam_fiducials/third-party/apriltags/AprilTags/TagDetector.h



        std::vector<AprilTags::TagDetection> detectedTags = _tagDetector->extractTags(imageCopy);


        //AprilTags::TagDetection::draw( imageCopy );
        // \todo Search for the dots in the dot pattern
        //       see http://www.vision.ee.ethz.ch/software/calibration_toolbox//calibration_toolbox.php


     //   cv::Mat gray;
    //    vector<cv::Vec3f> circles;
   //     HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, gray->rows/4, 200, 100 );


        // \todo Fill in the grid observation based on (a) the id of the april tag, and (b) the grid found



        // DEBUG: Draw and display an image
        cv::Mat colorImage;
        cv::cvtColor(imageCopy, colorImage, CV_GRAY2RGB);

        // Now draw the results on the color image.
        // http://docs.opencv.org/modules/core/doc/drawing_functions.html

        for(std::vector<AprilTags::TagDetection>::iterator it = detectedTags.begin(); it != detectedTags.end(); ++it)
        {
            it->draw(colorImage);

        }


      // cv::circle(colorImage,cv::Point(250,512),30,cv::Scalar(0,0,255));
      // cv::rectangle(colorImage, p1, p4, cv::Scalar(0,255,255));
     //   cv::line(colorImage,p1,p2,/*color*/)
//        for( size_t i = 0; i < circles.size(); i++ )
//        {
//             float* p = (float*)cvGetSeqElem(circles, i);
//           cv::Point center(cvRound(p[0]), cvRound(p[1]));
//         int radius = cvRound(p[2]);
//          // draw the circle outline
//            circle( colorImage, center, radius, cv::Scalar(0,0,255));
//        }







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
