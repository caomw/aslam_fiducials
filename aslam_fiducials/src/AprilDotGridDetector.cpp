#include <aslam/AprilDotGridDetector.hpp>
#include <sm/assert_macros.hpp>
#include <AprilTags/TagDetector.h>
#include "/home/vp/ros_aslam/aslam_fiducials/aslam_fiducials/third-party/apriltags/AprilTags/TagDetection.h"
#include "/home/vp/ros_aslam/aslam_fiducials/aslam_fiducials/third-party/apriltags/AprilTags/MathUtil.h"
#include "/home/vp/ros_aslam/aslam_fiducials/aslam_fiducials/third-party/apriltags/AprilTags/GLine2D.h"
#include "/home/vp/ros_aslam/aslam_fiducials/aslam_fiducials/third-party/apriltags/AprilTags/Quad.h"

#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include "opencv2/features2d/features2d.hpp"

const char * WINDOW_NAME = "April Dot Grid Detector";
float  dist1, dist2;
int i,j;





namespace aslam {

   /*static std::pair<float,float> min_distance(std::pair<float,float> p, std::vector<cv::KeyPoint> vect)
    {
        int i=0;
        float dist;
        std::pair<float,float> pt1;
        for(i=0; i < vect.size(); i++)
        {
            cv::KeyPoint pts = vect[i];
            float dx = p.first - pts.pt.x;
            float dy = p.second - pts.pt.y;
            dist = std::sqrt(dx*dx + dy*dy);

            if(dist<10)
            {
              pt1.first = pts.pt.x;
              pt1.second = pts.pt.y;
            }
        }
        return pt1;
    }
*/
    static int FindNextPoint(std::pair<float,float> p, std::pair<float,float> v , std::vector<cv::KeyPoint> vect)
    {
        int i=0, j=0;
        float dist;
        std::pair<float,float> pn;
        pn.first = p.first + v.first;
        pn.second = p.second + v.second;
        for(i=0 ; i<vect.size() ; i++)
        {
            cv::KeyPoint pts = vect[i];
            float dx = pn.first - pts.pt.x;
            float dy = pn.second - pts.pt.y;
            dist = std::sqrt(dx*dx + dy*dy);
            if(dist < 10)
            {
                j = i;
                break;
            }
            else
            {
                j = -1;
            }

        }
        return j;
    }

    static std::vector<int> find_all_points(std::pair<float,float> p, std::pair<float,float> v , std::vector<cv::KeyPoint> vect, cv::Mat & img)
    {
        printf("p1.x = %2.2f\n p1.y =%2.2f\n", p.first, p.second);
        int index = -1;
        std::vector<int> found;
        index = FindNextPoint(p, v, vect);
        if( index == -1)
                cv::circle(img, cv::Point(25, 25), 15, cv::Scalar(0,0,255), -1);
        else
        {
            while (index != -1)
            {
                found.push_back(index);
                v.first = vect[index].pt.x - p.first;
                v.second = vect[index].pt.y - p.second;
                printf("v.x = %2.2f\n v.y = %2.2f\n", v.first, v.second);
                p.first = vect[index].pt.x;
                p.second = vect[index].pt.y;
                cv::circle(img, cv::Point(p.first, p.second), 5, cv::Scalar(0,0,255),-1);
                index = FindNextPoint(p, v, vect);

            printf("p.x = %2.2f\n p.y =%2.2f\n", p.first, p.second);

            }



        }

        return found;
    }

    AprilDotGridDetector::AprilDotGridDetector(AprilTags::TagCodes tagCodes) :
        _tagDetector( new AprilTags::TagDetector(tagCodes) ), _tagCodes(tagCodes)
    {
        // Create the named window needed to view images:
        // see: http://docs.opencv.org/modules/highgui/doc/user_interface.html
        cv::namedWindow(WINDOW_NAME); //Création d'une fenêtre pour la visualisation de l'image

    }

    AprilDotGridDetector::~AprilDotGridDetector()
    {

    }

    bool AprilDotGridDetector::findTarget(const cv::Mat & image, cameras::GridCalibrationTargetObservation & outObservation)
    {
        SM_ASSERT_EQ(std::runtime_error, image.type(), CV_8UC1, "This function only works for single channel grayscale images");

        bool success = false;

        cv::Mat imageCopy = image;  //Création d'une copie de l'image pour pouvoir l'utiliser.

        // \todo Detect the april tags
        //       see aslam_fiducials/third-party/apriltags/AprilTags/TagDetector.h

        //Détection de(s) AprilTag(s) dans l'image
        std::vector<AprilTags::TagDetection> detectedTags = _tagDetector->extractTags(imageCopy);



        // \todo Search for the dots in the dot pattern
        //       see http://www.vision.ee.ethz.ch/software/calibration_toolbox//calibration_toolbox.php
        cv::SimpleBlobDetector::Params param;
        param.filterByCircularity = true;
        //param.minCircularity = 10.0;
        //param.maxCircularity = 100.0;


        cv::Ptr<cv::FeatureDetector> blob_detector;
        blob_detector = new cv::SimpleBlobDetector(param);

        std::vector<cv::KeyPoint> blobs;
        blob_detector->detect(imageCopy, blobs);


        // DEBUG: Draw and display an image
        cv::Mat colorImage;
        cv::cvtColor(imageCopy, colorImage, CV_GRAY2RGB); //Conversion de notre image (gray) en une image couleur (RGB)

        // Now draw the results on the color image & Fill in the grid observation based on (a) the id of the april tag, and (b) the grid found.
        // http://docs.opencv.org/modules/core/doc/drawing_functions.html
        //cv::drawKeypoints(imageCopy, blobs, colorImage); // Affichage des blops sur l'image

        for(std::vector<AprilTags::TagDetection>::iterator it = detectedTags.begin(); it != detectedTags.end(); ++it)
        {
            it->draw(colorImage);
        }

        if( detectedTags.size() > 0 )
         {
            AprilTags::TagDetection corners = detectedTags[0];
            std::pair<float,float> p1 = corners.interpolate(0,0);
       //     std::pair<float,float> p11 = corners.interpolate(-0.5,-1);
       //     std::pair<float,float> p12 = corners.interpolate(0.5,-1);
            ////////////////////////////
            std::pair<float,float> p2 = corners.interpolate(0,0.5);
        //    std::pair<float,float> p21 = corners.interpolate(-0.5,-1);
       //     std::pair<float,float> p22 = corners.interpolate(0.5,-1);
            ////////////////////////////
            std::pair<float,float> p3 = corners.interpolate(-1,0);
       //     std::pair<float,float> p31 = corners.interpolate(-1,-0.5);
       //     std::pair<float,float> p32 = corners.interpolate(-1,0.5);
            ////////////////////////////
            std::pair<float,float> p4 = corners.interpolate(1,0);
      //      std::pair<float,float> p41 = corners.interpolate(1,-0.5);
     // /      std::pair<float,float> p42 = corners.interpolate(1,0.5);
     //       cv::circle(colorImage, cv::Point(p1.first, p1.second), 5, cv::Scalar(0,255,0,0), -1);
     //       cv::circle(colorImage, cv::Point(p2.first, p2.second), 5, cv::Scalar(255,255,0,0), -1);

          //  cv::KeyPoint pt1 = blobs[0];
         //   cv::circle(colorImage, cv::Point(pt1.pt.x, pt1.pt.y), 5, cv::Scalar(0,255,255,0), -1);
           // dist1 = AprilTags::MathUtil::distance2D(p1, p2);
           // dist2 = dist1/5.0;
            std::pair<float,float> pp1 = corners.interpolate(0,1);
            std::pair<float,float> V1;
            V1.first = pp1.first - p1.first;
            V1.second = pp1.second - p1.second;
            //V1.first = V.first * ((0.2+1.1)/1.1);
            //V1.second = V.second * ((0.2+1.1)/1.1);
            std::vector<int> Dots = find_all_points(p2, V1, blobs, colorImage);
            //std::vector<int> Dots = find_all_points(pp2, V1, blobs, colorImage);

            for(i=0 ; i< Dots.size() ; i++)
            {
                j = Dots[i];
                cv::KeyPoint pt1 = blobs[j];
                //cv::circle(colorImage, cv::Point(pt1.pt.x, pt1.pt.y), 5, cv::Scalar(0,255,255,0), -1);

            }

    /*        std::pair<float,float> V1, Vd, Pp1;
            V1.first = p2.first - p1.first;
            V1.second = p2.second - p1.second;

            Vd.first = V1.first * ((0.2+1.1)/1.1);
            Vd.second = V1.second * ((0.2+1.1)/1.1);

            Pp1.first = p1.first + Vd.first;
            Pp1.second = p1.second + Vd.second;
            cv::circle(colorImage, cv::Point(Pp1.first, Pp1.second), 2, cv::Scalar(255,0,0,0), -1);

            std::pair<float,float> pr1 = min_distance(Pp1, blobs);
            cv::circle(colorImage, cv::Point(pr1.first, pr1.second), 5, cv::Scalar(0,0,255,0), -1);

                                ////////////////////////////////////////
            std::pair<float,float> V2, Vd2, Pp2;
            V2.first = pr1.first - p2.first;
            V2.second = pr1.second - p2.second;
            Vd2.first = V2.first * ((0.2+0.35)/0.2);
            Vd2.second = V2.second * ((0.2+0.35)/0.2);

            Pp2.first = p2.first + Vd2.first;
            Pp2.second = p2.second + Vd2.second;
            //cv::circle(colorImage, cv::Point(Pp2.first, Pp2.second), 5, cv::Scalar(255,255,255,255), -1);

            std::pair<float,float> pr2 = min_distance(Pp2, blobs);
            cv::circle(colorImage, cv::Point(pr2.first, pr2.second), 5, cv::Scalar(0,0,255,0), -1);
                                ////////////////////////////////////////
            std::pair<float,float> V3, Vd3, Pp3;
            V3.first = pr2.first - pr1.first;
            V3.second = pr2.second - pr1.second;
            Vd3.first = V3.first * ((0.325+0.35)/0.35);
            Vd3.second = V3.second * ((0.325+0.35)/0.35);

            Pp3.first = pr1.first + Vd3.first;
            Pp3.second = pr1.second + Vd3.second;
            //cv::circle(colorImage, cv::Point(Pp3.first, Pp3.second), 5, cv::Scalar(0,0,255,0), -1);

            std::pair<float,float> pr3 = min_distance(Pp3, blobs);
            cv::circle(colorImage, cv::Point(pr3.first, pr3.second), 5, cv::Scalar(0,0,255,0), -1);
                                ////////////////////////////////////////
            std::pair<float,float> V4, Vd4, Pp4;
            V4.first = pr3.first - pr2.first;
            V4.second = pr3.second - pr2.second;
            Vd4.first = V4.first * ((0.3+0.325)/0.325);
            Vd4.second = V4.second * ((0.3+0.325)/0.325);

            Pp4.first = pr2.first + Vd4.first;
            Pp4.second = pr2.second + Vd4.second;
            //cv::circle(colorImage, cv::Point(Pp4.first, Pp4.second), 5, cv::Scalar(255,0,0,0), -1);

            std::pair<float,float> pr4 = min_distance(Pp4, blobs);
            cv::circle(colorImage, cv::Point(pr4.first, pr4.second), 5, cv::Scalar(0,0,255,0), -1);
                                /////////////////////////////////////////
            std::pair<float,float> V5, Vd5, Pp5;
            V5.first = pr4.first - pr3.first;
            V5.second = pr4.second - pr3.second;
            Vd5.first = V5.first * ((0.275+0.3)/0.3);
            Vd5.second = V5.second * ((0.275+0.3)/0.3);

            Pp5.first = pr3.first + Vd5.first;
            Pp5.second = pr3.second + Vd5.second;
            //cv::circle(colorImage, cv::Point(Pp5.first, Pp5.second), 5, cv::Scalar(255,255,255,255), -1);

            std::pair<float,float> pr5 = min_distance(Pp5, blobs);
            cv::circle(colorImage, cv::Point(pr5.first, pr5.second), 5, cv::Scalar(0,0,255,0), -1);
                                /////////////////////////////////////////
            std::pair<float,float> V6, Vd6, Pp6;
            V6.first = pr5.first - pr4.first;
            V6.second = pr5.second - pr4.second;
            Vd6.first = V6.first * ((0.25+0.275)/0.275);
            Vd6.second = V6.second * ((0.25+0.275)/0.275);

            Pp6.first = pr4.first + Vd6.first;
            Pp6.second = pr4.second + Vd6.second;
            //cv::circle(colorImage, cv::Point(Pp6.first, Pp6.second), 5, cv::Scalar(0,0,255,0), -1);

            std::pair<float,float> pr6 = min_distance(Pp6, blobs);
            cv::circle(colorImage, cv::Point(pr6.first, pr6.second), 5, cv::Scalar(0,0,255,0), -1);
                                //////////////////////////////////////////
            std::pair<float,float> V7, Vd7, Pp7;
            V7.first = pr6.first - pr5.first;
            V7.second = pr6.second - pr5.second;
            Vd7.first = V7.first * ((0.225+0.25)/0.25);
            Vd7.second = V7.second * ((0.225+0.25)/0.25);

            Pp7.first = pr5.first + Vd7.first;
            Pp7.second = pr5.second + Vd7.second;
            //cv::circle(colorImage, cv::Point(Pp7.first, Pp7.second), 5, cv::Scalar(0,255,255,0), -1);

            std::pair<float,float> pr7 = min_distance(Pp7, blobs);
            cv::circle(colorImage, cv::Point(pr7.first, pr7.second), 5, cv::Scalar(0,0,255,0), -1);
                                /////////////////////////////////////////
            std::pair<float,float> V8, Vd8, Pp8;
            V8.first = pr7.first - pr6.first;
            V8.second = pr7.second - pr6.second;
            Vd8.first = V8.first * ((0.2+0.225)/0.225);
            Vd8.second = V8.second * ((0.2+0.225)/0.225);

            Pp8.first = pr6.first + Vd8.first;
            Pp8.second = pr6.second + Vd8.second;
            //cv::circle(colorImage, cv::Point(Pp8.first, Pp8.second), 5, cv::Scalar(255,0,255,0), -1);

            std::pair<float,float> pr8 = min_distance(Pp8, blobs);
            cv::circle(colorImage, cv::Point(pr8.first, pr8.second), 5, cv::Scalar(0,0,255,0), -1);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            std::pair<float,float> U1, Ud1, Ptp1;
            U1.first = p4.first - p3.first;
            U1.second = p4.second - p3.second;

            Ud1.first = U1.first * ((0.2+1.1)/1.1);
            Ud1.second = U1.second * ((0.2+1.1)/1.1);

            Ptp1.first = p3.first + Ud1.first;
            Ptp1.second = p3.second + Ud1.second;

            std::pair<float,float> pr21 = min_distance(Ptp1, blobs);
            cv::circle(colorImage, cv::Point(pr21.first, pr21.second), 5, cv::Scalar(0,0,255,0), -1);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            std::pair<float,float> U2, Ud2, Ptp2;
            U2.first = p3.first - p4.first;
            U2.second = p3.second - p4.second;

            Ud2.first = U2.first * ((0.2+1.1)/1.1);
            Ud2.second = U2.second * ((0.2+1.1)/1.1);

            Ptp2.first = p4.first + Ud2.first;
            Ptp2.second = p4.second + Ud2.second;

            std::pair<float,float> pr22 = min_distance(Ptp2, blobs);
            cv::circle(colorImage, cv::Point(pr22.first, pr22.second), 5, cv::Scalar(0,0,255,0), -1);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            std::pair<float,float> U3, Ud3, Ptp3;
            U3.first = p1.first - p2.first;
            U3.second = p1.second - p2.second;

            Ud3.first = U3.first * ((0.2+1.1)/1.1);
            Ud3.second = U3.second * ((0.2+1.1)/1.1);

            Ptp3.first = p2.first + Ud3.first;
            Ptp3.second = p2.second + Ud3.second;

            std::pair<float,float> pr23 = min_distance(Ptp3, blobs);
            cv::circle(colorImage, cv::Point(pr23.first, pr23.second), 5, cv::Scalar(0,0,255,0), -1);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            std::pair<float,float> V11, Vd11, Pp11;
            V11.first = p21.first - p11.first;
            V11.second = p21.second - p11.second;

            Vd11.first = V11.first * ((0.2+1.1)/1.1);
            Vd11.second = V11.second * ((0.2+1.1)/1.1);

            Pp11.first = p11.first + Vd.first;
            Pp11.second = p11.second + Vd.second;

            std::pair<float,float> ptr11 = min_distance(Pp11, blobs);
            cv::circle(colorImage, cv::Point(ptr11.first, ptr11.second), 5, cv::Scalar(0,0,255,0), -1);
                                        /////////////////////////////////////
             std::pair<float,float> V21, Vd21, Pp21;
            V21.first = p22.first - p12.first;
            V21.second = p22.second - p12.second;

            Vd21.first = V21.first * ((0.2+1.1)/1.1);
            Vd21.second = V21.second * ((0.2+1.1)/1.1);

            Pp21.first = p12.first + Vd21.first;
            Pp21.second = p12.second + Vd21.second;

            std::pair<float,float> ptr12 = min_distance(Pp21, blobs);
            cv::circle(colorImage, cv::Point(ptr12.first, ptr12.second), 5, cv::Scalar(0,0,255,0), -1);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

           for(j=2 ; j<21 ; j++)
            {
                std::pair<float,float> Vd4, Pp4;
                Vd4.first = V1.first * (((j*0.35)+0.55+1.1)/1.1);
                Vd4.second = V1.second * (((j*0.35)+0.55+1.1)/1.1);

                Pp4.first = p1.first + Vd4.first;
                Pp4.second = p1.second + Vd4.second;
                cv::circle(colorImage, cv::Point(Pp4.first, Pp4.second), 5, cv::Scalar(0,255,255,0), -1);
            }
*/


         //  cv::circle(colorImage, cv::Point(p2.first, p2.second - dist2), 5, cv::Scalar(255,255,255,255), -1);
         //   int result = blobs.size();
         //   if( !blobs.empty() )
          //  {
          //      for(i=0; i<result; i++)
         //       {

         //               std::pair<float,float> p3;
        //                p3.first = blobs[i].pt.x;
        //                p3.second = blobs[i].pt.y;
        //                if(AprilTags::MathUtil::distance2D(p3, p2) < dist2)
         //               {
         //                   pt1 = blobs[i];
         //                   cv::circle(colorImage, cv::Point(pt1.pt.x, pt1.pt.y), 5, cv::Scalar(0,255,255,0), -1);

         //               }


              //  }

            //}



        }




        // Display the image
        cv::imshow(WINDOW_NAME, colorImage); //Affichage de l'image finale avec détection

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

