/*
 * Copyright 2016, Simula Research Laboratory
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "cctag/Detection.hpp"
#include "cctag/utils/Exceptions.hpp"
#include "cctag/utils/FileDebug.hpp"
#include "cctag/utils/VisualDebug.hpp"

#ifdef CCTAG_WITH_CUDA
#include "cctag/cuda/debug_macros.hpp"
#include "cctag/cuda/device_prop.hpp"
#endif // CCTAG_WITH_CUDA

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/exception/all.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/ptr_container/ptr_list.hpp>
#include <boost/timer/timer.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <tbb/tbb.h>

#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#define PRINT_TO_CERR

using namespace cctag;

namespace bfs = boost::filesystem;

/**
 * @brief Check if a string is an integer number.
 *
 * @param[in] s The string to check.
 * @return Return true if the string is an integer number
 */
bool isInteger(std::string& s) { return (s.size() == 1 && std::isdigit(s[0])); }

/**
 * @brief Draw the detected marker int the given image. The markers are drawn as a
 * circle centered in the center of the marker and with its id. It draws the
 * well identified markers in green, the unknown / badly detected markers in red.
 *
 * @param[in] markers The list of markers to draw.
 * @param[out] image The image in which to draw the markers.
 */
void drawMarkers(const boost::ptr_list<CCTag>& markers, cv::Mat& image, bool showUnreliable = true)
{
    for(const cctag::CCTag& marker : markers)
    {
        const cv::Point center = cv::Point(marker.x(), marker.y());
        const int radius = 10;
        const int fontSize = 3;
        if(marker.getStatus() == status::id_reliable)
        {
            const cv::Scalar color = cv::Scalar(0, 255, 0, 255);
            const auto rescaledOuterEllipse = marker.rescaledOuterEllipse();

            cv::circle(image, center, radius, color, 3);
            cv::putText(image, std::to_string(marker.id()), center, cv::FONT_HERSHEY_SIMPLEX, fontSize, color, 3);
            cv::ellipse(image,
                        center,
                        cv::Size(rescaledOuterEllipse.a(), rescaledOuterEllipse.b()),
                        rescaledOuterEllipse.angle() * 180 / boost::math::constants::pi<double>(),
                        0,
                        360,
                        color,
                        3);
        }
        else if(showUnreliable)
        {
            const cv::Scalar color = cv::Scalar(0, 0, 255, 255);
            cv::circle(image, center, radius, color, 2);
            cv::putText(image, std::to_string(marker.id()), center, cv::FONT_HERSHEY_SIMPLEX, fontSize, color, 3);
        }
    }
}

/**
 * @brief Extract the cctag from an image.
 *
 * @param[in] frameId The number of the frame.
 * @param[in] pipeId The pipe id (used for multiple streams).
 * @param[in] cv_ptr->image The image to process.
 * @param[in] params The parameters for the detection.
 * @param[in] bank The marker bank.
 * @param[out] markers The list of detected markers.
 * @param[out] outStream The output stream on which to write debug information.
 * @param[out] debugFileName The filename for the image to save with the detected
 * markers.
 */
void detection(std::size_t frameId,
               int pipeId,
               const cv::Mat& src,
               const cctag::Parameters& params,
               const cctag::CCTagMarkersBank& bank,
               boost::ptr_list<CCTag>& markers,
               std::ostream& outStream,
               std::string debugFileName = "")
{
    if(debugFileName.empty())
    {
        debugFileName = "00000";
    }

    // Process markers detection
    boost::timer::cpu_timer t;

    CCTagVisualDebug::instance().initBackgroundImage(src);
    CCTagVisualDebug::instance().setImageFileName(debugFileName);
    CCTagFileDebug::instance().setPath(CCTagVisualDebug::instance().getPath());

    static cctag::logtime::Mgmt* durations = nullptr;

    // Call the main CCTag detection function
    cctagDetection(markers, pipeId, frameId, src, params, bank, true, durations);

    if(durations)
    {
        durations->print(std::cerr);
    }

    CCTagFileDebug::instance().outPutAllSessions();
    CCTagFileDebug::instance().clearSessions();
    CCTagVisualDebug::instance().outPutAllSessions();
    CCTagVisualDebug::instance().clearSessions();

    std::cout << "Total time: " << t.format() << std::endl;
    CCTAG_COUT_NOENDL("Id : ");

    std::size_t counter = 0;
    std::size_t nMarkers = 0;
    outStream << "#frame " << frameId << '\n';
    outStream << "Detected " << markers.size() << " candidates" << '\n';

    for(const cctag::CCTag& marker : markers)
    {
        outStream << marker.x() << " " << marker.y() << " " << marker.id() << " " << marker.getStatus() << '\n';
        ++counter;
        if(marker.getStatus() == status::id_reliable)
            ++nMarkers;
    }

    counter = 0;
    for(const cctag::CCTag& marker : markers)
    {
        if(counter == 0)
        {
            CCTAG_COUT_NOENDL(marker.id() + 1);
        }
        else
        {
            CCTAG_COUT_NOENDL(", " << marker.id() + 1);
        }
        ++counter;
    }

    std::cout << std::endl << nMarkers << " markers detected and identified" << std::endl;
}

/*************************************************************/
/*                    Main entry                             */

/*************************************************************/

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cctag::Parameters params(3);

    CCTagMarkersBank bank(params._nCrowns);

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat display;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        if(cv_ptr->image.channels() != 1)
        {
            display = cv_ptr->image;
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2GRAY);
            ROS_INFO("gray");
        } else
        {
            cv::cvtColor(cv_ptr->image, display, cv::COLOR_GRAY2BGRA);
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    boost::ptr_list<CCTag> markers;
    detection(msg->header.seq, 0, cv_ptr->image, params, bank, markers, std::cerr);

    drawMarkers(markers, display, true);
    cv::imshow("Detections", display);
    cv::waitKey(100);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "cctag");

    ros::NodeHandle pnh("~");
    ros::Subscriber image_sub = pnh.subscribe("/camera1/infra1/image_rect_raw", 1, imageCallback);

    ros::spin();

    return 0;
}
