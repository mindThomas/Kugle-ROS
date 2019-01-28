/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details. 
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 
#include "Trajectory.h"
#include "Path.h"

#include <string>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

/* For visualization/plotting only */
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define deg2rad(x) (M_PI*x/180.f)
#define rad2deg(x) (180.f*x/M_PI)

namespace MPC
{

    double TrajectoryPoint::EuclideanDistance(const TrajectoryPoint& other)
    {
        return sqrt( (point[0]-other.point[0])*(point[0]-other.point[0]) + (point[1]-other.point[1])*(point[1]-other.point[1]) );
    }

    double TrajectoryPoint::EuclideanDistance(const Eigen::Vector2d& other)
    {
        return sqrt( (point[0]-other[0])*(point[0]-other[0]) + (point[1]-other[1])*(point[1]-other[1]) );
    }

    Trajectory::Trajectory() : lastSeq_(-1), sorted_(false)
    {

    }

    Trajectory::Trajectory(std::vector<TrajectoryPoint>& points) : lastSeq_(-1), sorted_(false)
    {
        points_ = points;
        // Find largest sequence number
        for (auto& p : points_) {
            if (p.seq > lastSeq_) lastSeq_ = p.seq;
        }
    }

    Trajectory::Trajectory(TrajectoryPoint points[], int num_points) : lastSeq_(-1), sorted_(false)
    {
        points_.insert(points_.end(), points, points + num_points);
        // Find largest sequence number
        for (auto& p : points_) {
            if (p.seq > lastSeq_) lastSeq_ = p.seq;
        }
    }

    Trajectory::~Trajectory()
    {

    }

    // Assignment operator
    Trajectory& Trajectory::operator=(const Trajectory& other)
    {
        points_ = other.points_;
        sorted_ = other.sorted_;
        // Find largest sequence number
        for (auto& p : points_) {
            if (p.seq > lastSeq_) lastSeq_ = p.seq;
        }
        return *this;
    }

    void Trajectory::AddPoint(TrajectoryPoint& point)
    {
        points_.push_back(point);
        if (point.seq > lastSeq_) lastSeq_ = point.seq;
        sorted_ = false;
    }

    void Trajectory::AddPoint(Eigen::Vector2d point, bool goal, double heading, double velocity)
    {
        points_.push_back(TrajectoryPoint(++lastSeq_, point[0], point[1], goal, heading, velocity));
        sorted_ = false;
    }

    void Trajectory::AddPoint(double x, double y, bool goal, double heading, double velocity)
    {
        points_.push_back(TrajectoryPoint(++lastSeq_, x, y, goal, heading, velocity));
        sorted_ = false;
    }

    void Trajectory::AddPoint(int seq, double x, double y, bool goal, double heading, double velocity)
    {
        points_.push_back(TrajectoryPoint(seq, x, y, goal, heading, velocity));
        if (seq > lastSeq_) lastSeq_ = seq;
        sorted_ = false;
    }

    void Trajectory::clear()
    {
        points_.clear();
        lastSeq_ = -1;
        sorted_ = false;
    }

    size_t Trajectory::size()
    {
        return points_.size();
    }

    void Trajectory::rotate(Trajectory& rotatedTrajectory, double angle)
    {
        rotatedTrajectory.clear();
        //Eigen::Rotation2Df rot(angle);
        Eigen::Matrix2d rot = Eigen::Rotation2Dd(angle).toRotationMatrix();

        for (auto& p : points_) {
            rotatedTrajectory.AddPoint(rot*p.point, p.goal, p.heading, p.velocity);
        }
    }

    // Inline rotation of current trajectory
    void Trajectory::rotate(double angle)
    {
        Eigen::Matrix2d rot = Eigen::Rotation2Dd(angle).toRotationMatrix();

        for (auto& p : points_) {
            p.point = rot*p.point;
        }
    }

    void Trajectory::move(double x, double y)
    {
        move(Eigen::Vector2d(x, y));
    }

    void Trajectory::move(Eigen::Vector2d offset)
    {
        for (auto& p : points_) {
            p.point += offset;
        }
    }

    void Trajectory::scale(double scale_factor)
    {
        for (auto& p : points_) {
            p.point *= scale_factor;
        }
    }

    void Trajectory::move(Trajectory& movedTrajectory, double x, double y)
    {
        Eigen::Vector2d offset(x, y);
        movedTrajectory.clear();

        for (auto& p : points_) {
            movedTrajectory.AddPoint(p.point += offset, p.goal, p.heading, p.velocity);
        }
        movedTrajectory.sorted_ = sorted_;
    }

    void Trajectory::PositionExtract(Trajectory& extractedTrajectory, Eigen::Vector2d p_min, Eigen::Vector2d p_max)
    {
        extractedTrajectory.clear();
        for (auto& p : points_) {
            if (p.point[0] >= p_min[0] && p.point[1] >= p_min[1] && p.point[0] <= p_max[0] && p.point[1] <= p_max[1])
                extractedTrajectory.AddPoint(p);
        }
        extractedTrajectory.sorted_ = false;
    }

    void Trajectory::PositionExtract(Trajectory& extractedTrajectory, double x_min, double y_min, double x_max, double y_max)
    {
        PositionExtract(extractedTrajectory, Eigen::Vector2d(x_min,y_min), Eigen::Vector2d(x_max,y_max));
    }

    void Trajectory::WindowExtract(Trajectory& extractedTrajectory, Eigen::Vector2d center, double heading, double width, double height, Eigen::Vector2d offset)
    {
        Trajectory trajectory = *this;

        trajectory.move(-center);
        trajectory.rotate(-heading);

        trajectory.PositionExtract(extractedTrajectory, -height/2, -width/2, height/2, width/2);
        if (offset[0] != 0 || offset[1] != 0)
            extractedTrajectory.move(-offset); // offset defines center of robot in the window
    }

    void Trajectory::DistanceExtract(Trajectory& extractedTrajectory, double distance_max, double distance_min)
    {
        double distance_min_squared = distance_min*distance_min;
        double distance_max_squared = distance_max*distance_max;
        extractedTrajectory.clear();
        for (auto& p : points_) {
            double distance_squared = p.point[0]*p.point[0] + p.point[1]*p.point[1];
            if (distance_squared >= distance_min_squared && distance_squared <= distance_max_squared)
                extractedTrajectory.AddPoint(p);
        }
        extractedTrajectory.sorted_ = false;
    }

    void Trajectory::ApproxCurveLengthExtract(Trajectory& extractedTrajectory, double curve_length_max)
    {
        double curve_length_current = 0;
        TrajectoryPoint p_prev = points_.at(0);

        extractedTrajectory.clear();
        for (auto& p : points_) {
            double curve_length = p.EuclideanDistance(p_prev);
            curve_length_current += curve_length;
            extractedTrajectory.AddPoint(p);

            if (curve_length_current >= curve_length_max)
                break;

            p_prev = p;
        }
        extractedTrajectory.sorted_ = false;
    }

    void Trajectory::SequenceExtract(Trajectory& extractedTrajectory, int seq_min, int seq_max)
    {
        extractedTrajectory.clear();
        for (auto& p : points_) {
            if (p.seq >= seq_min && p.seq <= seq_max)
                extractedTrajectory.AddPoint(p);
        }
        extractedTrajectory.sorted_ = false;
    }

    bool Trajectory::find(int seq, TrajectoryPoint& foundPoint)
    {
        for (auto& p : points_) {
            if (p.seq != -1 && p.seq == seq) {
                foundPoint = p;
                return true;
            }
        }
        return false;
    }

    bool Trajectory::find(int seq)
    {
        TrajectoryPoint foundPoint;
        return find(seq, foundPoint);
    }

    bool Trajectory::includesGoal()
    {
        // Check whether the current trajectory includes a goal point
        for (auto& p : points_) {
            if (p.goal)
                return true;
        }
        return false;
    }

    void Trajectory::print()
    {
        std::cout << "Trajectory points:" << std::endl;
        for (auto& p : points_) {
            std::cout << "   #" << p.seq << "  [" << p.point[0] << ", " << p.point[1] << "]" << std::endl;
        }
        std::cout << std::endl;
    }

    void Trajectory::sort()
    {
        /* Sort current trajectory based on sequence number */
        std::sort(points_.begin(), points_.end());
        sorted_ = true;
    }

    double Trajectory::distance()
    {
        /* Computes total trajectory distance from first point (smallest sequence number) to last point (largest sequence number) */
        if (!sorted_) sort();
        double distance = 0;
        TrajectoryPoint prevPoint;

        for (auto& p : points_) {
            if (prevPoint.seq != -1) {
                Eigen::Vector2d diff = p.point - prevPoint.point;
                distance += diff.norm();
            }
            prevPoint = p;
        }

        return distance;
    }

    int Trajectory::GetLastSequenceID()
    {
        return lastSeq_;
    }

    std::vector<double> Trajectory::GetDistanceList()
    {
        /* Computes trajectory distance from first point to given point in the (sorted) trajectory */
        std::vector<double> distanceList;

        if (!sorted_) sort();
        double distance = 0;
        TrajectoryPoint prevPoint = points_.at(0);

        for (auto& p : points_) {
            Eigen::Vector2d diff = p.point - prevPoint.point;
            distance += diff.norm();
            prevPoint = p;
            distanceList.push_back(distance);
        }

        return distanceList;
    }

    std::vector<double> Trajectory::GetX()
    {
        std::vector<double> xValues;
        for (auto &p : points_) {
            xValues.push_back(p.point[0]);
        }
        return xValues;
    }

    std::vector<double> Trajectory::GetY()
    {
        std::vector<double> yValues;
        for (auto &p : points_) {
            yValues.push_back(p.point[1]);
        }
        return yValues;
    }

    TrajectoryPoint Trajectory::get(unsigned int index)
    {
        TrajectoryPoint emptyPoint;
        if (index < 0 || index >= points_.size()) return emptyPoint;
        return points_.at(index);
    }

    TrajectoryPoint Trajectory::back()
    {
        return points_.back();
    }

    void Trajectory::plot(cv::Mat& image, cv::Scalar color, bool drawXup, bool plotText, double x_min, double y_min, double x_max, double y_max)
    {
        double xres = image.cols;
        double yres = image.rows;

        cv::line(image, cv::Point(0, yres/2), cv::Point(xres-1, yres/2), cv::Scalar(128,128,128), 1, 8, 0);
        cv::line(image, cv::Point(xres/2, 0), cv::Point(xres/2, yres-1), cv::Scalar(128,128,128), 1, 8, 0);

        // Scale range (x_min:x_max) and (y_min:y_max) to (0:499)
        double scale_x = xres / (x_max - x_min);
        double scale_y = yres / (y_max - y_min);
        double center_x = (x_min + x_max) / 2.0;
        double center_y = (y_min + y_max) / 2.0;

        for (auto& p : points_) {
            std::ostringstream text;
            text << p.seq;

            float x = (p.point[0]-x_min) * scale_x;
            float y = (p.point[1]-y_min) * scale_y;

            if (x >= 0 && x < xres && y >= 0 && y < yres) {
                cv::Point point;
                if (drawXup) // draw with robot x-axis pointing up in plot
                    point = cv::Point(yres-y,xres-x);
                else
                    point = cv::Point(x,yres-y);
                cv::drawMarker(image, point, color, cv::MARKER_CROSS, 3, 2, 8);
                if (plotText) {
                    cv::putText(image, text.str(), point + cv::Point(10, 10),
                                cv::FONT_HERSHEY_PLAIN, 1.5, cvScalar(0, 0, 0), 1.5, CV_AA);
                }
            }
        }
    }

    void Trajectory::plot(bool drawXup, bool plotText, double x_min, double y_min, double x_max, double y_max)
    {
        double aspect_ratio = (x_max - x_min) / (y_max - y_min);
        double xres, yres;
        if (aspect_ratio >= 1) {
            yres = 500;
            xres = yres * aspect_ratio;
        } else {
            xres = 500;
            yres = xres / aspect_ratio;
        }

        // Create black empty images
        cv::Mat image;
        if (drawXup)
            image = cv::Mat( xres, yres, CV_8UC3, cv::Scalar( 255, 255, 255 ) );
        else
            image = cv::Mat( yres, xres, CV_8UC3, cv::Scalar( 255, 255, 255 ) );

        // Scale range (x_min:x_max) and (y_min:y_max) to (0:499)
        double scale_x = xres / (x_max - x_min);
        double scale_y = yres / (y_max - y_min);
        double center_x = (x_min + x_max) / 2.0;
        double center_y = (y_min + y_max) / 2.0;

        for (auto& p : points_) {
            std::ostringstream text;
            text << p.seq;

            float x = (p.point[0]-x_min) * scale_x;
            float y = (p.point[1]-y_min) * scale_y;

            if (x >= 0 && x < xres && y >= 0 && y < yres) {
                cv::Point point;
                if (drawXup) // draw with robot x-axis pointing up in plot
                    point = cv::Point(yres-y,xres-x);
                else
                    point = cv::Point(x,yres-y);
                cv::drawMarker(image, point, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 3, 2, 8);
                if (plotText) {
                    cv::putText(image, text.str(), point + cv::Point(10, 10),
                                cv::FONT_HERSHEY_PLAIN, 1.5, cvScalar(0, 0, 0), 1.5, CV_AA);
                }
            }
        }

        cv::imshow("Trajectory", image);

        cv::waitKey( 5 );
    }

    Trajectory Trajectory::GenerateTestTrajectory(void)
    {
        Trajectory trajectory;

        double r = 20;

        Eigen::Vector2d p;
        for (unsigned int i = 0; i < (100+50+100+50-1); i++) {
            if (i < 100) { //  straight x=20, y=-50...50
                p << 20, (double(i) - 50);
            } else if (i < 100+50) { // left turn 180 degree with radius, r
                p << (20 - r), 50;
                p += r * Eigen::Vector2d(cos(M_PI / 50 * (double(i) - 100)), sin(M_PI / 50 * (double(i) - 100)));
            } else if (i < 100+50+100) { // straight x=-20, y=50...-50
                p << -20, (50 - (double(i) - 100 - 50));
            } else if (i < 100+50+100+50) { // left turn 180 degree with radius, r
                p << 20 - r, -50;
                p += r * Eigen::Vector2d(cos(M_PI / 50 * (double(i) - 100 - 50 - 100 + 50)),
                                         sin(M_PI / 50 * (double(i) - 100 - 50 - 100 + 50)));
            } else {
                continue;
            }
            trajectory.AddPoint(p);
        }

        trajectory.rotate(deg2rad(90)); // Rotate trajectory 90 degree
        trajectory.scale(1.0/10); //  downscale trajectory

        return trajectory;
    }

    void Trajectory::FixSequenceOrder(Trajectory& correctedTrajectory, int startSeqID, int endSeqID)
    {
        // If the trajectory is looping, the window trajectory can include jumps from a small sequence ID to a large sequence ID because the trajectory is incorrectly ordered/sorted
        // This function will both sort the trajectory and fix these types of jumps, such that the first element in the trajectory will indeed be the first element of a continuous sequence
        // So if the trajectory includes both start and end, the first element will be on the end part, since this is what is continuous with the start part
        sort();

        // Check if the trajectory includes both the start and end index
        TrajectoryPoint startPoint, endPoint;
        if (find(startSeqID, startPoint) && find(endSeqID, endPoint)) {
            // Ensure that the start and end point are not belonging to the actual start and end of the extracted trajectory - as we should not reorder anything in this case
            /*if (startPoint.seq == points_.front().seq && endPoint.seq == points_.back().seq) {
                correctedTrajectory = *this;
                return;
            }*/

            // Find the element at which the jump happens closest to the end point


            int prevSeqID = points_.at(0).seq;
            int jumpIndex = 0;
            for (int i = 0; i < points_.size(); i++) {
                if (points_[i].seq == endPoint.seq) break; // stop looking for jumps when we reach the end point
                if ((points_[i].seq - prevSeqID) > 1) {
                    jumpIndex = i;
                }
                prevSeqID = points_[i].seq;
            }

            if (jumpIndex == 0) { // there is no need for begin/end reordering, since the start and end indices are not both within the trajectory
                correctedTrajectory = *this;
                return;
            }

            // Now create a corrected trajectory where the sequence order is fixed
            correctedTrajectory.clear();
            for (int i = jumpIndex; i < points_.size(); i++)
                correctedTrajectory.AddPoint(points_[i]);
            for (int i = 0; i < jumpIndex; i++) {
                TrajectoryPoint pTmp = points_[i];
                pTmp.seq += endSeqID + 1; // make the ending sequence part of the trajectory have greater sequence ID's such that the whole trajectory sequence becomes a series of continuous ID's
                correctedTrajectory.AddPoint(pTmp);
            }

            return;
        }
        else { // there is no need for begin/end reordering, since the start and end indices are not both within the trajectory
            correctedTrajectory = *this;
            return;
        }
    }

    void Trajectory::ExtractContinuousClosestSequence(Trajectory& continuousSequenceTrajectory, Eigen::Vector2d position)
    {
        // Extract a continuous sequence of trajectory points closest to the input position
        if (points_.size() <= 0) return; // trajectory is empty

        // Find indices where there is a jump in the sequence index
        int prevSeqID = points_.at(0).seq;
        std::vector<int> jumpIndex;
        jumpIndex.push_back(0);
        jumpIndex.push_back(points_.size());
        for (int i = 0; i < points_.size(); i++) {
            if ((points_[i].seq - prevSeqID) > 1) {
                jumpIndex.push_back(i); // jump indices are the index for the element AFTER a jump occured - hence a jump happens between points_[jumpIndex-1] and points_[jumpIndex]
            }
            prevSeqID = points_[i].seq;
        }

        // Find point in trajectory being closest to input position
        TrajectoryPoint pos(-1, position[0], position[1]);
        double smallestDistance = std::numeric_limits<double>::infinity();
        int smallestDistanceIndex = 0;
        for (int i = 0; i < points_.size(); i++) {
            double distance = points_[i].EuclideanDistance(pos);
            if (distance < smallestDistance) {
                smallestDistanceIndex = i;
                smallestDistance = distance;
            }
        }

        // Add this closest index to the jump index list, such that we can sort this list and find a continuous sequence including the closest point
        jumpIndex.push_back(smallestDistanceIndex);
        std::sort(jumpIndex.begin(), jumpIndex.end());

        // Now find the closest point index in the sorted sequence and extract start and end index of the resulting trajectory
        int startIndex = 0, endIndex = points_.size()-1;
        for (int i = 0; i < jumpIndex.size(); i++) {
            if (jumpIndex[i] == smallestDistanceIndex) {
                if (i > 0)
                    startIndex = jumpIndex[i] - 2; // only include trajectory points from input position and onwards (however include 2 "old" points from back in time on the path)
                else
                    startIndex = jumpIndex[i] - 2;

                if (i < (jumpIndex.size()-1))
                    endIndex = jumpIndex[i+1] - 1; // minus 1 because the jump index position marks the index AFTER the jump
                else
                    endIndex = jumpIndex[i] - 1;
            }
        }

        if (startIndex < 0) startIndex = 0;
        if (endIndex >= (points_.size()-1) && includesGoal()) { // if the trajectory includes goal, then the index will likely be equal to the size of the trajectory (end item)
            endIndex = points_.size()-1;
        }

        if (endIndex < (points_.size()-1) || (endIndex == (points_.size()-1) && includesGoal())) {
            // Extract the continuous sequence trajectory
            continuousSequenceTrajectory.clear();
            for (int i = startIndex; i <= endIndex; i++) {
                continuousSequenceTrajectory.AddPoint(points_[i]);
            }
        }
        else { // endIndex equal to the last point but goal is not set, hence this is likely due to looping
            // Extract the continuous sequence trajectory from closest point to end item of trajectory
            endIndex = points_.size()-1;
            continuousSequenceTrajectory.clear();
            for (int i = startIndex; i <= endIndex; i++) {
                continuousSequenceTrajectory.AddPoint(points_[i]);
            }
            int endSeqID = points_[endIndex].seq;
            // Add points from the beginning until first jump
            for (int i = jumpIndex[0]; i < jumpIndex[1]; i++) {
                TrajectoryPoint pTmp = points_[i];
                pTmp.seq += endSeqID + 1; // make the ending sequence part of the trajectory have greater sequence ID's such that the whole trajectory sequence becomes a series of continuous ID's
                continuousSequenceTrajectory.AddPoint(pTmp);
            }
        }
    }

    TrajectoryPoint Trajectory::FindClosestPoint(Eigen::Vector2d position)
    {
        // Find point in trajectory being closest to input position
        TrajectoryPoint pos(-1, position[0], position[1]);
        double smallestDistance = std::numeric_limits<double>::infinity();
        int smallestDistanceIndex = 0;

        if (points_.size() == 0) return pos;

        for (int i = 0; i < points_.size(); i++) {
            double distance = points_[i].EuclideanDistance(pos);
            if (distance < smallestDistance) {
                smallestDistanceIndex = i;
                smallestDistance = distance;
            }
        }

        return points_.at(smallestDistanceIndex);
    }
}
