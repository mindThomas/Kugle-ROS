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
        return *this;
    }

    void Trajectory::AddPoint(TrajectoryPoint& point)
    {
        points_.push_back(point);
        sorted_ = false;
    }

    void Trajectory::AddPoint(Eigen::Vector2d point, double heading, double velocity)
    {
        points_.push_back(TrajectoryPoint(++lastSeq_, point[0], point[1], heading, velocity));
        sorted_ = false;
    }

    void Trajectory::AddPoint(double x, double y, double heading, double velocity)
    {
        points_.push_back(TrajectoryPoint(++lastSeq_, x, y, heading, velocity));
        sorted_ = false;
    }

    void Trajectory::AddPoint(unsigned int seq, double x, double y, double heading, double velocity)
    {
        points_.push_back(TrajectoryPoint(seq, x, y, heading, velocity));
        lastSeq_ = seq;
        sorted_ = false;
    }

    void Trajectory::clear()
    {
        points_.clear();
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
            rotatedTrajectory.AddPoint(rot*p.point, p.heading, p.velocity);
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
            movedTrajectory.AddPoint(p.point += offset, p.heading, p.velocity);
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

    void Trajectory::SequenceExtract(Trajectory& extractedTrajectory, unsigned int seq_min, unsigned int seq_max)
    {
        extractedTrajectory.clear();
        for (auto& p : points_) {
            if (p.seq >= seq_min && p.seq <= seq_max)
                extractedTrajectory.AddPoint(p);
        }
        extractedTrajectory.sorted_ = false;
    }

    bool Trajectory::find(TrajectoryPoint& foundPoint, unsigned int seq)
    {
        for (auto& p : points_) {
            if (p.seq != -1 && (unsigned int)p.seq == seq)
                foundPoint = p;
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

        double r = 10;

        Eigen::Vector2d p;
        for (unsigned int i = 0; i < (100+50+100+50-1); i++) {
            if (i < 100) { //  straight x=10, y=-50...50
                p << 10, (double(i) - 50);
            } else if (i < 100+50) { // left turn 180 degree with radius, r
                p << (10 - r), 50;
                p += r * Eigen::Vector2d(cos(M_PI / 50 * (double(i) - 100)), sin(M_PI / 50 * (double(i) - 100)));
            } else if (i < 100+50+100) { // straight x=-10, y=50...-50
                p << -10, (50 - (double(i) - 100 - 50));
            } else if (i < 100+50+100+50) { // left turn 180 degree with radius, r
                p << 10 - r, -50;
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

    /*
    Path Trajectory::ApproximatePath()
    {

    }
    */
}
