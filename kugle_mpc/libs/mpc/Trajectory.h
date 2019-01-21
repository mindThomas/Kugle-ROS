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
 
#ifndef LIBRARY_TAJECTORY_H
#define LIBRARY_TAJECTORY_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <cstdint>

#include <vector>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

/* For visualization/plotting only */
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace MPC
{

	class TrajectoryPoint
	{
	    public:
            int seq; // sequence number
            Eigen::Vector2d point;
            double heading;
            double velocity;

            TrajectoryPoint() : seq(-1), point(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()), heading(std::numeric_limits<double>::infinity()), velocity(std::numeric_limits<double>::infinity()) {} // uninitialized point
            TrajectoryPoint(int seq_, double _x, double _y, double _heading = -1, double _velocity = -1) : seq(seq_), point(_x, _y), heading(_heading), velocity(_velocity) {}
            double EuclideanDistance(const TrajectoryPoint& other);

            /* Operator used for sorting */
            bool operator < (const TrajectoryPoint& p) const
            {
                return (seq < p.seq);
            }
	};

	class Trajectory
	{
		public:
            Trajectory();
            Trajectory(std::vector<TrajectoryPoint>& points);
            Trajectory(TrajectoryPoint points[], int num_points);
			~Trajectory();

            Trajectory& operator=(const Trajectory& other);

			void AddPoint(TrajectoryPoint& point);
            void AddPoint(Eigen::Vector2d point, double heading = -1, double velocity = -1);
			void AddPoint(double x, double y, double heading = -1, double velocity = -1);
            void AddPoint(int seq, double x, double y, double heading = -1, double velocity = -1);
            void clear();
			size_t size();

            void rotate(Trajectory& rotatedTrajectory, double angle);
            void rotate(double angle);
            void move(double x, double y);
            void move(Eigen::Vector2d offset);
            void move(Trajectory& movedTrajectory, double x, double y);
            void scale(double scale_factor);
            void PositionExtract(Trajectory& extractedTrajectory, Eigen::Vector2d p_min, Eigen::Vector2d p_max);
            void PositionExtract(Trajectory& extractedTrajectory, double x_min, double y_min, double x_max, double y_max);
            void SequenceExtract(Trajectory& extractedTrajectory, int seq_min, int seq_max = std::numeric_limits<int>::infinity());
            void DistanceExtract(Trajectory& extractedTrajectory, double distance_max, double distance_min = 0);
            void ApproxCurveLengthExtract(Trajectory& extractedTrajectory, double curve_length_max);
			void WindowExtract(Trajectory& extractedTrajectory, Eigen::Vector2d center, double heading, double width, double height, Eigen::Vector2d offset = Eigen::Vector2d(0,0));
            void print();
            bool find(int seq, TrajectoryPoint& foundPoint);
			bool find(int seq);
            void sort();
            double distance();
            int GetLastSequenceID();
            std::vector<double> GetDistanceList();
			std::vector<double> GetX();
			std::vector<double> GetY();

            void FixSequenceOrder(Trajectory& correctedTrajectory, int startSeqID, int endSeqID);
            void ExtractContinuousClosestSequence(Trajectory& continuousSequenceTrajectory, Eigen::Vector2d position = Eigen::Vector2d(0,0));

            void plot(bool drawXup = false, bool plotText = true, double x_min = -2.5, double y_min = -2.5, double x_max = 2.5, double y_max = 2.5);
            void plot(cv::Mat& image, cv::Scalar color, bool drawXup = false, bool plotText = true, double x_min = -2.5, double y_min = -2.5, double x_max = 2.5, double y_max = 2.5);

	    public:
	        static Trajectory GenerateTestTrajectory(void);

		private:
			std::vector<TrajectoryPoint> points_;
			int lastSeq_;
			bool sorted_;
	};

}
	
	
#endif
