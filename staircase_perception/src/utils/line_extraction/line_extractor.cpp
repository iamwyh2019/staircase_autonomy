/*
 * Copyright (c) 2014, Marc Gallant
 * All rights reserved.
 *
 * This file has been modified from its original version.
 * Copyright (c) 2025, Prasanna Sriganesh
 *
 * The original portions of this file are licensed under the BSD 3-Clause
 * license, the text of which is reproduced below.
 *
 * ----------------------------------------------------------------
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * * Neither the name of the {organization} nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include "staircase_perception/utils/line_extraction/line_extractor.hpp"
#include <algorithm>
#include <Eigen/Dense>
#include <iostream>

// Constructor / destructor
LineExtractor::LineExtractor(){}

LineExtractor::~LineExtractor(){}

// Main Line Extractor function
void LineExtractor::extractLines(std::vector<Line>& lines) 
{
  // Resets
  filtered_indices_ = c_data_.indices;
  lines_.clear();

  // Filter indices
  filterCloseAndFarPoints();
  filterOutlierPoints();

  // Return no lines if not enough points left
  if (filtered_indices_.size() <= std::max(params_.min_line_points, static_cast<unsigned int>(3)))
  {
    return;
  }

  // Split indices into lines and filter out short and sparse lines
  split(filtered_indices_);

  // Fit each line using least squares and merge colinear lines
  for (std::vector<Line>::iterator it = lines_.begin(); it != lines_.end(); ++it)
  {
    it->leastSqFit();
  }
  
  // If there is more than one line, check if lines should be merged based on the merging criteria
  if (lines_.size() > 1)
  {
    mergeLines();
  }
  
  filterLines();
  lines = lines_;
}

// Line extract functions using shared pointer.
void LineExtractor::extractLines(const std::shared_ptr<std::deque<stair_utility::DetectedLine>>& lines){
  // Resets
  filtered_indices_ = c_data_.indices;
  lines_.clear();

  // Filter indices
  filterCloseAndFarPoints();
  filterOutlierPoints();

  // Return no lines if not enough points left
  if (filtered_indices_.size() <= std::max(params_.min_line_points, static_cast<unsigned int>(3)))
  {
    return;
  }

  // Split indices into lines and filter out short and sparse lines
  split(filtered_indices_);

  // Fit each line using least squares and merge colinear lines
  for (std::vector<Line>::iterator it = lines_.begin(); it != lines_.end(); ++it)
  {
    it->leastSqFit();
  }
  
  // If there is more than one line, check if lines should be merged based on the merging criteria
  if (lines_.size() > 1)
  {
    mergeLines();
  }
  
  filterLines();

  lines->clear();
  for(const Line &l: lines_){
    lines->push_back(stair_utility::DetectedLine(l.getStart(), l.getEnd(), l.getCenter(), l.getYaw(), l.getLineLength(), l.getRadius(), l.getAngle(), l.getCovariance(), params_.z_var));
  }

}


// Data setting
void LineExtractor::setPrecomputedCache(const std::vector<float>& bearings,
                                   const std::vector<float>& cos_bearings,
                                   const std::vector<float>& sin_bearings,
                                   const std::vector<unsigned int>& indices)
{
  c_data_.bearings = bearings;
  c_data_.cos_bearings = cos_bearings;
  c_data_.sin_bearings = sin_bearings;
  c_data_.indices = indices;
}

void LineExtractor::setRangeData(const std::vector<float>& ranges, const std::vector<float>& xs, const std::vector<float>& ys, const std::vector<float>& zs)
{ 
  r_data_.ranges.clear();
  r_data_.ranges = ranges;
  r_data_.xs.clear();
  r_data_.ys.clear();
  r_data_.zs.clear();
  r_data_.xs = xs;
  r_data_.ys = ys;
  r_data_.zs = zs;
}


// Parameter setting
void LineExtractor::setBearingVariance(double value)
{
  params_.bearing_var = value;
}

void LineExtractor::setRangeVariance(double value)
{
  params_.range_var = value;
}

void LineExtractor::setLeastSqAngleThresh(double value)
{
  params_.least_sq_angle_thresh = value;
}

void LineExtractor::setZVariance(double value){
  params_.z_var = value;
}

void LineExtractor::setLeastSqRadiusThresh(double value)
{
  params_.least_sq_radius_thresh = value;
}

void LineExtractor::setMaxLineGap(double value)
{
  params_.max_line_gap = value;
}

void LineExtractor::setMinLineLength(double value)
{
  params_.min_line_length = value;
}

void LineExtractor::setMinRange(double value)
{
  params_.min_range = value;
}

void LineExtractor::setMaxRange(double value)
{
  params_.max_range = value;
}

void LineExtractor::setMinLinePoints(unsigned int value)
{
  params_.min_line_points = value;
}

void LineExtractor::setMinSplitDist(double value)
{
  params_.min_split_dist = value;
}

void LineExtractor::setOutlierDist(double value)
{
  params_.outlier_dist = value;
}


// Utility methods
double LineExtractor::chiSquared(const Eigen::Vector2d &dL, const Eigen::Matrix2d &P_1,
                                  const Eigen::Matrix2d &P_2)
{
  return dL.transpose() * (P_1 + P_2).inverse() * dL;
}

double LineExtractor::distBetweenPoints(unsigned int index_1, unsigned int index_2)
{
  return sqrt(pow(r_data_.xs[index_1] - r_data_.xs[index_2], 2) + 
              pow(r_data_.ys[index_1] - r_data_.ys[index_2], 2));
}


// Filtering points
void LineExtractor::filterCloseAndFarPoints()
{
  std::vector<unsigned int> output;
  for (std::vector<unsigned int>::const_iterator cit = filtered_indices_.begin(); 
       cit != filtered_indices_.end(); ++cit)
  {
    const double& range = r_data_.ranges[*cit];
    if (range >= params_.min_range && range <= params_.max_range)
    {
      output.push_back(*cit);
    }
  }
  filtered_indices_ = output;
}

void LineExtractor::filterOutlierPoints()
{
  // if (filtered_indices_.size() < 3)
  // {
  //   return;
  // }

  std::vector<unsigned int> output;
  unsigned int p_i, p_j, p_k;
  for (std::size_t i = 0; i < filtered_indices_.size(); ++i)
  {

    // Get two closest neighbours

    p_i = filtered_indices_[i];
    if (i == 0) // first point
    {
      p_j = filtered_indices_[i + 1];
      p_k = filtered_indices_[i + 2];
    }
    else if (i == filtered_indices_.size() - 1) // last point
    {   
      p_j = filtered_indices_[i - 1];
      p_k = filtered_indices_[i - 2];
    }
    else // middle points
    {
      p_j = filtered_indices_[i - 1];
      p_k = filtered_indices_[i + 1];
    }

    // Check if point is an outlier

    if (fabs(r_data_.ranges[p_i] - r_data_.ranges[p_j]) > params_.outlier_dist &&
        fabs(r_data_.ranges[p_i] - r_data_.ranges[p_k]) > params_.outlier_dist) 
    {
      // Check if it is close to line connecting its neighbours
      std::vector<unsigned int> line_indices;
      line_indices.push_back(p_j);
      line_indices.push_back(p_k);
      Line line(c_data_, r_data_, params_, line_indices);
      line.endpointFit();
      if (line.distToPoint(p_i) > params_.min_split_dist)
      {
        continue; // point is an outlier
      }
    }

    output.push_back(p_i);
  }

  filtered_indices_ = output;
}


// Filtering and merging lines
void LineExtractor::filterLines()
{
  std::vector<Line> output;
  for (std::vector<Line>::const_iterator cit = lines_.begin(); cit != lines_.end(); ++cit)
  {
    if (cit->length() >= params_.min_line_length && cit->numPoints() >= params_.min_line_points)
    {
      output.push_back(*cit);
    }
  }
  lines_ = output;
}

void LineExtractor::mergeLines()
{
  std::vector<Line> merged_lines;

  for (std::size_t i = 1; i < lines_.size(); ++i)
  {
    // Get L, P_1, P_2 of consecutive lines
    Eigen::Vector2d L_1(lines_[i-1].getRadius(), lines_[i-1].getAngle());
    Eigen::Vector2d L_2(lines_[i].getRadius(), lines_[i].getAngle());
    Eigen::Matrix2d P_1;
    P_1 << lines_[i-1].getCovariance()[0], lines_[i-1].getCovariance()[1],
           lines_[i-1].getCovariance()[2], lines_[i-1].getCovariance()[3];
    Eigen::Matrix2d P_2;
    P_2 << lines_[i].getCovariance()[0], lines_[i].getCovariance()[1],
           lines_[i].getCovariance()[2], lines_[i].getCovariance()[3];

    // Merge lines if chi-squared distance is less than 3
    if (chiSquared(L_1 - L_2, P_1, P_2) < 3)
    {
      // Get merged angle, radius, and covariance
      Eigen::Matrix2d P_m = (P_1.inverse() + P_2.inverse()).inverse();
      Eigen::Vector2d L_m = P_m * (P_1.inverse() * L_1 + P_2.inverse() * L_2);
      // Populate new line with these merged parameters
      std::array<double, 4> cov;
      cov[0] = P_m(0,0);
      cov[1] = P_m(0,1);
      cov[2] = P_m(1,0);
      cov[3] = P_m(1,1);
      std::vector<unsigned int> indices;
      const std::vector<unsigned int> &ind_1 = lines_[i-1].getIndices();
      const std::vector<unsigned int> &ind_2 = lines_[i].getIndices();
      indices.resize(ind_1.size() + ind_2.size());
      indices.insert(indices.end(), ind_1.begin(), ind_1.end());
      indices.insert(indices.end(), ind_2.begin(), ind_2.end());
      Line merged_line(L_m[1], L_m[0], cov, lines_[i-1].getStart(), lines_[i].getEnd(), indices);
      // Project the new endpoints
      merged_line.projectEndpoints();
      lines_[i] = merged_line;
    }
    else
    {
      merged_lines.push_back(lines_[i-1]);
    }

    if (i == lines_.size() - 1)
    {
      merged_lines.push_back(lines_[i]);
    }
  }
  lines_ = merged_lines;
}


// Function for Splitting points into lines
void LineExtractor::split(const std::vector<unsigned int>& indices)
{
  // Don't split if only a single point (only occurs when orphaned by gap)
  if (indices.size() <= 1)
  {
    return;
  }

  Line line(c_data_, r_data_, params_, indices);
  line.endpointFit();
  double dist_max = 0;
  double gap_max = 0;
  double dist, gap;
  int i_max, i_gap;

  // Find the farthest point and largest gap
  for (std::size_t i = 1; i < indices.size() - 1; ++i)
  {
    dist = line.distToPoint(indices[i]);
    if (dist > dist_max)
    {
      dist_max = dist;
      i_max = i;
    }
    gap = distBetweenPoints(indices[i], indices[i+1]);
    if (gap > gap_max)
    {
      gap_max = gap;
      i_gap = i;
    }
  }

  // Check for gaps at endpoints
  double gap_start = distBetweenPoints(indices[0], indices[1]);
  if (gap_start > gap_max)
  {
    gap_max = gap_start;
    i_gap = 1;
  }
  double gap_end = distBetweenPoints(indices.rbegin()[1], indices.rbegin()[0]);
  if (gap_end > gap_max)
  {
    gap_max = gap_end;
    i_gap = indices.size() - 1;
  }

  // Check if line meets requirements or should be split
  if (dist_max < params_.min_split_dist && gap_max < params_.max_line_gap)
  {
    lines_.push_back(line);
  }
  else
  {
    int i_split = dist_max >= params_.min_split_dist ? i_max : i_gap;
    std::vector<unsigned int> first_split(&indices[0], &indices[i_split - 1]);
    std::vector<unsigned int> second_split(&indices[i_split], &indices.back());
    split(first_split);
    split(second_split);
  }

}
