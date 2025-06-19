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

#ifndef LINE_EXTRACTOR_H
#define LINE_EXTRACTOR_H
#include <cmath>
#include <vector>
#include <deque>
#include <boost/array.hpp>
#include <Eigen/Dense>

#include "staircase_perception/utils/stair_utilities.hpp"
#include "staircase_perception/utils/line_extraction/line.hpp"

class LineExtractor{
    public:

    //Constructor and Destructor
    LineExtractor();
    ~LineExtractor();

    //Main Extract Line function
    void extractLines(std::vector<Line>&);
    void extractLines(const std::shared_ptr<std::deque<stair_utility::DetectedLine>>& );

    //Function to set Data
    void setPrecomputedCache(const std::vector<float>&, const std::vector<float>&,
                     const std::vector<float>&, const std::vector<unsigned int>&);
    void setRangeData(const std::vector<float>& ranges, const std::vector<float>& xs, const std::vector<float>& ys, const std::vector<float>& zs);
    
    // Parameter setting
    void setBearingVariance(double);
    void setRangeVariance(double);
    void setZVariance(double);
    void setLeastSqAngleThresh(double);
    void setLeastSqRadiusThresh(double);
    void setMaxLineGap(double);
    void setMinLineLength(double);
    void setMinRange(double);
    void setMaxRange(double);
    void setMinLinePoints(unsigned int);
    void setMinSplitDist(double);
    void setOutlierDist(double);

    private:
        // Data structures
        stair_utility::PrecomputedCache c_data_;
        stair_utility::RangeData r_data_;
        stair_utility::LineExtractorParams params_;

        // Indices after filtering
        std::vector<unsigned int> filtered_indices_;
        
        // Line data
        std::vector<Line> lines_;
        
        // Methods
        double chiSquared(const Eigen::Vector2d&, const Eigen::Matrix2d&,
                    const Eigen::Matrix2d&);
        
        double distBetweenPoints(unsigned int index_1, unsigned int index_2);
        
        void   filterCloseAndFarPoints();
        void   filterOutlierPoints();

        void   filterLines();
        void   mergeLines();
        void   split(const std::vector<unsigned int>&);
};

#endif