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

#ifndef LINE_H
#define LINE_H

#include <vector>
#include "staircase_perception_standalone/utils/stair_utilities.hpp"

class Line{
    public:
        // Constructor / destructor
        Line(const stair_utility::PrecomputedCache&, const stair_utility::RangeData&, const stair_utility::LineExtractorParams&, std::vector<unsigned int>);

        Line(double angle, double radius, const std::array<double, 4> &covariance,
           const std::array<double, 3> &start, const std::array<double, 3> &end,
           const std::vector<unsigned int> &indices);

        ~Line();
        Line();

        // Get methods for the line parameters
        double                              getAngle() const;
        const std::array<double, 4>&        getCovariance() const;
        const std::array<double, 3>&        getEnd() const;
        const std::vector<unsigned int>&    getIndices() const;
        double                              getRadius() const;
        const std::array<double, 3>&        getStart() const;
        const std::array<double, 3>&        getCenter() const;
        double                              getYaw() const;
        double                              getLineLength() const;


        // Methods for line fitting
        double       distToPoint(unsigned int);
        void         endpointFit();
        void         leastSqFit();
        double       length() const;
        unsigned int numPoints() const;
        void         projectEndpoints();

        bool skip;
    private:

        std::vector<unsigned int> indices_;
        // Data structures
        stair_utility::PrecomputedCache c_data_;
        stair_utility::RangeData r_data_;
        stair_utility::LineExtractorParams params_;
        stair_utility::PointParams p_params_;
        
        // Point variances used for least squares
        std::vector<double> point_scalar_vars_;
        std::vector<std::array<double, 4> > point_covs_;
        double p_rr_;

        // Line parameters
        double angle_;
        double radius_;
        double yaw_angle_;
        double line_length_;
        std::array<double, 3> start_;
        std::array<double, 3> end_;
        std::array<double, 3> center_;
        std::array<double, 4> covariance_;
        
        // Methods
        void    angleFromEndpoints();
        void    angleFromLeastSq();
        double  angleIncrement();
        void    calcCovariance();
        void    calcPointCovariances();
        void    calcPointParameters();
        void    calcPointScalarCovariances();
        void    radiusFromEndpoints();
        void    radiusFromLeastSq();
        void    computeOtherParams();
};

#endif