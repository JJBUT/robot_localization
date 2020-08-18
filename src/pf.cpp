/*
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "robot_localization/pf.h"
#include "robot_localization/filter_common.h"

#include <XmlRpcException.h>

#include <sstream>
#include <iomanip>
#include <limits>

#include <Eigen/Cholesky>

#include <iostream>
#include <vector>

#include <assert.h>

namespace RobotLocalization
{
  Pf::Pf(std::vector<double> args) :
    FilterBase(),  // Must initialize filter base!
    uncorrected_(true)
  {
    assert(args.size() == 2);

    int np = args[0];
    int np_min = args[1];

    particles_.resize(np, Eigen::VectorXd(STATE_SIZE));

  
    particleWeights_.resize(np);
   
    
    for (size_t i = 0; i < np; ++i)
    {
      particleWeights_[i] = 1.0/np_min;
      particles_[i].setZero();
    }
  }

  

  

  Pf::~Pf()
  {
  }

  void Pf::correct(const Measurement &measurement)
  {
    FB_DEBUG("---------------------- Ukf::correct ----------------------\n" <<
             "State is:\n" << state_ <<
             "\nMeasurement is:\n" << measurement.measurement_ <<
             "\nMeasurement covariance is:\n" << measurement.covariance_ << "\n");


    FB_DEBUG("\nCorrected full state is:\n" << state_ <<
             "\n\n---------------------- /Ukf::correct ----------------------\n");
  }

  void Pf::predict(const double referenceTime, const double delta)
  {
    FB_DEBUG("---------------------- Ukf::predict ----------------------\n" <<
             "delta is " << delta <<
             "\nstate is " << state_ << "\n");


    FB_DEBUG("Predicted state is:\n" << state_ <<
             "\n\n--------------------- /Ukf::predict ----------------------\n");
  }

}  // namespace RobotLocalization
