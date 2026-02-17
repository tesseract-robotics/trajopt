/******************************************************************************
Copyright (c) 2017, Alexander W Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <trajopt_ifopt/core/problem.h>
#include <iomanip>
#include <iostream>

namespace trajopt_ifopt
{
Problem::Problem(Variables::Ptr variables)
  : variables_(std::move(variables))
  , constraints_("constraint-sets", variables_->getRows(), Differentiable::Mode::kStackRows, false)
  , costs_("cost-terms", variables_->getRows(), Differentiable::Mode::kStackRows, false)
{
}

void Problem::addConstraintSet(ConstraintSet::Ptr constraint_set)
{
  constraint_set->linkWithVariables(variables_);
  constraints_.addComponent(std::move(constraint_set));
}

void Problem::addCostSet(CostTerm::Ptr cost_set)
{
  cost_set->linkWithVariables(variables_);
  costs_.addComponent(std::move(cost_set));
}

int Problem::getNumberOfOptimizationVariables() const { return variables_->getRows(); }

std::vector<Bounds> Problem::getBoundsOnOptimizationVariables() const { return variables_->getBounds(); }

Eigen::VectorXd Problem::getVariableValues() const { return variables_->getValues(); }

void Problem::setVariables(const double* x)
{
  variables_->setVariables(convertToEigen(x));

  constraints_.update();
  costs_.update();
}

double Problem::evaluateCostFunction()
{
  Eigen::VectorXd g = Eigen::VectorXd::Zero(1);
  if (hasCostTerms())
    g = costs_.getValues();

  return g(0);
}

Eigen::VectorXd Problem::evaluateCostFunctionGradient(bool use_finite_difference_approximation, double epsilon)
{
  int n = getNumberOfOptimizationVariables();
  Jacobian jac = Jacobian(1, n);
  Eigen::VectorXd x = getVariableValues();
  if (hasCostTerms())
  {
    if (use_finite_difference_approximation)
    {
      double step_size = epsilon;

      // calculate forward difference by disturbing each optimization variable
      double g = evaluateCostFunction();
      std::vector<double> x_new(x.data(), x.data() + x.size());
      for (std::size_t i = 0; i < n; ++i)
      {
        x_new[i] += step_size;  // disturb
        setVariables(x_new.data());
        double g_new = evaluateCostFunction();
        jac.coeffRef(0, static_cast<Eigen::Index>(i)) = (g_new - g) / step_size;
        x_new[i] = x[static_cast<Eigen::Index>(i)];  // reset for next iteration
      }
      setVariables(x.data());
    }
    else
    {
      jac = costs_.getJacobian();
    }
  }

  return jac.row(0).transpose();
}

std::vector<Bounds> Problem::getBoundsOnConstraints() const { return constraints_.getBounds(); }

int Problem::getNumberOfConstraints() const { return static_cast<int>(getBoundsOnConstraints().size()); }

Eigen::VectorXd Problem::evaluateConstraints() { return constraints_.getValues(); }

bool Problem::hasCostTerms() const { return costs_.getRows() > 0; }

void Problem::evalNonzerosOfJacobian(double* values) const
{
  Jacobian jac = getJacobianOfConstraints();

  jac.makeCompressed();  // so the valuePtr() is dense and accurate
  std::copy(jac.valuePtr(), jac.valuePtr() + jac.nonZeros(), values);
}

Jacobian Problem::getJacobianOfConstraints() const { return constraints_.getJacobian(); }

Jacobian Problem::getJacobianOfCosts() const { return costs_.getJacobian(); }

void Problem::saveCurrent() { x_prev.push_back(variables_->getValues()); }

Variables::Ptr Problem::getOptVariables() const { return variables_; }

void Problem::setOptVariables(int iter) { variables_->setVariables(x_prev.at(static_cast<std::size_t>(iter))); }

void Problem::setOptVariablesFinal()
{
  variables_->setVariables(x_prev.at(static_cast<std::size_t>(getIterationCount() - 1)));
}

void Problem::printCurrent() const
{
  using namespace std;
  cout << "\n"
       << "************************************************************\n"
       << "    IFOPT - Interface to Nonlinear Optimizers (v2.0)\n"
       << "                \u00a9 Alexander W. Winkler\n"
       << "           https://github.com/ethz-adrl/ifopt\n"
       << "************************************************************"
       << "\n"
       << "Legend:\n"
       << "c - number of variables, constraints or cost terms\n"
       << "i - indices of this set in overall problem\n"
       << "v - number of [violated variable- or constraint-bounds] or [cost "
          "term value]"
       << "\n\n"
       << std::right << std::setw(33) << "" << std::setw(5) << "c  " << std::setw(16) << "i    " << std::setw(11)
       << "v " << std::left << "\n";

  int idx = 0;
  variables_->print(idx);
  idx = 0;
  constraints_.print(idx);
  idx = 0;
  costs_.print(idx);
};

Eigen::VectorXd Problem::convertToEigen(const double* x) const
{
  return Eigen::Map<const Eigen::VectorXd>(x, getNumberOfOptimizationVariables());
}

}  // namespace trajopt_ifopt
