# trajopt_ros_ifopt

This package provides robotics costs and constraints for use with the ifopt optimization framework. Each term is written as a constraint that can then be converted into a cost.

## Currently Supported Constraints
*  Joint Position
*  Joint Velocity
*  Cartesian Position(FK)
*  Inverse Kinematics
*  Collision (single timestep evaluator only)

## Currently Supported Costs
These costs convert any constraint into a cost

*  Squared Cost


## Trajopt_optimizers
Additionally, the SQP solver has been rewritten. Currently it provides the SQP routine with interfaces to these QP solvers
*  [OSQPEigen](https://github.com/robotology/osqp-eigen/tree/master/include/OsqpEigen)

Therefore the solvers that will work in this new framework are
*  [IPOPT](https://github.com/coin-or/Ipopt)
*  [SNOPT](http://www.sbsi-sol-optimize.com/asp/sol_product_snopt.htm)
*  Trajopt_sqp

## TODO

- [ ] Collision - Add other collision evaluators
- [ ] Callbacks - Callbacks for plotting need to be developed (here and in Tesseract)
- [ ] Print debug info - trajopt_sco prints a bunch of debugging information that developers have become accustomed to. We need to replicate that here.


### Additional Solvers

- [ ] [OptimLib](https://github.com/kthohr/optim)
- [ ] Gurobi interface - We should try SNOPT first, but an interface to Gurobi might be nice.
- [ ] [Pagmo2](https://github.com/esa/pagmo2)


### IFOPT Improvements
- [ ] [Add Hessian to IFOPT](https://github.com/ethz-adrl/ifopt/issues/41)
- [ ] Add caching to functions like Problem::EvaluateCostFunction
