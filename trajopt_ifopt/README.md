# trajopt_ifopt

This package provides robotics costs and constraints for use with the ifopt optimization framework. Each term is written as a constraint that can then be converted into a cost.

## Why Weights Usually Apply to Slack Penalties (Not Constraint Rows)

In the trajopt_ifopt “soft constraint via slack variables” pattern, we keep the **constraint function** in its natural units and introduce a **slack** `s >= 0` to allow controlled violation:

- Original constraint (example): `g(x) <= 0`
- Softened with slack: `g(x) <= s`, `s >= 0`
- Add a penalty on `s` to the objective (hinge / L1 / L2 style): `min w * phi(s)`

With this structure, it’s typically preferred to apply the coefficient `w` to the **slack penalty** (objective side) rather than scaling the **constraint value** and **Jacobian** (constraint row).

### Key reasons

#### 1) Clean separation of “constraint definition” vs “priority”
- `g(x)` represents geometry/physics (meters, radians, etc.).
- `w` represents how much you care about violating it.

Keeping `g(x)` unscaled makes it easier to reason about correctness and tune priorities without changing the constraint’s numerical meaning.

#### 2) Better numerical conditioning and solver behavior
Scaling the constraint residual/Jacobian (i.e., scaling QP row(s)) changes the conditioning of the constraint matrix and the primal/dual balance in the KKT system. This can lead to:
- worse conditioning,
- unstable step sizes,
- slower or less reliable convergence.

Weighting the slack penalty primarily changes objective terms, which many SQP/QP solvers handle more robustly than heavily scaled constraint rows.

#### 3) Slack variables remain interpretable
If you keep `g(x)` in natural units, then the slack `s` directly represents “amount of violation” in those same units. This makes it easier to:
- interpret results,
- set bounds on `s`,
- initialize slack values consistently across constraints.

If you scale constraint rows by `w`, slack effectively becomes “weighted units,” which is harder to interpret and tune.

#### 4) Avoids fighting feasibility/scaling logic inside solvers
Many solvers apply their own internal scaling, feasibility handling, and merit function logic. Manually scaling constraint rows for “importance” can interact poorly with these mechanisms. Applying weights in the slack penalty tends to be more predictable.

#### 5) Equivalent in theory, not in practice
While you can sometimes rewrite formulations so “scale constraints” and “scale penalties” appear similar on paper, real SQP/QP implementations include regularization, trust regions, merit functions, and adaptive updates that make these choices behave differently numerically.

### Practical rule of thumb
- **Scale constraints/Jacobians** only to normalize units / improve conditioning (e.g., meters vs millimeters).
- **Use coefficients/weights** on the **slack penalty** to represent priority/importance of the soft constraint.


## Currently Supported Constraints
* Joint Position
* Joint Velocity
* Joint Jerk
* Cartesian Position(FK)
* Inverse Kinematics
* Collision
  * Fixed Size
    * single timestep evaluator
    * longest valid segment discrete evaluator
    * longest valid segment continuous evaluator
  * Dynamic Size
    * single timestep evaluator
    * longest valid segment discrete evaluator
    * longest valid segment continuous evaluator
    
### Adding New Constraints
*  The process of filling out the jacobian can be confusing because you are only responsible for filling out your portion and do not need to worry about it position in the full jacobian matrix.


## Currently Supported Costs
These costs convert any constraint into a cost

*  Squared Cost
*  Absolute Cost


## Trajopt_optimizers
Additionally, the SQP solver has been rewritten. Currently it provides the SQP routine with interfaces to these QP solvers
*  [OSQPEigen](https://github.com/robotology/osqp-eigen/tree/master/include/OsqpEigen)

Therefore the solvers that will work in this new framework are
*  [IPOPT](https://github.com/coin-or/Ipopt)
*  [SNOPT](http://www.sbsi-sol-optimize.com/asp/sol_product_snopt.htm)
*  Trajopt_sqp

## TODO

- [ ] Collision - Add other collision evaluators
- [ ] Collision Callback - Collision is the only one that is missing
- [ ] Print debug info - trajopt_sco prints a bunch of debugging information that developers have become accustomed to. Some of this has been added to trajopt_sqp, but some like the cost associated with individual convex costs, will require some work to implement


### Additional Solvers

- [ ] [OptimLib](https://github.com/kthohr/optim)
- [ ] Gurobi interface - We should try SNOPT first, but an interface to Gurobi might be nice.
- [ ] [Pagmo2](https://github.com/esa/pagmo2) - "A C++ scientific library for massively parallel optimization"
- [ ] [NOMAD](https://sourceforge.net/projects/nomad-bb-opt/) - LGPL Derivative free
- [ ] [Others](http://plato.asu.edu/sub/nlores.html)


### IFOPT Improvements
- [ ] [Add Hessian to IFOPT](https://github.com/ethz-adrl/ifopt/issues/41)
- [ ] Add caching to functions like Problem::EvaluateCostFunction
