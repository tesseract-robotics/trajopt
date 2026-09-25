# trajopt_ifopt

This package provides robotics costs and constraints, built on an in-tree core derived from [ifopt](https://github.com/ethz-adrl/ifopt) (`trajopt_ifopt/core`), and solved by `trajopt_sqp`. Each term is written as a constraint that can then be converted into a cost.

## Why Weights Usually Apply to Slack Penalties (Not Constraint Rows)

A soft constraint keeps its **constraint function** in natural units and allows controlled violation through a **slack** `s >= 0`:

- Original constraint (example): `g(x) <= 0`
- Softened with slack: `g(x) <= s`, `s >= 0` (an equality needs two slacks)
- Penalize `s` in the objective: `w * s` for a hinge or absolute cost, `mu * w * s` for a constraint (ℓ1 penalty)

The weight `w` scales the **slack penalty**, not the **constraint value** and **Jacobian** (the QP row).

### Key reasons

#### 1) Clean separation of “constraint definition” vs “priority”
`g(x)` represents geometry or physics (meters, radians); `w` represents how much violating it matters. Keeping `g(x)` unscaled lets you tune priorities without changing what the constraint means.

#### 2) Conditioning depends on the QP solver
The two placements give the same penalty but different QPs. Interior-point and active-set solvers (the original TrajOpt used Gurobi) are much less sensitive to row scaling than first-order methods. ADMM solvers such as OSQP are sensitive to it:
- OSQP's Ruiz equilibration rescales the constraint rows and the NLP variables, but not a slack: the slack's column holds only its own row and its `s >= 0` bound, so the formulation fixes its scale.
- With the weight on the slack, a row's coupling in OSQP is set by its Jacobian, independent of its weight. With the weight in the row, as in `trajopt_sco`, heavier rows are coupled more strongly.
- ADMM tends to converge faster when the rows active at the QP solution are strongly coupled and the inactive ones weakly, so on OSQP this placement can cost iterations compared with `trajopt_sco`.

`trajopt_sqp::PIQPSolver`, an interior-point solver, is insensitive to this scaling: its iteration count per QP stays nearly constant across constraint weights.

#### 3) Slack variables remain interpretable
With `g(x)` in natural units, the slack is the amount of violation in those units. Scaling rows by `w` puts slacks in weighted units.

#### 4) Weights can change without changing the constraint
The constraint and its Jacobian do not depend on the weight, so a weight can follow the iterate (collision coefficients change with the contact pairs found). A weight of exactly `0` disables a row cleanly instead of leaving an all-zero row.

#### 5) Equivalent in theory, not in practice
For `w >= 0` both placements give the same ℓ1 penalty (`w * |g|⁺ = |w * g|⁺`) and the same merit. Two things differ: the QP the solver sees (reason 2), and the tolerance. Here `cnt_tolerance` applies to violations in the constraint's own units; `trajopt_sco` applies it to weighted violations, so a small weight loosens it there.

### Practical rule of thumb
- **Scale constraints/Jacobians** only to normalize units or improve conditioning (e.g., meters vs millimeters).
- **Use weights** on the **slack penalty** to express priority.

## Merit Weighting in trajopt_sqp

`TrustRegionSQPSolver` is an ℓ1-penalty SQP. For row $i$, let $c_i(x)$ be its value, $[l_i, u_i]$ its bounds and $w_i$ its weight from `getCoefficients()`. Its violation is

$$v_i(x) = \max(l_i - c_i(x),\, 0) + \max(c_i(x) - u_i,\, 0)$$

which is $\lvert c_i(x) - l_i\rvert$ for an equality row and a hinge for a one-sided one. A squared cost row has target $t_i = l_i = u_i$. With $\mu_s$ the merit coefficient of constraint set $s$, the merit is

$$\phi(x) = \sum_{\text{squared}} w_i \big(c_i(x) - t_i\big)^2 + \sum_{\text{hinge, abs}} w_i v_i(x) + \sum_s \mu_s \sum_{i \in s} w_i v_i(x)$$

The QP model $m$ is the same expression with each row linearized at the iterate $x_k$; slacks carry the $v_i$ terms at cost $\mu_s w_i$ ($w_i$ for costs). The trust-region ratio $\rho = (\phi(x_k) - \phi(x^+)) / (\phi(x_k) - m(x^+))$ compares merit and model, so both must use the same weights. `TrajOptQPProblem` re-reads the weights at every `convexify()`, so the model stays exact at $x_k$ when weights follow the iterate (collision coefficients do).

Violations are reported per constraint set in two forms (`ConstraintViolations`):
- `weighted` sums $w_i v_i$ and feeds the merit.
- `raw` sums $v_i$, in the rows' own units, and is compared with `cnt_tolerance`. The solve is feasible once every set's `raw` is below it; otherwise the $\mu_s$ of each set above it is multiplied by `merit_coeff_increase_ratio`, or every $\mu_s$ if `inflate_constraints_individually` is off. Being a sum, it is not a per-row tolerance; `trajopt_sco` applies the same test to its weighted sums.

A row whose weight is exactly `0` is disabled: the QP does not penalize it, and it is left out of both sums.

`IfoptQPProblem` applies no per-row weights to constraints, in the QP or the merit, so its two forms are equal.

## Trust-Box Construction Near Variable Bounds

The trust region in `TrajOptQPProblem` is an $L_\infty$ box on the NLP step, $|p_i| \le \Delta_i$, which the QP sees as bounds on $x_i + p_i$ intersected with the variable bounds $[l_i, u_i]$. Two cases need care:

1. $x_i$ is at or near a bound, so $[x_i - \Delta_i, x_i + \Delta_i]$ extends past it.
2. $x_i$ is outside $[l_i, u_i]$, after a step rejection, a bad warm start, or a failed solve.

`TrajOptQPProblem::Implementation::updateNLPVariableBounds` handles both with one rule: **clamp $x_i$ into $[l_i, u_i]$, then shrink the box to fit within the bounds.**

$$x_i^{\text{eff}} \;=\; \mathrm{clamp}(x_i,\, l_i,\, u_i), \qquad \text{box}_i \;=\; [\,\max(x_i^{\text{eff}} - \Delta_i,\, l_i),\; \min(x_i^{\text{eff}} + \Delta_i,\, u_i)\,].$$

For $x_i$ strictly inside its bounds the clamp is a no-op and $\|p\|_\infty \le \Delta$ holds, which the ratio $\rho$ relies on: it is only meaningful for steps within the radius the model was built for. Near a bound the box width shrinks monotonically:

| $x$ position relative to upper bound $u$ | Box width |
|---|---|
| $x \le u - \Delta$ | $2\Delta$ (centered) |
| $u - \Delta < x < u$ | $\Delta + (u - x)$ (linear ramp from $2\Delta$ down to $\Delta$) |
| $x = u$ | $\Delta$ |
| $x > u$ | $\Delta$ (constant; $x^{\text{eff}} = u$) |

Past the bound $\|p\|_\infty \le \Delta$ cannot hold, since any step back through the bound has magnitude $\ge |x - u|$, but the box stays non-empty, so the QP solver does not error out.

## Currently Supported Constraints
* Joint Position
* Joint Velocity
* Joint Acceleration
* Joint Jerk
* Cartesian Position (FK)
* Cartesian Line
* Inverse Kinematics
* Collision, in fixed-size and dynamic-size forms, with these evaluators:
  * single timestep
  * longest valid segment, discrete
  * longest valid segment, continuous
* Numerical-Jacobian variants of the discrete and continuous collision constraints

### Adding New Constraints
* Fill in only your own block of the Jacobian; its placement in the full Jacobian is handled for you.
* `getCoefficients()` returns exactly one finite, non-negative weight per row, in row order; `0` disables the row. Validate weights where they are set, as the in-tree constraints do.

## Currently Supported Costs
Any constraint set can be used as a cost:

* `TrajOptQPProblem::addCostSet` takes a `CostPenaltyType`: squared, absolute, or hinge.
* The `SquaredCost` and `AbsoluteCost` wrappers turn a constraint set into a `CostTerm` for a `trajopt_ifopt::Problem`.

## Solver
`trajopt_sqp` (in `trajopt_optimizers`) solves these problems with OSQP through [OsqpEigen](https://github.com/gbionics/osqp-eigen/tree/master/include/OsqpEigen), or with [PIQP](https://github.com/PREDICT-EPFL/piqp) through `PIQPSolver` when PIQP is found at build time. It is the only NLP solver: the ifopt dependency, and with it the IPOPT and SNOPT interfaces, was removed.

## TODO

- [ ] Collision plotting callback (`CollisionPlottingCallback::plot` is a stub)

### Additional Solvers

- [ ] [IPOPT](https://github.com/coin-or/Ipopt)
- [ ] [SNOPT](http://www.sbsi-sol-optimize.com/asp/sol_product_snopt.htm)
- [ ] [OptimLib](https://github.com/kthohr/optim)
- [ ] Gurobi QP interface for `trajopt_sqp`
- [ ] [Pagmo2](https://github.com/esa/pagmo2) - "A C++ scientific library for massively parallel optimization"
- [ ] [NOMAD](https://sourceforge.net/projects/nomad-bb-opt/) - LGPL Derivative free
- [ ] [Others](http://plato.asu.edu/sub/nlores.html)

### Core Improvements
- [ ] Add Hessians to the core (upstream request: [ethz-adrl/ifopt#41](https://github.com/ethz-adrl/ifopt/issues/41))
- [ ] Add caching to functions like `Problem::evaluateCostFunction`
