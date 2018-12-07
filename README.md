# trajopt_ros
Integration of TrajOpt into ROS

## Solvers support
`trajopt_ros` implements sequential convex optimization to solve the motion planning problem.
It implements a penalty method to optimize for joint velocities while satisfying a set of constraints.
Internally, it makes use of convex solvers that are able to solve linearly constrained quadratic problems.
At the moment, the following solvers are supported:
- `BPMPD` (interior point method, free for non-commercial use only)
- `Gurobi` (simplex and interior point/parallel barrier, license required)
- `OSQP` (ADMM, BSD2 license)
- `qpOASES` (active set, LGPL 2.1 license)

While the `BPMPD` library is bundled in the distribution, `Gurobi`, `OSQP` and `qpOASES` need to be installed in the system.
To compile with `Gurobi` support, a `GUROBI_HOME` variable needs to be defined.
Once `trajopt_ros` is compiled with support for a specific solver, you can select it by properly setting the `TRAJOPT_CONVEX_SOLVER` environment variable. Possible values are `GUROBI`, `BPMPD`, `OSQP`, `QPOASES`, `AUTO_SOLVER`.
The selection to `AUTO_SOLVER` is the default and automatically picks the best between the available solvers.

## Build Branch Sphinx Documentation

```
cd gh_pages
sphinx-build . output
```
Now open gh_pages/output/index.rst and remove *output* directory before commiting changes.
