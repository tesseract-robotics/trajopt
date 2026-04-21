# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Trajectory optimization library using Sequential Convex Optimization (SCO/SQP). Converts non-convex motion planning problems into sequences of convex quadratic subproblems solved iteratively with a trust region method.

## Architecture

### Packages

- **trajopt** — Core optimization. `TrajOptProblem` holds the full formulation. Cost/constraint types: joint position/velocity/acceleration, cartesian pose, collision (discrete and continuous)
- **trajopt_sco** — SQP engine. `BasicTrustRegionSQP` with QP backends: OSQP (default, BSD2), qpOASES (LGPL), GUROBI, BPMPD
- **trajopt_common** — Shared types: `CollisionCoeffData` (per-pair collision coefficients, keyed on `tesseract::common::LinkIdPair`), `TrajOptCollisionConfig`, gradient-result structs
- **trajopt_ifopt** — Alternative IFOPT-based interface (Ipopt, Snopt, etc.)
- **trajopt_optimizers** — Optimizer implementations for the IFOPT variant

### Key details

- **Collision integration**: `CollisionEvaluator` wraps Tesseract's contact managers, converts contact results to affine expressions with Jacobians from contact normals
- **Problem construction**: `TermInfo` factories with `fromJson()`/`hatch()` pattern — terms registered by name, instantiated from JSON or YAML
