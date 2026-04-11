# Brief: Port optimizations C and E to trajopt_ifopt

## Context

Perf profiling of `hvr_planning_se` identified allocation hotspots in trajopt's collision evaluators. Optimizations C (clone JointGroup) and E (persistent FK transform caches) were implemented in trajopt but the identical anti-patterns exist in trajopt_ifopt. This brief documents the port.

## Files to modify

- `trajopt_ifopt/include/trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h`
- `trajopt_ifopt/include/trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h`
- `trajopt_ifopt/src/constraints/collision/continuous_collision_evaluators.cpp`
- `trajopt_ifopt/src/constraints/collision/discrete_collision_evaluators.cpp`

## Step 1: Optimization C — Clone JointGroup per evaluator

**Why:** Each evaluator stores `shared_ptr<const JointGroup>` via `manip_(std::move(manip))`. If multiple evaluators share the same JointGroup, they share the internal `KDLStateSolver` mutex and KDL's mutable `Joint::pose()` state — a data race under concurrency and a serialization bottleneck.

**What:** In all three evaluator constructors, change:
```cpp
: manip_(std::move(manip))
```
to:
```cpp
: manip_(std::make_shared<tesseract::kinematics::JointGroup>(*manip))
```

**Locations (3 constructors):**
- `discrete_collision_evaluators.cpp:43` — `SingleTimestepCollisionEvaluator`
- `continuous_collision_evaluators.cpp:46` — `LVSContinuousCollisionEvaluator`
- `continuous_collision_evaluators.cpp:286` — `LVSDiscreteCollisionEvaluator`

The `get_state_fn_` lambdas capture `this` (via `[&]`) so they automatically use the cloned `manip_`. No lambda changes needed.

## Step 2: Optimization E — Persistent FK transform caches

**Why:** Every `calcCollisionsHelper` call clears the TransformMap and repopulates it via `operator[]`, causing per-link hash node alloc/dealloc every call. With the OFKT `insert_or_assign` fix (optimization I) already committed, the maps can be safely reused without clearing.

### 2a: Move transform caches from static thread_local to instance members

**Header changes:**

In `continuous_collision_evaluators.h`, remove the `static thread_local` declarations from the `ContinuousCollisionEvaluator` base class (lines 120-121):
```cpp
// REMOVE:
static thread_local tesseract::common::TransformMap transforms_cache0;  // NOLINT
static thread_local tesseract::common::TransformMap transforms_cache1;  // NOLINT
```

Replace with regular (non-static, non-thread_local) instance members in the `protected` section:
```cpp
tesseract::common::TransformMap transforms_cache0_;
tesseract::common::TransformMap transforms_cache1_;
```

In `discrete_collision_evaluators.h`, add an instance member to `SingleTimestepCollisionEvaluator` (private section, after `contact_manager_`):
```cpp
tesseract::common::TransformMap transforms_cache_;
```

### 2b: Remove `.clear()` calls and update variable names

**`discrete_collision_evaluators.cpp` — `calcCollisionsHelper` (lines 150-188):**

Remove the function-local `TRAJOPT_THREAD_LOCAL TransformMap state; state.clear();` (lines 153-154). Use the instance member `transforms_cache_` instead of `state` throughout the function.

Before:
```cpp
TRAJOPT_THREAD_LOCAL tesseract::common::TransformMap state;
state.clear();
get_state_fn_(state, dof_vals);
for (const auto& link_name : diff_active_link_names_)
    contact_manager_->setCollisionObjectsTransform(link_name, state[link_name]);
for (const auto& link_name : manip_active_link_names_)
    contact_manager_->setCollisionObjectsTransform(link_name, state[link_name]);
```

After:
```cpp
get_state_fn_(transforms_cache_, dof_vals);
for (const auto& link_name : diff_active_link_names_)
    contact_manager_->setCollisionObjectsTransform(link_name, transforms_cache_[link_name]);
for (const auto& link_name : manip_active_link_names_)
    contact_manager_->setCollisionObjectsTransform(link_name, transforms_cache_[link_name]);
```

**`continuous_collision_evaluators.cpp`:**

Remove the `thread_local` static definitions (lines 38-39):
```cpp
// REMOVE:
thread_local tesseract::common::TransformMap ContinuousCollisionEvaluator::transforms_cache0;
thread_local tesseract::common::TransformMap ContinuousCollisionEvaluator::transforms_cache1;
```

In `LVSContinuousCollisionEvaluator::calcCollisionsHelper` (lines 176-177), remove:
```cpp
transforms_cache0.clear();
transforms_cache1.clear();
```

And rename all `transforms_cache0` → `transforms_cache0_`, `transforms_cache1` → `transforms_cache1_` in the function body (lines 176-251).

In `LVSDiscreteCollisionEvaluator::calcCollisionsHelper` (line 406), remove:
```cpp
transforms_cache0.clear();
```

And rename all `transforms_cache0` → `transforms_cache0_` in the function body (lines 406-473).

## Verification

```bash
./colcon_tesseract.sh --packages-select trajopt_ifopt
source install/setup.bash
colcon test --packages-select trajopt_ifopt
colcon test-result --verbose
```

All existing tests should pass — the behavioral change is that TransformMap entries are overwritten in-place rather than being freed and reallocated. The FK computation (`calcFwdKin` via `operator[]`, or `getLinkTransforms` via `insert_or_assign`) overwrites every entry on every call, so stale data is impossible.
