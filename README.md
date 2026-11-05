# RevoluteArm

Template-heavy C++23 kinematics for robot arms whose joints each have a single
rotational degree of freedom, with link twists restricted to 0°, +90° or −90°.
Developed for a small drawing arm driven by an ESP32.

> **Status:** work in progress. Interfaces may still change.

## Features

- Arms described at compile time: `RevoluteArm<T, Twist...>`, built from links
  (translation along the common normal, translation along the rotation axis, twist)
- Forward kinematics returning the full tool pose
- Analytic inverse kinematics for
  - 4R arms (base rotation + planar 3R), e.g. pen or spray tools
  - 5R arms (4R + wrist roll), e.g. two-finger grippers
- Splitting arms into prefix and suffix chains: `arm.prefix(at<K>)`, `arm.suffix(at<K>, base)`
- Small fixed-size linear algebra library without heap allocation

## Repository layout

| Path | Contents |
|---|---|
| `robot-arm-code/c++/libs/` | header-only libraries: `linalg`, `geometry`, `kinematics` |
| `robot-arm-code/c++/tests/` | randomized round-trip tests (inverse → forward) |
| `robot-arm-code/c++/bench/` | accuracy and cache benchmarks |

## Building and testing

```bash
cd robot-arm-code/c++
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build
```

Requires a compiler with C++23 support (e.g. GCC 13 or later).

## Documentation

Conventions (frame placement, twist signs, joint zero positions, homogeneous
coordinates) and the derivations behind the solvers are described in
[`docs/`](docs/).