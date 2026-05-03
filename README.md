# Qt Stochastic Planner

A Rapidly-exploring Random Tree (RRT) path planner for a differential-drive
mobile robot, with a Qt-based visualization and a Kobuki simulator/driver
integration. Originally developed as part of a bachelor thesis (2020).

> **Status:** Frozen at `v1.0-thesis`. This release preserves the code as
> delivered for the thesis. Active development continues on a `v2` line that
> extracts the planner into a Qt-free core library, drops the Kobuki simulator,
> and targets Unreal Engine and WebAssembly front-ends.

## Features

- 2D RRT planner with simple kinematic constraints (max linear / angular speed)
- Live visualization of the growing tree and final path
- Map loading from a static obstacle file (`priestor.txt`)
- RPLIDAR and Kobuki driver wrappers for hardware experiments

## Repository layout

```
demoRMR/
├── main.cpp / mainwindow.*      # Qt UI
├── rrt.{h,cpp}                  # RRT planner (core algorithm)
├── map_loader.{h,cpp}           # Obstacle map loader
├── rplidar.{h,cpp}              # RPLIDAR driver wrapper
├── CKobuki.{h,cpp}              # Kobuki driver wrapper
├── priestor.txt                 # Sample obstacle map
└── demoRMR.pro                  # qmake project file
```

## Build

Requires Qt 5 (Widgets) and a C++11 compiler.

```sh
cd demoRMR
qmake demoRMR.pro
make
./demoRMR
```

## License

MIT — see [LICENSE](LICENSE).

Some files in `demoRMR/` originate from the RMR course at FEI STU and are
**not** covered by the MIT license. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

## Acknowledgements

- Thesis supervisor: **Ing. Martin Dekan, PhD.** (FEI STU)
- RMR course lecturer: **prof. Ing. František Duchoň, PhD.** (FEI STU)
- Kobuki driver and RPLIDAR wrapper: Martin Dekan & Peter Beno (RMR course materials)