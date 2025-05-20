# Bimini
Bimini is a C++ library that provides an interface between Building Information Modeling (BIM) data (such as IFC files) and simulation environments (such as ROS - Robot Operating System).

## Overview
Bimini bridges the gap between architectural/construction BIM data and robotics simulation environments, enabling seamless integration of building models into robotic planning, navigation, and simulation workflows.

### Key Features
- Import IFC (Industry Foundation Classes) building models
- Convert BIM geometries to simulation-compatible formats
- Extract semantic information from BIM models (rooms, doors, windows, etc.)
- Generate navigation meshes and collision models for robotic simulation
- Interface with ROS environments for robotics applications
- Support for C++20 features for modern, efficient code

## Prerequisites
- CMake 3.16 or higher
- C++20 compatible compiler
- Git (for submodule management)

## Dependencies
Bimini uses the following dependencies (included as submodules):

- [IfcPlusPlus](https://github.com/ifcquery/ifcplusplus) - A library for reading and writing IFC files
- [GoogleTest](https://github.com/google/googletest) - For unit testing

## Building
```bash
# Clone the repository with submodules
git clone --recursive https://github.com/yourusername/bimini.git
cd bimini

# If you already cloned without --recursive, get the submodules with:
git submodule update --init --recursive

# Configure with CMake and build
cmake -S . -B build && cmake --build . -j8 --config Release

# Run tests
./bimini-test
```

### Build Options
- `BIMINI_BUILD_TESTS` - Build the test suite (ON by default)
- `BIMINI_BUILD_DOCS` - Build documentation (OFF by default)

Example:
```bash
cmake -DBIMINI_BUILD_DOCS=ON ..
```

## Project Structure
```
bimini/
├── include/              # Public API headers
│   └── bimini/           # Library header files
├── src/                  # Source code
│   └── bimini/           # Implementation files
├── test/                 # Test code
│   └── bimini/           # Unit and integration tests
├── deps/                 # Dependencies (submodules)
│   ├── googletest/       # Google Test framework
│   └── ifcplusplus/      # IFC file handling library
├── docs/                 # Documentation
└── cmake/                # CMake modules and config files
```

## Usage Example

```cpp
#include <bimini/importer.hpp>
#include <bimini/simulator.hpp>

int main() {
    // Import an IFC building model
    bimini::Importer importer;
    auto building = importer.importIFC("office_building.ifc");
    
    // Convert to simulation model
    bimini::Simulator simulator;
    auto simModel = simulator.convertToSimulation(building);
    
    // Export for ROS
    simModel.exportToROSEnvironment("/path/to/ros/workspace");
    
    return 0;
}
```

## Documentation
To build the documentation:
```bash
cmake -DBIMINI_BUILD_DOCS=ON ..
make docs
```

Documentation will be generated in the `build/docs` directory.

## License
[Your chosen license - e.g. MIT, Apache 2.0, etc.]

## Contributing
Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the project
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request
