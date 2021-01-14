# rmf_performance_tests
Performance tests for RMF

## Installation
```bash
git clone https://github.com/osrf/rmf_performance_tests.git
cd rmf_performance_tests
mkdir build && cd build
cmake ..
make install
```

## Testing

```bash
cd build
./test_planner {SCENARIO_NAME}
```