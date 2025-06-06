# 🧰 Building and Testing

This guide explains how to compile the library and run the provided unit tests.

## Building the Static Library

- 🔨 Run `make lib` to build `libas5047u.a`
- 🧹 Use `make clean` before rebuilding

The archive will be placed in the project root. Typical output:
```
g++ -I./src -I. -std=c++20 -Wall -Wextra -pedantic -O2 -c src/AS5047U.cpp -o src/AS5047U.o
ar rcs libas5047u.a src/AS5047U.o
```

Object files are generated in the `src` directory.

## Running the Unit Tests
- 🏃 `make test` builds and runs `unit_tests`
  - Expect `All tests passed` on success


If the build fails ensure your compiler supports C++20 and that `make` is installed.

---
⬅️ **Previous:** [Setup](setup.md) | [Back to Documentation Index](README.md) | **Next:** [Using the Library](usage.md) ➡️
