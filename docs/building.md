# Building and Testing

This guide explains how to compile the library and run the provided unit tests.

## Building the Static Library

Run the supplied `Makefile` to build `libas5047u.a`:

```bash
make lib
```

If you want to rebuild from scratch, run `make clean` first.

The archive will be placed in the project root and you should see output similar to:

```
g++ -I./src -I. -std=c++20 -Wall -Wextra -pedantic -O2 -c src/AS5047U.cpp -o src/AS5047U.o
ar rcs libas5047u.a src/AS5047U.o
```

Object files are generated in the `src` directory.

## Running the Unit Tests

Compile and execute the tests using:

```bash
make test
```

The build creates an executable named `unit_tests` and runs it automatically.  A successful run prints:

```
All tests passed
```

If the build fails ensure your compiler supports C++20 and that `make` is installed.
