# Quick Start Workflow

Follow these steps to go from a fresh clone to running the example code.

1. **Setup** – install `g++`, `make` and `git`. On Debian/Ubuntu:
   ```bash
   sudo apt-get update
   sudo apt-get install build-essential git
   ```
   Verify the compiler works:
   ```bash
   g++ --version
   ```

2. **Clone the repository**
   ```bash
   git clone https://github.com/N3b3x/HF-AS5047U.git
   cd HF-AS5047U
   ```

3. **Build the library**
   ```bash
   make lib
   ```
   Successful output ends with something like:
   ```
   ar rcs libas5047u.a src/AS5047U.o
   ```

4. **Run the unit tests**
   ```bash
   make test
   ```
   Expect to see `All tests passed`.

5. **Integrate the driver** – create an `spiBus` as shown in [Using the Library](usage.md) and link against `libas5047u.a` in your project.

6. **Try the examples**
   ```bash
   cd examples/arduino_basic_interface
   # open `main.ino` in the Arduino IDE and upload
   ```
   The serial monitor should show angle readings similar to `Angle: 16384`.

