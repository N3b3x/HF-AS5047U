# 🏁 Quick Start Workflow
Follow these steps to go from a fresh clone to running the example code.


- 🛠️ **Setup** – install `g++`, `make` and `git` (Debian/Ubuntu example)
  ```bash
  sudo apt-get update
  sudo apt-get install build-essential git
  g++ --version
  ```
- 📥 **Clone** the repository
  ```bash
  git clone https://github.com/N3b3x/HF-AS5047U.git
  cd HF-AS5047U
  ```
- 🔨 **Build** the library
  ```bash
  make lib
  ```
  Typical output:
  ```
  ar rcs libas5047u.a src/AS5047U.o
  ```
- ✅ **Run the unit tests**
  ```bash
  make test
  ```
  Expect `All tests passed`.
- 🧩 **Integrate** the driver as shown in [Using the Library](usage.md) and link against `libas5047u.a` in your project.
- 🚀 **Try the examples**
  ```bash
  cd examples/arduino_basic_interface
  # open `main.ino` in the Arduino IDE and upload
  ```
  The serial monitor should show angle readings similar to `Angle: 16384`.

---
⬅️ **Previous:** [Running the Examples](examples.md) | [Back to Documentation Index](README.md)
