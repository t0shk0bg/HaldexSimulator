# Building HaldexController

## Prerequisites

### Windows
- **CMake ≥ 3.16** — [cmake.org/download](https://cmake.org/download/) — during installation select "Add CMake to the system PATH"
- **Git** — [git-scm.com](https://git-scm.com/) — required to download GoogleTest
- **MSVC** — Visual Studio 2019 or 2022 with the "Desktop development with C++" workload (or Build Tools)

### Mac
- **Xcode Command Line Tools** — installs clang, make and git in one step:
  ```sh
  xcode-select --install
  ```
- **CMake ≥ 3.16** — easiest via Homebrew:
  ```sh
  brew install cmake
  ```

Verify cmake is on your PATH:
```
cmake --version
```

---

## Build from terminal

### Windows
```bat
scripts\build.bat
```
Or manually:
```bat
cmake -S . -B build
cmake --build build --config Release
```
Output binary: `build\bin\Release\haldex.exe`

### Mac
```sh
scripts/build.sh
```
Or manually:
```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```
Output binary: `build/bin/haldex`

---

## Build from VSCode

Open the project folder, then:
- **Ctrl+Shift+B** — Build
- **Terminal → Run Task → Run Tests** — run unit tests

For debugging: press F5 and select "Run HaldexController (Windows)" or "Run HaldexController (Mac)".

---

## Tests

### Windows
```bat
cd build
ctest --output-on-failure -C Release -V
```

### Mac
```sh
cd build
ctest --output-on-failure -V
```
