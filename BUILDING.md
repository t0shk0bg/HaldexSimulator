# Building HaldexController

## Prerequisites

### Windows
- **CMake ≥ 3.16** — [cmake.org/download](https://cmake.org/download/) — при инсталация избери "Add CMake to the system PATH"
- **Git** — [git-scm.com](https://git-scm.com/) — нужен за сваляне на GoogleTest
- **MSVC** — Visual Studio 2019 или 2022 с "Desktop development with C++" workload (или Build Tools)

### Mac
- **Xcode Command Line Tools** — дава clang, make и git наведнъж:
  ```sh
  xcode-select --install
  ```
- **CMake ≥ 3.16** — най-лесно през Homebrew:
  ```sh
  brew install cmake
  ```

Провери дали cmake е в PATH:
```
cmake --version
```

---

## Build от терминал

### Windows
```bat
scripts\build.bat
```
Или ръчно:
```bat
cmake -S . -B build
cmake --build build --config Release
```
Изпълнимият файл: `build\bin\Release\haldex.exe`

### Mac
```sh
scripts/build.sh
```
Или ръчно:
```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```
Изпълнимият файл: `build/bin/haldex`

---

## Build от VSCode

Отвори папката, след което:
- **Ctrl+Shift+B** — Build
- **Terminal → Run Task → Run Tests** — unit тестове

За debug: F5 → избери "Run HaldexController (Windows)" или "Run HaldexController (Mac)".

---

## Тестове

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
