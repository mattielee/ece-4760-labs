﻿# CMAKE generated file: DO NOT EDIT!
# Generated by "NMake Makefiles" Generator, CMake Version 3.27

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

!IF "$(OS)" == "Windows_NT"
NULL=
!ELSE
NULL=nul
!ENDIF
SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\matti\Pico\ece-4760-labs

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\matti\Pico\ece-4760-labs\build

# Utility rule file for ELF2UF2Build.

# Include any custom commands dependencies for this target.
include Human_Input\Keypad\CMakeFiles\ELF2UF2Build.dir\compiler_depend.make

# Include the progress variables for this target.
include Human_Input\Keypad\CMakeFiles\ELF2UF2Build.dir\progress.make

Human_Input\Keypad\CMakeFiles\ELF2UF2Build: Human_Input\Keypad\CMakeFiles\ELF2UF2Build-complete
	cd C:\Users\matti\Pico\ece-4760-labs\build\Human_Input\Keypad
	cd C:\Users\matti\Pico\ece-4760-labs\build

Human_Input\Keypad\CMakeFiles\ELF2UF2Build-complete: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-install
Human_Input\Keypad\CMakeFiles\ELF2UF2Build-complete: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-mkdir
Human_Input\Keypad\CMakeFiles\ELF2UF2Build-complete: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-download
Human_Input\Keypad\CMakeFiles\ELF2UF2Build-complete: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-update
Human_Input\Keypad\CMakeFiles\ELF2UF2Build-complete: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-patch
Human_Input\Keypad\CMakeFiles\ELF2UF2Build-complete: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-configure
Human_Input\Keypad\CMakeFiles\ELF2UF2Build-complete: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-build
Human_Input\Keypad\CMakeFiles\ELF2UF2Build-complete: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-install
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=C:\Users\matti\Pico\ece-4760-labs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'ELF2UF2Build'"
	cd C:\Users\matti\Pico\ece-4760-labs\build\Human_Input\Keypad
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E make_directory C:/Users/matti/Pico/ece-4760-labs/build/Human_Input/Keypad/CMakeFiles
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/Users/matti/Pico/ece-4760-labs/build/Human_Input/Keypad/CMakeFiles/ELF2UF2Build-complete
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/Users/matti/Pico/ece-4760-labs/build/Human_Input/Keypad/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-done
	cd C:\Users\matti\Pico\ece-4760-labs\build

Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-build: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=C:\Users\matti\Pico\ece-4760-labs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Performing build step for 'ELF2UF2Build'"
	cd C:\Users\matti\Pico\ece-4760-labs\build\elf2uf2
	$(MAKE)
	cd C:\Users\matti\Pico\ece-4760-labs\build

Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-configure: Human_Input\Keypad\elf2uf2\tmp\ELF2UF2Build-cfgcmd.txt
Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-configure: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=C:\Users\matti\Pico\ece-4760-labs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Performing configure step for 'ELF2UF2Build'"
	cd C:\Users\matti\Pico\ece-4760-labs\build\elf2uf2
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -DCMAKE_MAKE_PROGRAM:FILEPATH=nmake "-GNMake Makefiles" -S C:/Users/matti/Pico/pico-sdk/tools/elf2uf2 -B C:/Users/matti/Pico/ece-4760-labs/build/elf2uf2
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/Users/matti/Pico/ece-4760-labs/build/Human_Input/Keypad/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-configure
	cd C:\Users\matti\Pico\ece-4760-labs\build

Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-download: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-source_dirinfo.txt
Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-download: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=C:\Users\matti\Pico\ece-4760-labs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "No download step for 'ELF2UF2Build'"
	cd C:\Users\matti\Pico\ece-4760-labs\build\Human_Input\Keypad
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/Users/matti/Pico/ece-4760-labs/build/Human_Input/Keypad/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-download
	cd C:\Users\matti\Pico\ece-4760-labs\build

Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-install: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-build
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=C:\Users\matti\Pico\ece-4760-labs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No install step for 'ELF2UF2Build'"
	cd C:\Users\matti\Pico\ece-4760-labs\build\elf2uf2
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	cd C:\Users\matti\Pico\ece-4760-labs\build

Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=C:\Users\matti\Pico\ece-4760-labs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Creating directories for 'ELF2UF2Build'"
	cd C:\Users\matti\Pico\ece-4760-labs\build\Human_Input\Keypad
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -Dcfgdir= -P C:/Users/matti/Pico/ece-4760-labs/build/Human_Input/Keypad/elf2uf2/tmp/ELF2UF2Build-mkdirs.cmake
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/Users/matti/Pico/ece-4760-labs/build/Human_Input/Keypad/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-mkdir
	cd C:\Users\matti\Pico\ece-4760-labs\build

Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-patch: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-patch-info.txt
Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-patch: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-update
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=C:\Users\matti\Pico\ece-4760-labs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "No patch step for 'ELF2UF2Build'"
	cd C:\Users\matti\Pico\ece-4760-labs\build\Human_Input\Keypad
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/Users/matti/Pico/ece-4760-labs/build/Human_Input/Keypad/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-patch
	cd C:\Users\matti\Pico\ece-4760-labs\build

Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-update: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-update-info.txt
Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-update: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-download
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=C:\Users\matti\Pico\ece-4760-labs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "No update step for 'ELF2UF2Build'"
	cd C:\Users\matti\Pico\ece-4760-labs\build\Human_Input\Keypad
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E echo_append
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E touch C:/Users/matti/Pico/ece-4760-labs/build/Human_Input/Keypad/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-update
	cd C:\Users\matti\Pico\ece-4760-labs\build

ELF2UF2Build: Human_Input\Keypad\CMakeFiles\ELF2UF2Build
ELF2UF2Build: Human_Input\Keypad\CMakeFiles\ELF2UF2Build-complete
ELF2UF2Build: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-build
ELF2UF2Build: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-configure
ELF2UF2Build: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-download
ELF2UF2Build: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-install
ELF2UF2Build: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-mkdir
ELF2UF2Build: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-patch
ELF2UF2Build: Human_Input\Keypad\elf2uf2\src\ELF2UF2Build-stamp\ELF2UF2Build-update
ELF2UF2Build: Human_Input\Keypad\CMakeFiles\ELF2UF2Build.dir\build.make
.PHONY : ELF2UF2Build

# Rule to build all files generated by this target.
Human_Input\Keypad\CMakeFiles\ELF2UF2Build.dir\build: ELF2UF2Build
.PHONY : Human_Input\Keypad\CMakeFiles\ELF2UF2Build.dir\build

Human_Input\Keypad\CMakeFiles\ELF2UF2Build.dir\clean:
	cd C:\Users\matti\Pico\ece-4760-labs\build\Human_Input\Keypad
	$(CMAKE_COMMAND) -P CMakeFiles\ELF2UF2Build.dir\cmake_clean.cmake
	cd C:\Users\matti\Pico\ece-4760-labs\build
.PHONY : Human_Input\Keypad\CMakeFiles\ELF2UF2Build.dir\clean

Human_Input\Keypad\CMakeFiles\ELF2UF2Build.dir\depend:
	$(CMAKE_COMMAND) -E cmake_depends "NMake Makefiles" C:\Users\matti\Pico\ece-4760-labs C:\Users\matti\Pico\ece-4760-labs\Human_Input\Keypad C:\Users\matti\Pico\ece-4760-labs\build C:\Users\matti\Pico\ece-4760-labs\build\Human_Input\Keypad C:\Users\matti\Pico\ece-4760-labs\build\Human_Input\Keypad\CMakeFiles\ELF2UF2Build.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : Human_Input\Keypad\CMakeFiles\ELF2UF2Build.dir\depend

