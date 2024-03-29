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

# Utility rule file for fft_incremental_rgb_pio_h.

# Include any custom commands dependencies for this target.
include Audio\g_Audio_FFT\CMakeFiles\fft_incremental_rgb_pio_h.dir\compiler_depend.make

# Include the progress variables for this target.
include Audio\g_Audio_FFT\CMakeFiles\fft_incremental_rgb_pio_h.dir\progress.make

Audio\g_Audio_FFT\CMakeFiles\fft_incremental_rgb_pio_h: Audio\g_Audio_FFT\rgb.pio.h
	cd C:\Users\matti\Pico\ece-4760-labs\build\Audio\g_Audio_FFT
	cd C:\Users\matti\Pico\ece-4760-labs\build

Audio\g_Audio_FFT\rgb.pio.h: C:\Users\matti\Pico\ece-4760-labs\Audio\g_Audio_FFT\rgb.pio
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=C:\Users\matti\Pico\ece-4760-labs\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating rgb.pio.h"
	cd C:\Users\matti\Pico\ece-4760-labs\build\Audio\g_Audio_FFT
	..\..\pioasm\pioasm.exe -o c-sdk C:/Users/matti/Pico/ece-4760-labs/Audio/g_Audio_FFT/rgb.pio C:/Users/matti/Pico/ece-4760-labs/build/Audio/g_Audio_FFT/rgb.pio.h
	cd C:\Users\matti\Pico\ece-4760-labs\build

fft_incremental_rgb_pio_h: Audio\g_Audio_FFT\CMakeFiles\fft_incremental_rgb_pio_h
fft_incremental_rgb_pio_h: Audio\g_Audio_FFT\rgb.pio.h
fft_incremental_rgb_pio_h: Audio\g_Audio_FFT\CMakeFiles\fft_incremental_rgb_pio_h.dir\build.make
.PHONY : fft_incremental_rgb_pio_h

# Rule to build all files generated by this target.
Audio\g_Audio_FFT\CMakeFiles\fft_incremental_rgb_pio_h.dir\build: fft_incremental_rgb_pio_h
.PHONY : Audio\g_Audio_FFT\CMakeFiles\fft_incremental_rgb_pio_h.dir\build

Audio\g_Audio_FFT\CMakeFiles\fft_incremental_rgb_pio_h.dir\clean:
	cd C:\Users\matti\Pico\ece-4760-labs\build\Audio\g_Audio_FFT
	$(CMAKE_COMMAND) -P CMakeFiles\fft_incremental_rgb_pio_h.dir\cmake_clean.cmake
	cd C:\Users\matti\Pico\ece-4760-labs\build
.PHONY : Audio\g_Audio_FFT\CMakeFiles\fft_incremental_rgb_pio_h.dir\clean

Audio\g_Audio_FFT\CMakeFiles\fft_incremental_rgb_pio_h.dir\depend:
	$(CMAKE_COMMAND) -E cmake_depends "NMake Makefiles" C:\Users\matti\Pico\ece-4760-labs C:\Users\matti\Pico\ece-4760-labs\Audio\g_Audio_FFT C:\Users\matti\Pico\ece-4760-labs\build C:\Users\matti\Pico\ece-4760-labs\build\Audio\g_Audio_FFT C:\Users\matti\Pico\ece-4760-labs\build\Audio\g_Audio_FFT\CMakeFiles\fft_incremental_rgb_pio_h.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : Audio\g_Audio_FFT\CMakeFiles\fft_incremental_rgb_pio_h.dir\depend

