# Ataboy
Original DMG Game Boy emulator in C++

How to compile :

Windows:

	- Visual Studio 2017 is required.
	- You need to install SDL2 :
		- Get vcpkg from https://github.com/Microsoft/vcpkg
		- Follow the instructions for bootstrap
		- .\vcpkg integrate install
		- .\vcpkg install sdl2:x64-windows
	
	- Open Ataboy.sln
	- Build
	
Linux:

	- C++11 compatible compiler required
	- Install SDL2 with your package manager
	- Install CMake with your package manager
	- Open a terminal on the source directory
	- mkdir build
	- cd build
	- cmake ..
	- make
	
How to use :

	- Don't use it. There are tons of better Game Boy emulators out here.

	- Ataboy.exe rom.gb
	
	- Keys are :
		- Arrow keys for D-Pad
		- z for B button
		- x for A button
		- c for Start button
		- v for Select button

It's a small hobby project inspired by Bisqwit's NES emulator : https://www.youtube.com/watch?v=y71lli8MS8s

My goal for this project are :

	- Familiarize myself with tools like GitHub, valgrind, clang, Performance Analyzer, etc...
	- Experiment with template meta programming
	- Investigate how code can be optimized.