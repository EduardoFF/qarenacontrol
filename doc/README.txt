Some notes ...

###########################################################################################

EPFL tool chain installation:
- Prerequisites:
1.-cmake 2.8.9 version (2.8.12 version does not works correctly)

-Installation steps:
1.- Download EPFL toolchain from http://wiki.epfl.ch/mobots-robots/toolchain and follow the instructions shown there.
2.- Execute cmake -DCMAKE_TOOLCHAIN_FILE=moboard.cmake . in the correct directory
3.- Probably you experienced the following error:

	CMake Error at /usr/local/angstrom/arm/share/cmake-2.6/Modules/CMakeTestCCompiler.cmake:32 (MESSAGE):
  		The C compiler "/usr/local/angstrom/arm/bin/arm-angstrom-linux-gnueabi-gcc"
  		is not able to compile a simple test program.
  		...
  		/usr/local/angstrom/arm/libexec/gcc/arm-angstrom-linux-gnueabi/4.3.3/cc1:
  		error while loading shared libraries: libmpfr.so.1: cannot open shared
  		object file: No such file or directory
  		
  	You need to create both symbolic links to the following libraries located in /usr/lib/x86_64_linux_gnu
  	
  	libmpfr.so.1 -> libmpfr.so.4.1.2
	libgmp.so.3 -> libgmp.so.10.1.3
	
	NOTE: probably you need to change the path of the libraries depending on your linux architecture.
  	
4.- Test again and enjoy it!

###########################################################################################

LCM 1.1.2 for argos foot-bots compilation (compilation through the toolchain):

1.- Download LCM from https://github.com/lcm-proj/lcm/archive/v1.1.2.tar.gz and extract it
2.- execute source /usr/local/angstrom/arm/environment-setup
3.- execute ./bootstrap.sh
4.- execute ./configure
5.- execute make
6.- execute sudo PATH=$PATH:/usr/local/angstrom/arm/bin/ make install 	

############################################################################################




