debug: build/Debug/Makefile
	cd build/Debug && make install -j
	rm -f build/Bin
	ln -s Debug/Bin/ build/Bin

release: build/Release/Makefile
	cd build/Release && make install -j
	rm -f build/Bin
	ln -s Release/Bin/ build/Bin

clean_debug:
	cd build/Debug && make clean

clean_release:
	cd build/Debug && make clean

wipe_debug:
	rm -rf build/Debug

wipe_release:
	rm -rf build/Release

build/Release/Makefile: build/Release
	cd build/Release && cmake ../.. -DMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=./Bin

build/Release:
	mkdir -p build/Release

build/Debug/Makefile: build/Debug
	cd build/Debug && cmake ../.. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=./Bin

build/Debug:
	mkdir -p build/Debug
