all: PlaybackSync.o
	g++ -O3 -o playback PlaybackSync.o -fopenmp -lboost_system -lboost_filesystem -lfreeimage -lOpenNI

PlaybackSync.o: PlaybackSync.cpp
	g++ -O3 -g -c PlaybackSync.cpp -fopenmp -o PlaybackSync.o -I/usr/include/ni/
