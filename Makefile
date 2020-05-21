# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #


LOPENIP = ~/Desktop/my_include/openip/libopenip.a /usr/local/opt/libjpeg-turbo/lib/libturbojpeg.a -lpng
COMPILER = gcc
OPTIMIZATION = -O0 -g
FLAGS = -ansi -std=c99 -Wall $(OPTIMIZATION)
CC = $(COMPILER) $(FLAGS)
CO = $(CC) -c

raytrace: vec3d.o quaternion.o raytrace.o
	$(CC) vec3d.o quaternion.o raytrace.o $(LOPENIP) -o raytrace

raytrace.o: raytrace.c
	$(CO) raytrace.c -I ~/Desktop/my_include/openip/ -o raytrace.o

vec3d.o: vec3d.h vec3d.c
	$(CO) vec3d.c -o vec3d.o

quaternion.o: quaternion.h quaternion.c
	$(CO) quaternion.c -o quaternion.o

clean:
	rm -rf *.o
	rm -rf raytrace

