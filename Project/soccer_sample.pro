TEMPLATE = app
TARGET = soccer_sample

#include(/DIST/lehre/lab_roso/tech/usr/include/settings.pri)
include(/DIST/lehre/lab_roso/tech/usr_sim/include/settings.pri)


DEPENDPATH += src
INCLUDEPATH += src SDL/headers
LIBS += -LSDL/static -lSDLmain -lSDL -ldl -lSDL_ttf
OBJECTS += /usr/lib/libfreetype.so.6

DESTDIR = bin
MOC_DIR = .moc
OBJECTS_DIR = .obj
#QT += core
#QT -= gui
CONFIG +=  debug

#QMAKE_CXXFLAGS += -std=c++0x

# Input

HEADERS += \ 
    src/*.h \
    src/sdl_gfx/*.h
SOURCES +=  \
    src/*.cpp \
    src/sdl_gfx/SDL_rotozoom.c
		
		
##############
## Documentation
##############
# custom target 'doc' in *.pro file
dox.target = doc
dox.commands = doxygen Doxyfile
dox.depends = FORCE

# somewhere else in the *.pro file
QMAKE_EXTRA_UNIX_TARGETS += dox





