TEMPLATE = app
TARGET = robosoccer

#DEFINES += SIMULATION

#DEFINES += PATHPLANNING_ASTAR
DEFINES += PATHPLANNING_POLYGONS

#DEFINES += STACK_LOG
DEFINES += VERBOSE
#DEFINES += VERY_VERBOSE

contains(DEFINES, SIMULATION) {
    include(/DIST/lehre/lab_roso/tech/usr_sim/include/settings.pri)
} else {
    include(/DIST/lehre/lab_roso/tech/usr/include/settings.pri)
}

contains(DEFINES, VERY_VERBOSE) {
    DEFINES += VERBOSE
}

DEPENDPATH += src
INCLUDEPATH += src SDL/headers
LIBS += -LSDL/static -lstdc++ -lSDLmain -lSDL -ldl -lSDL_ttf
OBJECTS += /usr/lib/libfreetype.so.6

DESTDIR = bin
MOC_DIR = .moc
OBJECTS_DIR = .obj
#QT += core
#QT -= gui
CONFIG +=  debug

# Input

HEADERS += \
    src/*.h \
    src/sdl_gfx/*.h
SOURCES +=  \
    src/*.cpp \
    src/sdl_gfx/SDL_rotozoom.c \
    src/sdl_gfx/SDL_gfxBlitFunc.c \
    src/sdl_gfx/SDL_gfxPrimitives.c
		
		
##############
## Documentation
##############
# custom target 'doc' in *.pro file
dox.target = doc
dox.commands = doxygen Doxyfile
dox.depends = FORCE

# somewhere else in the *.pro file
QMAKE_EXTRA_UNIX_TARGETS += dox
QMAKE_CXXFLAGS += -rdynamic -g










