TEMPLATE = app
TARGET = robosoccer

# Choose if you want to compile for simulation here
DEFINES += SIMULATION

# Choose your path planning algorihtm here. Note: Using A* is not recommended as it is not maintained anymore and is therefore not guaranteed to work.
#DEFINES += PATHPLANNING_ASTAR
DEFINES += PATHPLANNING_POLYGONS

# Choose the verbosity of the ouput here. VERBOSE is recommended.
DEFINES += VERBOSE
#DEFINES += VERY_VERBOSE

# Choose if you want to activate the record of the stack in case of unexpected crash.
#DEFINES += STACK_LOG


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










