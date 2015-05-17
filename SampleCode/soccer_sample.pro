TEMPLATE = app
TARGET = soccer_sample

include(/DIST/lehre/lab_roso/tech/usr/include/settings.pri)
#include(/DIST/lehre/lab_roso/tech/usr_sim/include/settings.pri)


DEPENDPATH += src
INCLUDEPATH += src SDL/headers
LIBS += -LSDL/static -L/lib64 -lSDLmain -lSDL -ldl

DESTDIR = bin
MOC_DIR = .moc
OBJECTS_DIR = .obj
#QT += core
#QT -= gui
CONFIG +=  debug

#QMAKE_CXXFLAGS += -std=c++0x

# Input
HEADERS += \ 
    src/ballmonitor.h \
    coordinates.h \
    src/refereedisplay.h
SOURCES += soccer_sample.cpp \
    src/ballmonitor.cpp \
    coordinates.cpp \
    src/refereedisplay.cpp
		
		
##############
## Documentation
##############
# custom target 'doc' in *.pro file
dox.target = doc
dox.commands = doxygen Doxyfile
dox.depends = FORCE

# somewhere else in the *.pro file
QMAKE_EXTRA_UNIX_TARGETS += dox









