TEMPLATE = app
TARGET = soccer_sample

#include(/DIST/lehre/lab_roso/tech/usr/include/settings.pri)
include(/DIST/lehre/lab_roso/tech/usr_sim/include/settings.pri)


DEPENDPATH += src
INCLUDEPATH += src

DESTDIR = bin
MOC_DIR = .moc
OBJECTS_DIR = .obj
#QT += core
#QT -= gui
CONFIG +=  debug

#QMAKE_CXXFLAGS += -std=c++0x

# Input
HEADERS += \ 
    src/interpreter.h \
    src/goalkeeper.h \
    src/playermain.h \
    src/playertwo.h
SOURCES += soccer_sample.cpp \
    src/interpreter.cpp \
    src/goalkeeper.cpp \
    src/playermain.cpp \
    src/playertwo.cpp
		
		
##############
## Documentation
##############
# custom target 'doc' in *.pro file
dox.target = doc
dox.commands = doxygen Doxyfile
dox.depends = FORCE

# somewhere else in the *.pro file
QMAKE_EXTRA_UNIX_TARGETS += dox







