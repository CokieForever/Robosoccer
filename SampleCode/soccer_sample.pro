TEMPLATE = app
TARGET = soccer_sample

include(/DIST/lehre/lab_roso/tech/usr/include/settings.pri)

DEPENDPATH += src
INCLUDEPATH += src

DESTDIR = bin
MOC_DIR = .moc
OBJECTS_DIR = .obj
#QT += core
#QT -= gui
CONFIG +=  debug

# Input
HEADERS += 
SOURCES += soccer_sample.cpp
		
		
##############
## Documentation
##############
# custom target 'doc' in *.pro file
dox.target = doc
dox.commands = doxygen Doxyfile
dox.depends = FORCE

# somewhere else in the *.pro file
QMAKE_EXTRA_UNIX_TARGETS += dox

