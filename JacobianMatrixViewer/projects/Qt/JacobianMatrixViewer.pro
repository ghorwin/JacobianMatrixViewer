# Project file for JacobianMatrixViewer
# remember to set DYLD_FALLBACK_LIBRARY_PATH on MacOSX
# set LD_LIBRARY_PATH on Linux

TARGET = JacobianMatrixViewer
TEMPLATE = app

# this pri must be sourced from all our applications
include( ../../../externals/IBK/projects/Qt/IBK.pri )

QT += xml printsupport widgets svg

CONFIG += c++11

unix {
	QMAKE_CXXFLAGS += -std=c++11
}

LIBS += -L../../../lib$${DIR_PREFIX} \
	-lIBK \
	-lIBKMK

win32 {
}

linux {
}

mac {
}

INCLUDEPATH = \
	../../src \
	../../../externals/IBK/src \
	../../../externals/IBKMK/src

DEPENDPATH = $${INCLUDEPATH}

win32 {

	PRE_TARGETDEPS += \
								$$PWD/../../../externals/lib$${DIR_PREFIX}/IBK.lib \
								$$PWD/../../../externals/lib$${DIR_PREFIX}/IBKMK.lib

}

FORMS += \
	../../src/MainDialog.ui

HEADERS += \
	../../src/MainDialog.h \
	../../src/MatrixAdapter.h \
	../../src/MatrixInterfaceBand.h \
	../../src/MatrixInterfaceSparse.h \
	../../src/MatrixTableModel.h \
	../../src/MatrixVisualizer.h

SOURCES += \
	../../src/MainDialog.cpp \
	../../src/MatrixAdapter.cpp \
	../../src/MatrixTableModel.cpp \
	../../src/main.cpp

RESOURCES += \
	../../resources/JacobianMatrixViewer.qrc
