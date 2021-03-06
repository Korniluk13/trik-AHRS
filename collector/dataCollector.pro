TRIKRUNTIME_SOURCES_PATH = ../../trik/trikRuntime
TRIKRUNTIME_PATH = $$TRIKRUNTIME_SOURCES_PATH/bin/arm-release

QT += network

TEMPLATE = app
CONFIG += console

INCLUDEPATH += \
	$$TRIKRUNTIME_SOURCES_PATH/trikControl/include/ \
        $$TRIKRUNTIME_SOURCES_PATH/trikKernel/include/ \

LIBS += -L$$TRIKRUNTIME_PATH -ltrikControl -ltrikKernel -lqslog -ltrikHal

QMAKE_LFLAGS += -Wl,-O1,-rpath,.

QMAKE_CXXFLAGS += -std=c++11 -g -fno-omit-frame-pointer

HEADERS += \
	collector.h \

SOURCES += \
	main.cpp \
	collector.cpp \
