#-------------------------------------------------
#
# Project created by QtCreator 2014-01-02T19:26:28
#
#-------------------------------------------------

QT       += core gui opengl widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

INCLUDEPATH += "D:/Program Files/MATLAB/R2013b/extern/include"

LIBS += -L"D:/Program Files/MATLAB/R2013b/extern/lib/win64/microsoft" -llibmat -llibmex -llibeng -llibmx

TARGET = ShipControl3D
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    Current.cpp \
    EnvObserver.cpp \
    Filter.cpp \
    NMPCcontroller.cpp \
    OptController.cpp \
    PIDController.cpp \
    ShipControl.cpp \
    ShipModel.cpp \
    ShipParameter.cpp \
    Tool.cpp \
    Wave.cpp \
    Wind.cpp \
    WOPC.cpp \
    ShipGraph.cpp \
	PlotData.cpp \
	OptionDialog.cpp

HEADERS  += mainwindow.h \
    Current.h \
    DataStruct.h \
    EnvObserver.h \
    Filter.h \
    NMPCcontroller.h \
    OptController.h \
    PIDController.h \
    ShipControl.h \
    ShipModel.h \
    ShipParameter.h \
    Tool.h \
    Wave.h \
    Wind.h \
    WOPC.h \
    ShipGraph.h \
	PlotData.h \
	OptionDialog.h

FORMS    += OptionDialog.ui
