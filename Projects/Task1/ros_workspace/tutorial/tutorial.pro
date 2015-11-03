TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += src/listener.cpp src/talker.cpp

include(deployment.pri)
qtcAddDeployment()

