######################################################################
# Automatically generated by qmake (3.1) Tue Aug 9 10:54:08 2022
######################################################################

TEMPLATE = app
TARGET = agvcom_linux02
INCLUDEPATH += .

# The following define makes your compiler warn you if you use any
# feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

# Input
HEADERS += agvcom_linux/settings.h \
           agvcom_linux/ui_widget.h \
           agvcom_linux/widget.h \
           agvcom_linux/include/xnetdriver/xNetDriver.h \
           agvcom_linux/include/xnetdriver/xNetRobotAcs.h \
           agvcom_linux/include/xnetdriver/xNetTagAcs.h
FORMS += agvcom_linux/widget.ui
SOURCES += agvcom_linux/main.cpp \
           agvcom_linux/settings.cpp \
           agvcom_linux/widget.cpp
TRANSLATIONS += agvcom_linux/agvcomm_zh_CN.ts
