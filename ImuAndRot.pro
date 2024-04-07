QT       += core gui
QT       += serialport
QT       += charts

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
#INCLUDEPATH += F:\Download\eigen-master\eigen-master
INCLUDEPATH += D:\qtcode\eigen-3.4.0

#INCLUDEPATH +=$$quote(C:\software\matlab\extern\include)
#LIBS +=$$quote(C:\software\matlab\extern\lib\win64\microsoft\libeng.lib)
#LIBS +=$$quote(C:\software\matlab\extern\lib\win64\microsoft\libmat.lib)
#LIBS +=$$quote(C:\software\matlab\extern\lib\win64\microsoft\libmx.lib)
#LIBS +=$$quote(C:\software\matlab\extern\lib\win64\microsoft\libmex.lib)

SOURCES += \
    INS.cpp \
    main.cpp \
    mainwindow.cpp \
    navigation.cpp \
    portthread.cpp

HEADERS += \
    INS.h \
    mainwindow.h \
    navigation.h \
    portthread.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
