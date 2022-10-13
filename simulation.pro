QT += core gui opengl

TARGET = simulation
TEMPLATE = app

QMAKE_CXXFLAGS += -mstackrealign
CONFIG += c++17

unix:!macx {
    LIBS += -lGLU
}
win32 {
    DEFINES += GLEW_STATIC
    LIBS += -lopengl32 -lglu32
}

QMAKE_CXXFLAGS += -msse2
CONFIG += sanitizer sanitize_address
CONFIG += sanitizer sanitize_undefined

SOURCES += \
    libs/glew-1.10.0/src/glew.c \
    src/main.cpp \
    src/mainwindow.cpp \
    src/solver.cpp \
    src/system.cpp \
    src/view.cpp \
    src/viewformat.cpp \
    src/graphics/Shader.cpp \
    src/graphics/GraphicsDebug.cpp \
    src/simulation.cpp \
    src/graphics/shape.cpp \
    src/graphics/camera.cpp \
    src/graphics/MeshLoader.cpp

HEADERS += \
    src/mainwindow.h \
    src/solver.h \
    src/system.h \
    src/view.h \
    src/viewformat.h \
    src/graphics/Shader.h \
    src/graphics/ShaderAttribLocations.h \
    src/graphics/GraphicsDebug.h \
    src/simulation.h \
    src/graphics/shape.h \
    src/graphics/camera.h \
    ui_mainwindow.h \
    src/graphics/MeshLoader.h

FORMS += src/mainwindow.ui

RESOURCES += \
    res/shaders/shaders.qrc

DISTFILES += \
    res/shaders/shader.vert \
    res/shaders/shader.frag

INCLUDEPATH += src libs glm libs/glew-1.10.0/include libs/Eigen/
DEPENDPATH += src libs glm libs/glew-1.10.0/include libs/Eigen/

HOME_DIR = $$(HOME)
QMAKE_CXXFLAGS = -I "$${HOME_DIR}/eigen-git-mirror"
INCLUDEPATH += "$${HOME_DIR}/eigen-git-mirror"
DEPENDPATH += "$${HOME_DIR}/eigen-git-mirror"

