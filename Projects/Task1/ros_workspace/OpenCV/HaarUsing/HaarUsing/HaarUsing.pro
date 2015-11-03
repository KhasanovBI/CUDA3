TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

LIBS += -L/usr/local/lib -L/usr/local/cuda-7.5/lib64 -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_objdetect -lopencv_ocl -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab -lcufft -lnpps -lnppi -lnppc -lcudart -lrt -lpthread -lm -ldl -lopencv_nonfree

SOURCES += main.cpp

include(deployment.pri)
qtcAddDeployment()
