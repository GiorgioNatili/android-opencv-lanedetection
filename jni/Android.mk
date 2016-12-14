# References:
# https://docs.nvidia.com/gameworks/content/technologies/mobile/opencv_tutorial_mixed_processing.htm
# https://docs.nvidia.com/gameworks/content/technologies/mobile/opencv_tutorial_cuda.htm

LOCAL_PATH := $(call my-dir)

# FIXME Do not use hard-coded SDK location here
OPENCVSDK := D:/Bibliotheques/OpenCV-2.4.9-android-sdk/sdk

include $(CLEAR_VARS)

# Cuda
INSTALL_CUDA_LIBRARIES  :=  on
WITH_CUDA               :=  on

# OpenCV
OPENCV_CAMERA_MODULES   :=  on
OPENCV_INSTALL_MODULES  :=  on

include $(OPENCVSDK)/native/jni/OpenCV.mk
#include $(OPENCVSDK)/native/jni/OpenCV-tegra3.mk

LOCAL_MODULE    := LaneDetectionNative
LOCAL_SRC_FILES := LaneDetector.cpp
LOCAL_LDFLAGS += -Wl,--gc-sections
LOCAL_LDLIBS += -llog -ldl -landroid -ljnigraphics -lGLESv2 -lEGL -latomic

CPPFLAGS += -fno-strict-aliasing -mfpu=vfp -mfloat-abi=softfp
LOCAL_CPP_FEATURES := rtti exceptions

include $(BUILD_SHARED_LIBRARY)
