#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux-x86
CND_DLIB_EXT=so
CND_CONF=Release
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/dynctrl.o \
	${OBJECTDIR}/input_uvc.o \
	${OBJECTDIR}/jpeg_utils.o \
	${OBJECTDIR}/v4l2uvc.o \
	${OBJECTDIR}/viking_cv/CConnection.o \
	${OBJECTDIR}/viking_cv/CConnectionServer.o \
	${OBJECTDIR}/viking_cv/CFrameGrinder.o \
	${OBJECTDIR}/viking_cv/CGpioLed.o \
	${OBJECTDIR}/viking_cv/CLedStrip.o \
	${OBJECTDIR}/viking_cv/CMessageFromClient.o \
	${OBJECTDIR}/viking_cv/CTargetInfo.o \
	${OBJECTDIR}/viking_cv/CTestMonitor.o \
	${OBJECTDIR}/viking_cv/CToteDetector.o \
	${OBJECTDIR}/viking_cv/CToteRectangle.o \
	${OBJECTDIR}/viking_cv/CVideoFrame.o \
	${OBJECTDIR}/viking_cv/CVideoFrameQueue.o \
	${OBJECTDIR}/viking_cv/dbgMsg.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_superres -lopencv_video

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ../../dist/${CND_CONF}/${CND_PLATFORM}/input_uvc.${CND_DLIB_EXT}

../../dist/${CND_CONF}/${CND_PLATFORM}/input_uvc.${CND_DLIB_EXT}: ${OBJECTFILES}
	${MKDIR} -p ../../dist/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ../../dist/${CND_CONF}/${CND_PLATFORM}/input_uvc.${CND_DLIB_EXT} ${OBJECTFILES} ${LDLIBSOPTIONS} -ljpeg -shared -fPIC

${OBJECTDIR}/dynctrl.o: dynctrl.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/dynctrl.o dynctrl.cpp

${OBJECTDIR}/input_uvc.o: input_uvc.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/input_uvc.o input_uvc.cpp

${OBJECTDIR}/jpeg_utils.o: jpeg_utils.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/jpeg_utils.o jpeg_utils.cpp

${OBJECTDIR}/v4l2uvc.o: v4l2uvc.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/v4l2uvc.o v4l2uvc.cpp

${OBJECTDIR}/viking_cv/CConnection.o: viking_cv/CConnection.cpp 
	${MKDIR} -p ${OBJECTDIR}/viking_cv
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/viking_cv/CConnection.o viking_cv/CConnection.cpp

${OBJECTDIR}/viking_cv/CConnectionServer.o: viking_cv/CConnectionServer.cpp 
	${MKDIR} -p ${OBJECTDIR}/viking_cv
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/viking_cv/CConnectionServer.o viking_cv/CConnectionServer.cpp

${OBJECTDIR}/viking_cv/CFrameGrinder.o: viking_cv/CFrameGrinder.cpp 
	${MKDIR} -p ${OBJECTDIR}/viking_cv
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/viking_cv/CFrameGrinder.o viking_cv/CFrameGrinder.cpp

${OBJECTDIR}/viking_cv/CGpioLed.o: viking_cv/CGpioLed.cpp 
	${MKDIR} -p ${OBJECTDIR}/viking_cv
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/viking_cv/CGpioLed.o viking_cv/CGpioLed.cpp

${OBJECTDIR}/viking_cv/CLedStrip.o: viking_cv/CLedStrip.cpp 
	${MKDIR} -p ${OBJECTDIR}/viking_cv
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/viking_cv/CLedStrip.o viking_cv/CLedStrip.cpp

${OBJECTDIR}/viking_cv/CMessageFromClient.o: viking_cv/CMessageFromClient.cpp 
	${MKDIR} -p ${OBJECTDIR}/viking_cv
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/viking_cv/CMessageFromClient.o viking_cv/CMessageFromClient.cpp

${OBJECTDIR}/viking_cv/CTargetInfo.o: viking_cv/CTargetInfo.cpp 
	${MKDIR} -p ${OBJECTDIR}/viking_cv
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/viking_cv/CTargetInfo.o viking_cv/CTargetInfo.cpp

${OBJECTDIR}/viking_cv/CTestMonitor.o: viking_cv/CTestMonitor.cpp 
	${MKDIR} -p ${OBJECTDIR}/viking_cv
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/viking_cv/CTestMonitor.o viking_cv/CTestMonitor.cpp

${OBJECTDIR}/viking_cv/CToteDetector.o: viking_cv/CToteDetector.cpp 
	${MKDIR} -p ${OBJECTDIR}/viking_cv
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/viking_cv/CToteDetector.o viking_cv/CToteDetector.cpp

${OBJECTDIR}/viking_cv/CToteRectangle.o: viking_cv/CToteRectangle.cpp 
	${MKDIR} -p ${OBJECTDIR}/viking_cv
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/viking_cv/CToteRectangle.o viking_cv/CToteRectangle.cpp

${OBJECTDIR}/viking_cv/CVideoFrame.o: viking_cv/CVideoFrame.cpp 
	${MKDIR} -p ${OBJECTDIR}/viking_cv
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/viking_cv/CVideoFrame.o viking_cv/CVideoFrame.cpp

${OBJECTDIR}/viking_cv/CVideoFrameQueue.o: viking_cv/CVideoFrameQueue.cpp 
	${MKDIR} -p ${OBJECTDIR}/viking_cv
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/viking_cv/CVideoFrameQueue.o viking_cv/CVideoFrameQueue.cpp

${OBJECTDIR}/viking_cv/dbgMsg.o: viking_cv/dbgMsg.cpp 
	${MKDIR} -p ${OBJECTDIR}/viking_cv
	${RM} $@.d
	$(COMPILE.cc) -O2 -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/viking_cv/dbgMsg.o viking_cv/dbgMsg.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ../../dist/${CND_CONF}/${CND_PLATFORM}/input_uvc.${CND_DLIB_EXT}

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
