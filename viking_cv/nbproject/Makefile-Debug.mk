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
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/CConnection.o \
	${OBJECTDIR}/CConnectionServer.o \
	${OBJECTDIR}/CFrameGrinder.o \
	${OBJECTDIR}/CGpioLed.o \
	${OBJECTDIR}/CLedStrip.o \
	${OBJECTDIR}/CMessageFromClient.o \
	${OBJECTDIR}/CTargetInfo.o \
	${OBJECTDIR}/CTestMonitor.o \
	${OBJECTDIR}/CToteDetector.o \
	${OBJECTDIR}/CToteRectangle.o \
	${OBJECTDIR}/CVideoFrame.o \
	${OBJECTDIR}/CVideoFrameQueue.o \
	${OBJECTDIR}/dbgMsg.o \
	${OBJECTDIR}/main.o


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
LDLIBSOPTIONS=-lopencv_highgui -lopencv_core -lopencv_imgproc -lpthread

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/viking_cv

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/viking_cv: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/viking_cv ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/CConnection.o: CConnection.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DCV_DEBUG_CONNECT -I../../../usr/include -MMD -MP -MF $@.d -o ${OBJECTDIR}/CConnection.o CConnection.cpp

${OBJECTDIR}/CConnectionServer.o: CConnectionServer.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DCV_DEBUG_CONNECT -I../../../usr/include -MMD -MP -MF $@.d -o ${OBJECTDIR}/CConnectionServer.o CConnectionServer.cpp

${OBJECTDIR}/CFrameGrinder.o: CFrameGrinder.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DCV_DEBUG_CONNECT -I../../../usr/include -MMD -MP -MF $@.d -o ${OBJECTDIR}/CFrameGrinder.o CFrameGrinder.cpp

${OBJECTDIR}/CGpioLed.o: CGpioLed.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DCV_DEBUG_CONNECT -I../../../usr/include -MMD -MP -MF $@.d -o ${OBJECTDIR}/CGpioLed.o CGpioLed.cpp

${OBJECTDIR}/CLedStrip.o: CLedStrip.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DCV_DEBUG_CONNECT -I../../../usr/include -MMD -MP -MF $@.d -o ${OBJECTDIR}/CLedStrip.o CLedStrip.cpp

${OBJECTDIR}/CMessageFromClient.o: CMessageFromClient.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DCV_DEBUG_CONNECT -I../../../usr/include -MMD -MP -MF $@.d -o ${OBJECTDIR}/CMessageFromClient.o CMessageFromClient.cpp

${OBJECTDIR}/CTargetInfo.o: CTargetInfo.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DCV_DEBUG_CONNECT -I../../../usr/include -MMD -MP -MF $@.d -o ${OBJECTDIR}/CTargetInfo.o CTargetInfo.cpp

${OBJECTDIR}/CTestMonitor.o: CTestMonitor.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DCV_DEBUG_CONNECT -I../../../usr/include -MMD -MP -MF $@.d -o ${OBJECTDIR}/CTestMonitor.o CTestMonitor.cpp

${OBJECTDIR}/CToteDetector.o: CToteDetector.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DCV_DEBUG_CONNECT -I../../../usr/include -MMD -MP -MF $@.d -o ${OBJECTDIR}/CToteDetector.o CToteDetector.cpp

${OBJECTDIR}/CToteRectangle.o: CToteRectangle.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DCV_DEBUG_CONNECT -I../../../usr/include -MMD -MP -MF $@.d -o ${OBJECTDIR}/CToteRectangle.o CToteRectangle.cpp

${OBJECTDIR}/CVideoFrame.o: CVideoFrame.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DCV_DEBUG_CONNECT -I../../../usr/include -MMD -MP -MF $@.d -o ${OBJECTDIR}/CVideoFrame.o CVideoFrame.cpp

${OBJECTDIR}/CVideoFrameQueue.o: CVideoFrameQueue.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DCV_DEBUG_CONNECT -I../../../usr/include -MMD -MP -MF $@.d -o ${OBJECTDIR}/CVideoFrameQueue.o CVideoFrameQueue.cpp

${OBJECTDIR}/dbgMsg.o: dbgMsg.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DCV_DEBUG_CONNECT -I../../../usr/include -MMD -MP -MF $@.d -o ${OBJECTDIR}/dbgMsg.o dbgMsg.cpp

${OBJECTDIR}/main.o: main.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DCV_DEBUG_CONNECT -I../../../usr/include -MMD -MP -MF $@.d -o ${OBJECTDIR}/main.o main.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/viking_cv

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
