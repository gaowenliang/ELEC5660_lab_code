#! /bin/make
##############################################
# Makefile for the 'wxPropView' sample,
##############################################
#-------------------------------------------
DEST:=$(shell basename `pwd`)

#-------------------------------------------
ifndef MVIMPACT_ACQUIRE_SOURCE_DIR
    ifndef MVIMPACT_ACQUIRE_DIR
        ROOT_PATH=../..
    else
        ROOT_PATH=$(MVIMPACT_ACQUIRE_DIR)/apps
    endif
else
    ROOT_PATH=$(MVIMPACT_ACQUIRE_SOURCE_DIR)/apps
endif

#-------------------------------------------
BINARY_NAME=wxPropView
MODULE_NAME=PropViewFrame

#-------------------------------------------
OOBJ =										\
	CaptureThread.o							\
	DataConversion.o						\
	DataGrid.o								\
	DevCtrl.o								\
	DevData.o								\
	DrawingCanvas.o							\
	HistogramCanvas.o						\
	HistogramCanvasPixel.o					\
	HistogramCanvasSpatialNoise.o			\
	HistogramCanvasTemporalNoise.o			\
	ImageAnalysisPlotControls.o				\
	ImageCanvas.o							\
	ImageCanvasCommon.o						\
	LineProfileCanvas.o						\
	LineProfileCanvasHorizontal.o			\
	LineProfileCanvasVertical.o				\
	PlotCanvas.o							\
	PlotCanvasImageAnalysis.o				\
	PlotCanvasInfo.o						\
	PlotCanvasIntensity.o					\
	PropData.o								\
	PropGridFrameBase.o						\
	PropTree.o								\
	PropViewCallback.o						\
	SpinEditorDouble.o						\
	spinctld.o								\
	ValuesFromUserDlg.o						\
	VectorScopeCanvas.o						\
	WizardLensControl.o						\
	WizardLUTControl.o						\
	WizardQuickSetup.o						\
	WizardSaturation.o						\
	WizardSequencerControl.o

#-------------------------------------------
vpath %.cpp ..

#-----------------------------------------
LOC_LIB_PATH=$(WORK)/lib/$(TARGET)

#-------------------------------------------
# 'WX_CONF_TOOL' will be set in Makefile.samp.inc
#-------------------------------------------
WXRELEASE=$(shell $(WX_CONF_TOOL) --release)
WXLIBFLAGS=$(shell $(WX_CONF_TOOL)  --libs)
WXOPTIONALLIBFLAGS=$(shell $(WX_CONF_TOOL)  --optional-libs propgrid)
WXCXXFLAGS=$(shell $(WX_CONF_TOOL)  --cxxflags)

#-------------------------------------------
USR_INCLS =								\

#-------------------------------------------
USR_LIBS =								\
	$(ADD_LIBS)

#-------------------------------------------
LDOPT_1 = \
	$(WXLIBFLAGS) \
	$(WXOPTIONALLIBFLAGS)

ifneq ($(DEST),ppc603e)
LOCALFLAGS=								\
	-Wno-variadic-macros -Wno-strict-aliasing
endif
#-------------------------------------------
# Local typedefs warning since gcc version 4.8 (wxWidgets code)
GCC_MAJOR          := $(shell g++ -dumpversion 2>&1 | cut -d'.' -f1)
GCC_MINOR          := $(shell g++ -dumpversion 2>&1 | cut -d'.' -f2)
NOLOCALTYPEDEFWARNING := $(shell if [ $(GCC_MAJOR) -ge 4 ] && [ $(GCC_MINOR) -ge 8 ]; then		\
						 echo "-Wno-unused-local-typedefs";			\
						 else										\
						 echo "";									\
						 fi )
#-------------------------------------------
CPPFLAGS_1 = $(NOLOCALTYPEDEFWARNING) 	\
	-pedantic -Wno-long-long            \
	$(WXCXXFLAGS)						\
	$(LOCALFLAGS)

#	-DUSE_RAW_BITMAP_SUPPORT

#-------------------------------------------
include $(ROOT_PATH)/Makefile.samp.inc
