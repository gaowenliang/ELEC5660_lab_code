#! /bin/bash
#-------------------------------------------------------------------------------
if test $# -gt 0
then
	SNAME=$1

	echo " ** ********************************************* **"
	echo " ** MATRIX VISION GmbH - mvIMPACT Acquire sample!"
	echo " ** ********************************************* **"

#-------------------------------------------------------------------------
	mkdir $SNAME


#-------------------------------------------------------------------------
echo " ** Generating the '$SNAME/Makefile'..."

echo "#! /bin/make
##############################################
# Creation date: `date`
##############################################

OSTYPE := \$(shell uname | cut -d _ -f 1 | tr [:upper:] [:lower:])
HOSTTYPE := \$(shell uname -m)

#-------------------------------------------
ifeq (\$(OSTYPE),cygwin)
	SUBDIRS=\$(OSTYPE)
else
	ifeq (\$(HOSTTYPE),i686)
		NATIVE=x86
	else
		NATIVE=\$(HOSTTYPE)
		ifeq (\$(NATIVE),x86_64)
			EXTRA_TARGET=x86
		endif
	endif

	SUBDIRS = \$(NATIVE) ppc603e ppc_6xx arm \$(EXTRA_TARGET)
endif


#-------------------------------------------
.PHONY:	\$(SUBDIRS) all info ipk native new build strip

#-------------------------------------------
all info ipk:
	@for dir in \$(SUBDIRS) ;	\\
	do							\\
		mkdir -p \$\$dir ;		\\
		\$(MAKE) -C \$\$dir -f ../Makefile.inc \$@ || exit \$\$?;	\\
	done						\\

#-------------------------------------------
native:
	@mkdir -p \$(NATIVE)
	\$(MAKE) -C \$(NATIVE) -f ../Makefile.inc all || exit \$\$?

#-------------------------------------------
ppc603e ppc x86 x86_64 arm cygwin ppc_6xx:
	@mkdir -p \$@
	\$(MAKE) -C \$@ -f ../Makefile.inc all || exit \$\$?

#-------------------------------------------
clean:
	@rm -rf \$(SUBDIRS)

#-------------------------------------------
build new: clean
	@for dir in \$(SUBDIRS) ; 	\\
	do 							\\
		mkdir -p \$\$dir; 		\\
		\$(MAKE) -C \$\$dir -f ../Makefile.inc \$@ || exit \$\$?; \\
	done

#-------------------------------------------
strip:
	@for dir in \$(SUBDIRS);	\\
	do							\\
		if test -d \$\$dir;		\\
		then					\\
			\$(MAKE) -C \$\$dir -f ../Makefile.inc \$@ || exit \$\$?;	\\
		fi;						\\
	done

#-------------------------------------------

" > $SNAME/Makefile

#-------------------------------------------------------------------------
echo " ** Generating the '$SNAME/Makefile.inc'..."

echo "#! /bin/make
##############################################
# Makefile for the '$SNAME' sample,
# Creation date: `date`
##############################################
#-------------------------------------------
ROOT_PATH=../..

#-------------------------------------------
MODULE_NAME=$SNAME

#-------------------------------------------
OOBJ =						\\

#-------------------------------------------
vpath %.cpp ..

#-------------------------------------------
USR_INCLS =					\\

#-------------------------------------------
USR_LIBS =					\\

#-------------------------------------------
CPPFLAGS_1 = 				\\

#-------------------------------------------
include \$(ROOT_PATH)/Makefile.samp.inc

#-------------------------------------------

" > $SNAME/Makefile.inc

#-------------------------------------------------------------------------
echo " ** Generating the '$SNAME/$SNAME.cpp'..."

echo "//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// MATRIX VISION GmbH
// mvIMPACT Acquire sample >>$SNAME<<
// Creation date: `date`
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
#include <iostream>
#include <apps/Common/exampleHelper.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

//-----------------------------------------------------------------------------
using namespace mvIMPACT::acquire;
using namespace std;

//-----------------------------------------------------------------------------
bool isDeviceSupportedBySample( const Device* const pDev )
//-----------------------------------------------------------------------------
{
	return true;
}
//-----------------------------------------------------------------------------
int main(int /*argc*/, char* /*argv[]*/)
//-----------------------------------------------------------------------------
{
	DeviceManager devMgr;
	Device* pDev = getDeviceFromUserInput( devMgr, isDeviceSupportedBySample );
	if( pDev == 0 )
	{
		cout << \"Unable to continue!\";
		cout << \"Press [ENTER] to end the application\" << endl;
		cin.get();
		return 0;
	}

	try
	{
		pDev->open();
	}
	catch( const ImpactAcquireException& e )
	{
		// this e.g. might happen if the same device is already opened in another process...
		cout << \"An error occurred while opening the device(error code: \" << e.getErrorCode() << \"). Press [ENTER] to end the application...\" << endl;
		cout << \"Press [ENTER] to end the application\" << endl;
		cin.get();
		return 0;
	}

	FunctionInterface fi( pDev );

	// send a request to the default request queue of the device and wait for the result.
	fi.imageRequestSingle();
	const int iMaxWaitTime_ms = 8000;   // USB 1.1 on an embedded system needs a large timeout for the first image.
	// wait for results from the default capture queue.
	int requestNr = fi.imageRequestWaitFor( iMaxWaitTime_ms );

	// check if the image has been captured without any problems.
	if( !fi.isRequestNrValid( requestNr ) )
	{
		// If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide
		// additional information under TDMR_ERROR in the interface reference.
		cout << \"*** imageRequestWaitFor failed (\" << requestNr << \")\"
			 << \", timeout value too small?\" << endl;
		return 0;
	}

	const Request* pRequest = fi.getRequest(requestNr);
	if( !pRequest->isOK() )
	{
		cout << \"*** Error: \" << pRequest->requestResult.readS() << endl;
		// if the application wouldn't terminate at this point this buffer HAS TO be unlocked before
		// it can be used again as currently it is under control of the user. However terminating the application
		// will free the resources anyway thus the call
		// fi.imageRequestUnlock( requestNr );
		// can be omitted here.
		return 0;
	}

	cout << \"An image has been captured(\" << pRequest->imageWidth.read() << \"x\" << pRequest->imageHeight.read() << \", Format: \" << pRequest->imagePixelFormat.readS() << \")\" << endl;

	// unlock the buffer to let the driver know that you no longer need this buffer.
	fi.imageRequestUnlock( requestNr );

	cout << \"Press [ENTER] to end the application\" << endl;
	cin.get();

	return 0;
}

" > $SNAME/$SNAME.cpp


	echo " ** ************************************************ **"
	echo " ** The new sample '$SNAME' is now created!"
	echo " ** Please change into the new folder '$SNAME' and execute 'make'!"
	echo " ** You can edit the file '$SNAME.cpp' to make the program you wish!"
	echo " ** For more details please read the documentation!"
	echo " ** ************************************************ **"

#-------------------------------------------------------------------------
else
	echo
	echo " usage: ./mknewappl [sample name]"
	echo
fi
