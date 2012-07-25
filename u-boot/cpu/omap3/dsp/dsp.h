/*
 * (C) Copyright 2008 - 2009 Texas Instruments.
 *
 * (C) Copyright 2010
 * MM solutions, <www.mm-sol.com>
 * Nikolay Nikolov, <nnikolov@mm-sol.com>
 *
 * Interface functions for fast splash screen drawing by means of the DSP.
 *
 */

#ifndef DSP_SPLASH_H
#define DSP_SPLASH_H

/*******************************************************************************
 *		NOTE 1: The image file should be one of the formats:
 *			1) Special 1-bit per pixel binary B&W format.
 *			2) PGM file format.
 ******************************************************************************/

/*******************************************************************************
 * Function name:	dsp_start()
 * Purpose:			Load configuration data and code for the DSP and start it.
 * Inputs:
 *	- init_img_fname:	Pointer to the filename of an image file that serves
 *		to show the DSP the old state of the screen. Provide NULL pointer
 *		if last displayed image file is unknown or if you require the DSP to
 *		clear the screen. See NOTE 1 above for the file formats supported.
 * Outputs:
 *	- (0):	Indicates DSP was started successfuly.
 *	- (1):	Indicates failure to start the DSP.
 ******************************************************************************/
int dsp_start (const char *init_img_fname);

/*******************************************************************************
 * Function name:	dsp_stop()
 * Purpose:			Command the DSP to stop and turn it off.
 * Inputs:
 *	- force_stop	Set to 0 to stop the DSP only if it has completed last job.
 *		Set to 1 to wait until DSP completes last job an then stop it.
 * Outputs:
 *	- (0):	Indicates DSP was stopped successfuly.
 *	- (1):	Indicates DSP was not stopped. This will only happen if has not
 *		 completed previous job and force_stop = 0.
 ******************************************************************************/
int dsp_stop(const int force_stop);

// Display image functions return values
typedef enum {
	DISP_ERR_OK	= 0,				// Image display completed successfuly
	DISP_ERR_NOTSTARTED		= 1,	// Image display not started at all. Maybe last successfuly displayed image is on the display !
	DISP_ERR_NOTCOMPLETED	= 2,	// Image display started but not completed (within timeout). Display state is unknown !
	DISP_ERR_unknown	// Jus placeholder
} DISP_ERR_t;
// Waveform IDs valid values
typedef enum {	// Keep track between these and DSP internals.
	WVFID_GC	= 0,
	WVFID_GU	= 1,
	WVFID_DU	= 2,
	WVFID_A2	= 3,
	WVFID_CUST1	= 4,
	WVFID_CUST2	= 5,
	WVFID_CUST3	= 6,
	WVFID_AUTO	= 255
} WVFID_t;
/*******************************************************************************
 * Functions name:	dsp_display_pic_file()
 *					dsp_display_pic_file_by_wvfid()
 * Purpose:			Command the DSP to stop and turn it off.
 * Inputs:
 *	- fname:	Filename of the image file to be shown on the display.
 *		See NOTE 1 above for the file formats supported.
 *	- timeout_ms:	A timeout in ms. If operation not completed whitin it
 *		operation is reported timed out.
 *	- wvfid:	Waveform ID. See WVFID_t for
 * Outputs:		See definition and comments of the DISP_ERR_t above.
 ******************************************************************************/
DISP_ERR_t dsp_display_pic_file(const char * fname, const int timeout_ms);
DISP_ERR_t dsp_display_pic_file_by_wvfid(const char * fname, const int timeout_ms, const WVFID_t wvfid);

/*******************************************************************************
 * Function name:	is_dsp_on()
 * Purpose:			Indicates state of dsp.
 * Inputs:
 *	- none
 * Outputs:
 *	- (0):	Indicates DSP is off.
 *	- (1):	Indicates DSP is on.
 ******************************************************************************/
int is_dsp_on(void);
#endif
