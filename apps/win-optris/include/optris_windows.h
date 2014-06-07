#ifndef __OPTRIS_WINDOWS_H__
#define __OPTRIS_WINDOWS_H__

#include <iostream>
#include <windows.h>
//#include <opencv2/highgui/highgui.hpp>

#using <System.dll>
#using <System.Drawing.dll>
#using <System.Windows.Forms.dll>

#include "IPC2.h"

#include "improc.hpp"

#define DEFAULT_COLORSCHEME_INDEX	3
#define DEFAULT_COLORSCHEME_CODE	CIELUV

#define DEFAULT_MIN_TEMP			25.0
#define DEFAULT_MAX_TEMP			35.0

#define FORM_OBJECT_OFFSET			-4

ref class optrisManager : public System::Windows::Forms::Form {

protected:
	HRESULT hr;
	IPC^ ipc;
	
	bool ipcInitialized;
	bool Connected;

	int colormap_index;

	bool pauseMode, autoscaleTemps;

	double minTemp, maxTemp;

	System::Windows::Forms::Button ^button;
	System::Windows::Forms::ComboBox ^dropdown;
	System::Windows::Forms::CheckBox ^safety, ^autoscale;
	System::Windows::Forms::TrackBar ^minSlider, ^maxSlider;

	System::DateTime LastFrameTime;

	int FrameCounter0, FrameCounter1, LastFrameCounter, FC0, FC1;

	void button_click(Object^ sender, System::EventArgs^ e);
	void dropdown_changed(Object^ sender, System::EventArgs^ e);
	void safety_changed(Object^ sender, System::EventArgs^ e);
	void autoscale_changed(Object^ sender, System::EventArgs^ e);
	void minslider_changed(Object^ sender, System::EventArgs^ e);
	void maxslider_changed(Object^ sender, System::EventArgs^ e);
	
	short FrameWidth, FrameHeight, FrameDepth;
	double FrameRatio;
	int FrameSize;

	cv::Mat *rawImage, *scaledImage, *_8bitImage, *colorImage;
	cScheme *cMapping;
	
	void InitializeComboBox();
	void Init(int frameWidth, int frameHeight, int frameDepth);
	void addLabel(System::Windows::Forms::Label ^label, int posx, int posy, string text);
	void setupForm();

public:
	optrisManager(HWND hostHandle);
	void initialize();
	void ReleaseIPC();

	HRESULT OnFrameInit(int frameWidth, int frameHeight, int frameDepth);
	HRESULT OnNewFrameEx(void * pBuffer, FrameMetadata *pMetadata);
	HRESULT NewFrame(short *ImgBuf, int frameCounter);
	HRESULT OnInitCompleted(void);

};

#endif // __OPTRIS_WINDOWS_H__