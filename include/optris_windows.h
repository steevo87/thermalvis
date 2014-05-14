#ifndef __OPTRIS_WINDOWS_H__
#define __OPTRIS_WINDOWS_H__

#include <iostream>
#include <windows.h>
#include <opencv2/highgui/highgui.hpp>

#using <System.dll>
#using <System.Drawing.dll>
#using <System.Windows.Forms.dll>

#include "IPC2.h"

ref class optrisManager : public System::Windows::Forms::Form {

protected:
	HRESULT hr;
	IPC^ ipc;
	
	bool ipcInitialized;
	bool Connected;

	System::DateTime LastFrameTime;

	int FrameCounter0, FrameCounter1, LastFrameCounter, FC0, FC1;
	
	short FrameWidth, FrameHeight, FrameDepth;
	double FrameRatio;
	int FrameSize;
	
	void Init(int frameWidth, int frameHeight, int frameDepth);

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