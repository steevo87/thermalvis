#include "optris_windows.h"

optrisManager::optrisManager(HWND hostHandle) {
	ipc = nullptr;
	ipcInitialized = false;
	hr = -1;
}

HRESULT optrisManager::OnFrameInit(int frameWidth, int frameHeight, int frameDepth) {
	Init(frameWidth, frameHeight, frameDepth);
	return S_OK;
} 

void optrisManager::Init(int frameWidth, int frameHeight, int frameDepth) {
	FrameWidth = frameWidth;
	FrameHeight = frameHeight;
	FrameSize = FrameWidth * FrameHeight;
	FrameRatio = (double)FrameWidth /  (double)FrameHeight;
	FrameDepth = frameDepth;
}

HRESULT optrisManager::OnNewFrameEx(void * pBuffer, FrameMetadata *pMetadata) {
	switch(pMetadata->FlagState)
	{
	case fsFlagOpen: 
		/*printf("%s << open.\n", __FUNCTION__);*/ 
		break;
	case fsFlagClose: 
		/*printf("%s << closed.\n", __FUNCTION__);*/ 
		break;
	case fsFlagOpening: 
		/*printf("%s << opening.\n", __FUNCTION__);*/ 
		break;
	case fsFlagClosing: 
		/*printf("%s << closing.\n", __FUNCTION__);*/ 
		break;
	default: 
		0;
		/*printf("%s << unknown.\n", __FUNCTION__);*/
	}
	return NewFrame((short*)pBuffer, pMetadata->Counter);
}

HRESULT optrisManager::NewFrame(short *ImgBuf, int frameCounter) {
	System::DateTime time = System::DateTime::Now;
	System::TimeSpan ts = time - LastFrameTime;
	LastFrameTime = time;

	FrameCounter0 = frameCounter;
	FrameCounter1++;
	
	cv::Mat frameMat(FrameHeight, FrameWidth, CV_16UC1);

	for (int iii = 0; iii < FrameHeight; iii++) {
		for (int jjj = 0; jjj < FrameWidth; jjj++) {
			frameMat.at<unsigned short>(iii,jjj) = ImgBuf[iii*FrameWidth+jjj];
		}
	}
	
	cv::imshow("optrisVideo", frameMat);
	cv::waitKey(1);

	return 0;
}

void optrisManager::ReleaseIPC() {
	Connected = false;
	if(ipc && ipcInitialized)
	{
		ipc->Release(0);
		ipcInitialized = false;
	}
}


void optrisManager::initialize() {
	ipc = gcnew IPC(1);
	ipc->SetImagerIPCCount(1);

	if(ipc && !ipcInitialized) {
		hr = ipc->Init(0, "");
		
		if(FAILED(hr)) {
			ipcInitialized = false;
		} else {
			ipc->OnFrameIRInit = gcnew IPC::delOnFrameInit(this, &optrisManager::OnFrameInit );
			ipc->SetCallback_OnFrameInit(0, ipc->OnFrameIRInit);

			ipc->OnNewFrameIREx = gcnew IPC::delOnNewFrameEx(this, &optrisManager::OnNewFrameEx );
			ipc->SetCallback_OnNewFrameEx(0, ipc->OnNewFrameIREx);

			ipc->OnInitCompleted = gcnew IPC::delOnInitCompleted(this, &optrisManager::OnInitCompleted );
			ipc->SetCallback_OnInitCompleted(0, ipc->OnInitCompleted);
			
			hr = ipc->Run(0);
			ipcInitialized = SUCCEEDED(hr);

			LastFrameTime = System::DateTime::Now;
		}
	}

}

HRESULT optrisManager::OnInitCompleted() {
	Connected = true;
	return S_OK;
}

[System::STAThreadAttribute]
int main(int argc, char* argv[]) {
	printf("%s << Launching Optris Windows Demo App!\n", __FUNCTION__);
		
	HWND hwnd = 0;
	optrisManager^ oM;

	oM = gcnew optrisManager(hwnd);

	oM->Text = "Win-Optris Control Panel";

	oM->initialize();
	System::Windows::Forms::Application::Run(oM);

	oM->ReleaseIPC();

	return S_OK;
}