#include "optris_windows.h"

[System::STAThreadAttribute]
int main(int argc, char* argv[]) {

	displayMessage("Launching Optris Windows Demo App!", MESSAGE_NORMAL, __FUNCTION__);
	displayMessage("Note: if no image appears, launch the PI Connect software and make sure IPC is enabled (see README.md)", MESSAGE_NORMAL, __FUNCTION__);
		
	HWND hwnd = 0;
	optrisManager^ oM;

	oM = gcnew optrisManager(hwnd);
	oM->initialize();
	oM->opStream->initializeOutput(argc, argv);
	oM->opStream->setWriteMode(!(argc >= 4)); 

	System::Windows::Forms::Application::Run(oM);
	oM->ReleaseIPC();

	return S_OK;
}