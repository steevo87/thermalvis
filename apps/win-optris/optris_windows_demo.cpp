#include "optris_windows.h"

[System::STAThreadAttribute]
int main(int argc, char* argv[]) {

	ROS_INFO("Launching Optris Windows Demo App!");
	ROS_INFO("Note: if no image appears, launch the PI Connect software and make sure IPC is enabled (see README.md)");
		
	HWND hwnd = 0;
	optrisManager^ oM;

	oM = gcnew optrisManager(hwnd);
	oM->initialize();

	char output_directory[256];
	bool wantsToOutput = false;

	if (argc >= 3) {
		printf("%s << Using data output directory of <%s>.\n", __FUNCTION__, argv[2]);
		wantsToOutput = true;
		sprintf(output_directory, "%s", argv[2]);
	}

	oM->opStream->initializeOutput(output_directory);
	oM->opStream->setWriteMode(!(argc >= 4)); 

	System::Windows::Forms::Application::Run(oM);
	oM->ReleaseIPC();

	return S_OK;
}