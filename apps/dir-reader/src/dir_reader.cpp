#include "directory_stream.hpp"

int main(int argc, char* argv[]) {
	
	//displayMessage("Launching Directory Reader Demo App!", MESSAGE_NORMAL, __FUNCTION__);

	bool wantsToAutoscale = true;

	directoryManager dM;

	char output_directory[256];
	bool wantsToOutput = false;

	if (argc >= 3) {
		printf("%s << Using data output directory of <%s>.\n", __FUNCTION__, argv[2]);
		wantsToOutput = true;
		sprintf(output_directory, "%s", argv[2]);
	}

	if (!dM.initializeInput(argc, argv)) return -1;
	dM.initializeOutput(output_directory);
	dM.initialize();
	dM.setWriteMode(!(argc >= 4));
	dM.setLoopMode(true);
	dM.set_autoscaleTemps(wantsToAutoscale);
	
	while (dM.wantsToRun()) {
		dM.grabFrame();
		dM.processFrame();
		dM.colorizeFrame();
		dM.displayFrame();
		dM.writeImageToDisk();
	}
	
	return S_OK;
}
