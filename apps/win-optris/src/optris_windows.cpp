#include "optris_windows.h"

optrisManager::optrisManager(HWND hostHandle) {
	ipc = nullptr;
	ipcInitialized = false;
	hr = -1;

	cMapping = new cScheme(DEFAULT_COLORSCHEME_CODE);
	colormap_index = DEFAULT_COLORSCHEME_CODE;

	this->Text = "Win-Optris Control";
	this->Size = System::Drawing::Size(400,300);

	minTemp = DEFAULT_MIN_TEMP; 
	maxTemp = DEFAULT_MAX_TEMP;

	pauseMode = false;
	autoscaleTemps = true;

	setupForm();
	
}

void optrisManager::setupForm() {

	System::Windows::Forms::Label ^dropdown_label, ^safety_label, ^autoscale_label, ^min_slider_label, ^max_slider_label, ^min_temp_label, ^max_temp_label;

	button = gcnew System::Windows::Forms::Button();
	button->Text = "Pause";
	button->Location = System::Drawing::Point( 150, 220 );
	button->Click += gcnew System::EventHandler(this, &optrisManager::button_click);
	this->Controls->Add(button);

	addLabel(dropdown_label, 20, 20, "Color Scheme:");
	dropdown = gcnew System::Windows::Forms::ComboBox();
	InitializeComboBox();
	dropdown->Location = System::Drawing::Point( 120, 20+FORM_OBJECT_OFFSET );
	this->Controls->Add(dropdown);
	dropdown->SelectedIndexChanged += gcnew System::EventHandler(this, &optrisManager::dropdown_changed );
	
	addLabel(safety_label, 20, 50, "Safe Colors:");
	safety = gcnew System::Windows::Forms::CheckBox();
	safety->Location = System::Drawing::Point( 120, 50+FORM_OBJECT_OFFSET );
	this->Controls->Add(safety);
	safety->CheckStateChanged += gcnew System::EventHandler(this, &optrisManager::safety_changed );

	addLabel(autoscale_label, 20, 80, "Autoscale:");
	autoscale = gcnew System::Windows::Forms::CheckBox();
	autoscale->Checked = true;
	autoscale->Location = System::Drawing::Point( 120, 80+FORM_OBJECT_OFFSET );
	this->Controls->Add(autoscale);
	autoscale->CheckStateChanged += gcnew System::EventHandler(this, &optrisManager::autoscale_changed );

	char buff[16];

	addLabel(min_slider_label, 20, 115, "Minimum:");
	sprintf(buff, "%.1f", DEFAULT_MIN_TEMP);
	string minTempString(buff);
	addLabel(min_temp_label, 118, 200, minTempString);
	minSlider = gcnew System::Windows::Forms::TrackBar();
	minSlider->Location = System::Drawing::Point( 120, 115+FORM_OBJECT_OFFSET );
	minSlider->Height = 3;
	minSlider->SetRange(10, 50);
	minSlider->Width = 240;
	minSlider->Value = int(DEFAULT_MIN_TEMP);
	this->Controls->Add(minSlider);
	minSlider->ValueChanged += gcnew System::EventHandler(this, &optrisManager::minslider_changed );
	
	addLabel(max_slider_label, 20, 155, "Maximum:");
	sprintf(buff, "%.1f", DEFAULT_MAX_TEMP);
	string maxTempString(buff);
	addLabel(max_temp_label, 338, 200, maxTempString);
	maxSlider = gcnew System::Windows::Forms::TrackBar();
	maxSlider->SetRange(10, 50);
	maxSlider->Width = 240;
	maxSlider->Location = System::Drawing::Point( 120, 155+FORM_OBJECT_OFFSET );
	maxSlider->Value = int(DEFAULT_MAX_TEMP);
	this->Controls->Add(maxSlider);
	maxSlider->ValueChanged += gcnew System::EventHandler(this, &optrisManager::maxslider_changed );
	
}

void optrisManager::addLabel(System::Windows::Forms::Label ^label, int posx, int posy, string text) {
	
	label = gcnew System::Windows::Forms::Label();
	label->Location = System::Drawing::Point( posx, posy );
	System::String ^x;

	x = gcnew System::String(text.c_str());

	label->Text = x; // (text.c_str());
	this->Controls->Add(label);
}

// http://msdn.microsoft.com/en-us/library/system.windows.forms.comboboxstyle.aspx
void optrisManager::InitializeComboBox() {

	array<System::String^>^ schemes = {
		"Iron",
		"Rainbow",
		"CIEComp",
		"CIELUV",
		"Blackbody",
		"Blue-Red",
		"Jet",
		"Grayscale"
	};

	dropdown->Items->AddRange( schemes );
	dropdown->Location = System::Drawing::Point( 136, 32 );
	dropdown->DropDownStyle = System::Windows::Forms::ComboBoxStyle::DropDownList;
	dropdown->Name = "Color Scheme";
	dropdown->TabIndex = 2;
	dropdown->SelectedIndex = DEFAULT_COLORSCHEME_INDEX;

}

void optrisManager::button_click(Object^ sender, System::EventArgs^ e) {
	pauseMode ? button->Text = "Pause" : button->Text = "Resume";
	pauseMode = !pauseMode;
}

void optrisManager::safety_changed(Object^ sender, System::EventArgs^ e) {
	cMapping->load_standard(colormap_index, safety->Checked ? 1 : 0);
}

void optrisManager::autoscale_changed(Object^ sender, System::EventArgs^ e) {
	autoscale->Checked ? autoscaleTemps = true : autoscaleTemps = false;
	cMapping->load_standard(colormap_index);
}

void optrisManager::minslider_changed(Object^ sender, System::EventArgs^ e) {    
	minTemp = double(minSlider->Value); 
}

void optrisManager::maxslider_changed(Object^ sender, System::EventArgs^ e) {    
	maxTemp = double(maxSlider->Value);
}

void optrisManager::dropdown_changed(Object^ sender, System::EventArgs^ e) {
    
	switch (dropdown->SelectedIndex) {
	case 0:
		colormap_index = IRON;
		break;
	case 1:
		colormap_index = RAINBOW;
		break;
	case 2:
		colormap_index = CIECOMP;
		break;
	case 3:
		colormap_index = CIELUV;
		break;
	case 4:
		colormap_index = BLACKBODY;
		break;
	case 5:
		colormap_index = BLUERED;
		break;
	case 6:
		colormap_index = JET;
		break;
	case 7:
		colormap_index = GRAYSCALE;
		break;
	default:
		colormap_index = GRAYSCALE;
	}
	
	cMapping->load_standard(colormap_index);

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

	printf("%s << Building images with dimensions (%d) x (%d)\n", __FUNCTION__, FrameWidth, FrameHeight);
	rawImage = new cv::Mat(FrameHeight, FrameWidth, CV_16UC1);
	scaledImage = new cv::Mat(FrameHeight, FrameWidth, CV_16UC1);
	_8bitImage = new cv::Mat(FrameHeight, FrameWidth, CV_8UC1);
	colorImage = new cv::Mat(FrameHeight, FrameWidth, CV_8UC3);
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
	
	rawImage->data = (uchar*) ImgBuf;

	if (autoscaleTemps) {
		normalize_16(*scaledImage, *rawImage);
	} else {
		double minLevel = minTemp * 100.0;
		double maxLevel = maxTemp * 100.0;
		normalize_16(*scaledImage, *rawImage, minLevel, maxLevel);
	}
	
	down_level(*_8bitImage, *scaledImage);

	cMapping->falsify_image(*_8bitImage, *colorImage, 0);		// THIS IS SLOWING THINGS DOWN!!
	
	!pauseMode ? cv::imshow("optrisVideo", *colorImage) : 0;
	
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
	oM->initialize();
	System::Windows::Forms::Application::Run(oM);
	oM->ReleaseIPC();

	return S_OK;
}