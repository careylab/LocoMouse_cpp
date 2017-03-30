#include "LocoMouse_ParseInputs.hpp"

void LocoMouse_ParseInputs::configurePath() {

#ifdef _WIN32
	//Needed for path manipulation on windows 
	char path_buffer[_MAX_PATH];
	char drive[_MAX_DRIVE];
	char dir[_MAX_DIR];
	char fname[_MAX_FNAME];
	char ext[_MAX_EXT];
	char drive_out[_MAX_DRIVE];
	char dir_out[_MAX_DIR];

	_splitpath_s(LM_CALL.c_str(), drive, dir, fname, ext);
	_makepath_s(path_buffer, drive, dir, "", "");
	REF_PATH = std::string(path_buffer);

	OUTPUT_PATH += "\\";

	_splitpath_s(VIDEO_FILE.c_str(), drive_out, dir_out, fname, ext);

	FILE_NAME = fname;

	//FIXME: These are created by the constructors.
	//output_file = OUTPUT_PATH + "output_" + std::string(fname) + ".yml";
	//debug_file = OUTPUT_PATH + "debug_" + std::string(fname) + ".yml";
	//debug_text = OUTPUT_PATH + "debug_" + std::string(fname) + ".txt";

#elif __linux__
	//FIXME: Does this even compile anymore?
	OUTPUT_PATH += "/";
	REF_PATH = std::string(dirname(LM_CALL.c_str()));
	FILE_NAME = output_file_name(video);
	//output_file = OUTPUT_PATH + "/output_" + output_file_name(video) + "yml";
	//debug_file = OUTPUT_PATH + "/debug_" + output_file_name(video) + "yml";
	

#endif

}


LocoMouse_ParseInputs::LocoMouse_ParseInputs(int argc, char* argv[]) {
	
	if (argc != 9) {

		std::cout << "Warning: Invalid input list. The input should be: LocoMouse method config.yml video.avi background.png model_file.yml calibration_file.yml side_char output_folder." << std::endl;
		std::cout << "Attempting to run with default paramters..." << std::endl;

		LM_CALL = std::string(argv[0]);
		VIDEO_FILE = "temp.avi";
		
		configurePath();

		CONFIG_FILE = REF_PATH + "config.yml";
		VIDEO_FILE = REF_PATH + "L7Y9_control1_L.avi";
		BKG_FILE = REF_PATH + "L7Y9_control1_L.png";
		MODEL_FILE = REF_PATH + "model_LocoMouse_paper.yml";
		CALIBRATION_FILE = REF_PATH + "IDX_pen_correct_fields2.yml";
		FLIP_CHAR = "L";
		OUTPUT_PATH = REF_PATH + ".";
		METHOD = "0";

	}
	else {
		LM_CALL = std::string(argv[0]);
		VIDEO_FILE = std::string(argv[3]);

		CONFIG_FILE = std::string(argv[2]);
		configurePath();
		
		MODEL_FILE = std::string(argv[5]);
		BKG_FILE = std::string(argv[4]);
		CALIBRATION_FILE = std::string(argv[6]);
		FLIP_CHAR = std::string(argv[7]);
		OUTPUT_PATH = std::string(argv[8]);
		METHOD = std::string(argv[1]);
	
	}
}