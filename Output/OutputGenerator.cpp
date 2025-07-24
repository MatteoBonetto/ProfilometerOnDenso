#include "OutputGenerator.h"

_OutputGenerator_::_OutputGenerator_(double sampling, double total_profile, double total_points, int num) {
	sampling_ = sampling;
	total_profile_ = total_profile;
	total_points_ = total_points;

	current_path_ = std::filesystem::current_path(); // Get the current directory
	output_folder_ = current_path_ / "Data";
	// Ensure the directory exists
	std::filesystem::create_directories(output_folder_);
	// Define the file path
	std::filesystem::path file_path = output_folder_ / (std::to_string(num) + ".txt");
	file_.open(file_path);

	// HEADER
	if (!file_) {
		std::cerr << "Failed to create file: " << file_path << std::endl;
	}
	else {
		std::cout << "created file: " << file_path << std::endl;
		file_ << "Cloud Points Information\n";
		file_ << "------------------------\n";
		file_ << "File Name:\t" << file_path << "\n";
		file_ << "Creation Date:\t" << GetCurrentDateTime() << "\n";
		file_ << "Encoder Resolution [deg/Tick]:\t" << 1 << "\n";
		file_ << "Sampling [deg]:\t" << std::abs(sampling_) << "\n";
		file_ << "Total Profiles:\t" << total_profile_ << "\n";
		file_ << "Total Points:\t" << total_points_ * total_profile_ << "\n";
		file_ << "Data Format:\tX [mm]\tZ [mm]\tEncoder_Value [Tick]\n";
		file_ << "------------------------\n\n";
	}
}

// *******************************************************************************************************
std::string _OutputGenerator_::GetCurrentDateTime() {
	std::time_t now = std::time(nullptr);
	std::tm* local_time = std::localtime(&now);
	std::ostringstream oss;
	oss << std::put_time(local_time, "%Y-%m-%d %H:%M:%S");
	return oss.str();
}

// *******************************************************************************************************
void _OutputGenerator_::AddLine(std::vector<ProfilePoint> profile, double tick, Eigen::Matrix4d transformationMatrix, bool intensity) {
	if (file_.is_open()) {
		for (int i = 0; i < total_points_; i++) {
			Eigen::Vector4d point_transformed = TransformPoint(profile[i], transformationMatrix);
			if (intensity)
				file_ << point_transformed.x() << "\t" << point_transformed.z() << "\t" << tick << "\t" << (unsigned short) profile[i].intensity << std::endl;
			else
				file_ << point_transformed.x() << "\t" << point_transformed.z() << "\t" << tick << std::endl;
		}
	}
	else {
		std::cerr << "File is not open!" << std::endl;
	}
}

// *******************************************************************************************************
void _OutputGenerator_::CloseFile() {
	file_.close();
}

// *******************************************************************************************************
Eigen::Vector4d _OutputGenerator_::TransformPoint(ProfilePoint pointStruct, Eigen::Matrix4d transformationMatrix) {
	// Define a 3D point (x, y, z) as a 4D homogeneous vector (x, y, z, 1)
	Eigen::Vector4d point(pointStruct.x, 0, pointStruct.z, 1.0); // (x, y, z, 1)

	// Apply the transformation
	Eigen::Vector4d transformed_point = transformationMatrix * point;

	// Print results
	//std::cout << "Original Point:\t " << point(0) << "\t\t" << point(1) << "\t\t" << point(2) << "\t\t" << point(3) << "\t\t" << std::endl;
	//std::cout << "transformed Point:\t " << transformed_point(0) << "\t\t" << transformed_point(1) << "\t\t" << transformed_point(2) << "\t\t" << transformed_point(3) << "\t\t" << std::endl << std::endl;

	return transformed_point;
}