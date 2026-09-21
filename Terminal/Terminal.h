#ifndef TERMINAL_H
#define TERMINAL_H
#include <Windows.h>

//#define RESET       "\033[0m"
//#define BLACK       "\033[30m"
//#define RED         "\033[31m"
//#define GREEN       "\033[32m"
//#define YELLOW      "\033[33m"
//#define BLUE        "\033[34m"
//#define MAGENTA     "\033[35m"
//#define CYAN        "\033[36m"
//#define WHITE       "\033[37m"


#define BLACK           16  //40
#define RED             12  //41
#define GREEN           10  //42
#define YELLOW          14  //43
#define WHITE           15  //47

#include <iostream>
#include <string>
#include <ini.h>
#include <fmt/core.h>
#include <fmt/color.h>

void Reset_Line();

// Function to set the console text and background color
void SetColor(int textColor);
void Generate_Warning(const bool flag, const std::string str);
void Generate_Error(const bool flag, const std::string str);
void displayLoadingBar(int progress, int total, int barWidth, std::string conclusion = "");
void HideCursor();
void ShowCursor();

class Manage_Ini {
public:
	Manage_Ini(const std::string& path);
	template<typename T>
	T Read(const std::string& section, const std::string& name) {
        // section not found error
        try {
            ini.Get(section);
        }
        catch (const std::runtime_error& e) {
            Generate_Error(0, std::string(e.what()) + "\nSection not found in parameters.ini file");  // Report error and exit
        }

        // key not found error
        try {
            ini.Get<T>(section, name);
        }
        catch (const std::runtime_error& e) {
            Generate_Error(0, std::string(e.what()) + "\nThe key is missing in parameters.ini file");  // Report error and exit
        }

        return ini.Get<T>(section, name);
    }
private:
	inih::INIReader ini;
};

#endif