#include "Terminal.h"

// Function to set the console text and background color
void SetColor(int textColor)
{
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(hConsole, textColor);
}

void Reset_Line() {
    // Ottenere l'handle della console
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

    // Ottenere le informazioni correnti della console, inclusa la posizione del cursore
    CONSOLE_SCREEN_BUFFER_INFO csbi;
    GetConsoleScreenBufferInfo(hConsole, &csbi);

    // Calcolare la lunghezza della riga corrente
    DWORD length = csbi.dwSize.X;
    COORD cursorPosition = csbi.dwCursorPosition;

    // Spostare il cursore all'inizio della riga
    cursorPosition.X = 0;
    SetConsoleCursorPosition(hConsole, cursorPosition);

    // Riempire la riga con spazi per cancellarla
    DWORD written;
    FillConsoleOutputCharacter(hConsole, ' ', length, cursorPosition, &written);

    // Spostare nuovamente il cursore all'inizio della riga
    SetConsoleCursorPosition(hConsole, cursorPosition);
}

void displayLoadingBar(int progress, int total, int barWidth, std::string conclusion) {
    float ratio = static_cast<float>(progress) / total;
    int pos = barWidth * ratio;
    HideCursor();

    std::cout << "[";
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "*";
        else std::cout << " ";
        //else if (i == pos) std::cout << ">";
    }
    std::cout << "] " << int(ratio * 100.0) << "%\r";
    std::cout.flush();

    if (ratio == 1) {
        ShowCursor();
        Sleep(500);
        //std::cout << "\33[2K\r";
        Reset_Line();
        if (conclusion != "") {
            SetColor(GREEN);
            std::cout << conclusion << std::endl;
            SetColor(WHITE);
        }
    }
    
}

// Generate_Error
//*****************************************************************************************************************************************
void Generate_Error(const bool flag, const std::string str) {
    if (flag) {
        Reset_Line();
        SetColor(RED);
        std::cerr << "Error: " <<  str << std::endl;
        std::cin.get();
        exit(EXIT_FAILURE);
    }
}

// Generate_Warning
//*****************************************************************************************************************************************
void Generate_Warning(const bool flag, const std::string str) {
    if (flag) {
        Reset_Line();
        SetColor(YELLOW);
        std::cerr << "Warning: " << str << std::endl;
        SetColor(WHITE);
    }
}

Manage_Ini::Manage_Ini(const std::string& path) {
    try {
        ini = inih::INIReader(path);
    }
    catch (const std::exception& e) {
        Generate_Error(false, "Unable to load INI file: " + path + ". Exception: " + e.what());
    }
}

// Function to hide the cursor
void HideCursor() {
    HANDLE consoleHandle = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_CURSOR_INFO cursorInfo;

    // Get the current cursor settings
    GetConsoleCursorInfo(consoleHandle, &cursorInfo);

    // Hide the cursor
    cursorInfo.bVisible = FALSE;

    // Apply the new cursor settings
    SetConsoleCursorInfo(consoleHandle, &cursorInfo);
}

// Function to show the cursor
void ShowCursor() {
    HANDLE consoleHandle = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_CURSOR_INFO cursorInfo;

    // Get the current cursor settings
    GetConsoleCursorInfo(consoleHandle, &cursorInfo);

    // Show the cursor
    cursorInfo.bVisible = TRUE;

    // Apply the new cursor settings
    SetConsoleCursorInfo(consoleHandle, &cursorInfo);
}