#include "stdafx.h"
#include <windows.h>

int _tmain(int argc, _TCHAR* argv[]);

//first example containing the tutorial for cma
int cma_program();

//this function launch the simulation
void launch_simulation(bool use_tk_interface=true, bool use_gl_interface=true);

//hello world tutorial
void hello_world();

//regression tuto
void regression_tuto();

#include <shark/ObjectiveFunctions/AbstractObjectiveFunction.h>
#include <boost/scoped_ptr.hpp>

class SimbiconOnjectiveFunction : public shark::SingleObjectiveFunction
{
public:

	SimbiconOnjectiveFunction();

	void proposeStartingPoint(SearchPointType & startingPoint)const {
		//and we set the starting point
		startingPoint=variable_vector;
	}

    std::string name() const { return "SimbiconOnjectiveFunction"; }
	
	std::size_t numberOfVariables()const{
		return m_dimensions;
	}
	bool hasScalableDimensionality()const{
		return true;
	}
	void setNumberOfVariables(std::size_t numberOfVariables){
		m_dimensions = numberOfVariables;
	}

	ResultType eval(const SearchPointType & input)const;

private:
	std::size_t m_dimensions;

public:
	//I'll store the concerned straj name in a vector for an easier use
	std::vector<std::string> vect_traj_name;

	//just nothing realy
	char* inputFile;

	//this contain the starting point
	shark::RealVector variable_vector;

	//this is the line I execute with system
	std::string exe_line;
};


#include <windows.h>

void SetWindow(int Width, int Height)
{
	_COORD coord;
	coord.X = Width;
	coord.Y = Height;

	_SMALL_RECT Rect;
	Rect.Top = 0;
	Rect.Left = 0;
	Rect.Bottom = Height - 1;
	Rect.Right = Width - 1;

	HANDLE Handle = GetStdHandle(STD_OUTPUT_HANDLE);      // Get Handle
	SetConsoleScreenBufferSize(Handle, coord);            // Set Buffer Size
	SetConsoleWindowInfo(Handle, TRUE, &Rect);            // Set Window Size
}

int execute_line(std::string line);

std::string get_data_folder_path(int lookup_nbr);

//string convert functions
std::wstring s2ws(const std::string& str);
std::string ws2s(const std::wstring& wstr);