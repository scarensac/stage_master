// Evolution_program.cpp : définit le point d'entrée pour l'application console.
//
#define STATIC_BUILD

#include "stdafx.h"
#include "Evolution_program.h"
#include <iostream>
#include <iomanip>
#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */
#include <sstream>
#include "UI.h"
#include "tcltk.h"
#include <fstream>
#include <clocale>
#include "Core\SimGlobals.h"

#include <conio.h>//for kbhit

int _tmain(int argc, _TCHAR* argv[])
{
	std::setlocale(LC_ALL, "en_US.UTF-8");

	SetWindow(800, 300);

	Globals::init_folder_path = "../AppGUI/init/";

	//try to launch the normal program
	//*
	if (argc > 1){
		Globals::evolution_mode = 1;
		Globals::animationRunning = 1;
		SimGlobals::liquid_density = 1000;
		SimGlobals::water_level = wcstof(argv[1], NULL);
		launch_simulation(false);
	}
	else{
		SimGlobals::water_level = 0;
		do{
			SimGlobals::water_level += 0.1;
			cma_program();
		} while (SimGlobals::water_level<1.05);

		
		//system("..\\Binaries\\Release\\Evolution_program.exe anything");
	}
	//*/


	return 0;
}



// Implementation of the CMA-ES
 #include <shark/Algorithms/DirectSearch/CMA.h>
 // Access to benchmark functions
 #include <shark/ObjectiveFunctions/Benchmarks/Benchmarks.h>
//acces to sphere
#include <shark/ObjectiveFunctions/Benchmarks/Sphere.h>
#include <sstream>
#include "Core\SimBiConFramework.h"


int cma_program() {

	 // Adjust the floating-point format to scientific and increase output precision.
	std::cout.setf(std::ios_base::scientific);
	std::cout.precision(10);
	//*
	

	//we create the objective function
	SimbiconOnjectiveFunction objective_func;
	std::ostringstream oss;
	oss << "..\\Binaries\\Release\\Evolution_program.exe " << SimGlobals::water_level;
	objective_func.exe_line = oss.str();

	// Initialize the optimizer for the objective function instance.
	shark::CMA cma;
	cma.init(objective_func);//the parameter here is the error function
	cma.setSigma(0.02); // Explicitely set initial globael step size.

	int nbr_iter = 0;
	// Iterate the optimizer until a solution of sufficient quality is found.
	do {
		//evolve the parameters
		cma.step(objective_func);

		
		// Report information on the optimizer state and the current solution to the console.
		std::cout <<nbr_iter <<" :: "<<objective_func.evaluationCounter() << " :: " << cma.solution().value <<  " :: " << cma.sigma() << std::endl;

		nbr_iter++;
		if (nbr_iter > 100){break;}
	} while (cma.solution().value > 1E-20);
	//*/

	//we save the last solution in a file 
	shark::RealVector result = cma.solution().point;

	//now I need to save the parameters back in the file so that the next eecution will see them
	SimBiConFramework* con = new SimBiConFramework(objective_func.inputFile, NULL);

	//so we push the values in a structure
	int cur_pos = 0;
	for (int k = 0; k < (int)objective_func.vect_traj_name.size(); ++k){
		Trajectory* cur_traj = con->getController()->getState(0)->getTrajectory(objective_func.vect_traj_name[k].c_str());
		for (int j = 0; j < (int)cur_traj->components.size(); ++j){
			TrajectoryComponent* traj_comp = cur_traj->components[j];
			for (int i = 0; i < (int)traj_comp->baseTraj.getKnotCount(); ++i){
				traj_comp->baseTraj.setKnotValue(i, result[cur_pos]);
				++cur_pos;
			}
		}
	}

	//we switch the save folder
	std::ofstream myfile1("../data/controllers/bipV2/learning_files_names.txt");
	if (myfile1.is_open())
	{
		myfile1 << "water_lvl_evo/"<<"learning_walk_waterlvl" << SimGlobals::water_level<<".rs" << std::endl;
		myfile1 << "water_lvl_evo/" << "learning_walk_waterlvl" << SimGlobals::water_level << ".sbc" << std::endl;
	}
	myfile1.close();
	
	//and we save the structure
	con->save(true, false);

	//we restore the save folder for the evolution
	std::ofstream myfile2("../data/controllers/bipV2/learning_files_names.txt");
	if (myfile2.is_open())
	{
		myfile2 << "learning_walk_state.rs" << std::endl;
		myfile2 << "learning_walk.sbc" << std::endl;
	}
	myfile2.close();



	int a = 2;
	std::cin >> a;

	return 0;
}

#include "AppGUI\UI.h"

//this function launch the simulation
void launch_simulation(bool use_interface){
	char *argv2[] = { "Framework" };
	int argc2 = 1;
	
	//to parameter how we evaluate the solution
	SimGlobals::steps_before_evaluation = 20;
	SimGlobals::nbr_evaluation_steps= 10;
	Globals::useConsole = false;

	Globals::use_interface = use_interface;

	try{
		Tk_Main(argc2, argv2, Tcl_AppInit);
	}
	catch (char* msg){
		logPrint("Exception: %s\n", msg);
		std::cout << "press any key to continue the execution" << std::endl;
		kbhit();
		exit(-1);
	}
	catch (std::exception& e){
		std::cout << e.what() << std::endl;
		std::cout << "press any key to continue the execution" << std::endl;
		kbhit();
		exit(-1);
	}
}

SimbiconOnjectiveFunction::SimbiconOnjectiveFunction(){
	m_features |= HAS_VALUE;
	m_features |= CAN_PROPOSE_STARTING_POINT;

	//first we will need to load
	std::ostringstream oss;
	oss << Globals::init_folder_path;
	oss << "input.conF";

	inputFile = new char[100];
	strcpy(inputFile, oss.str().c_str());
	SimBiConFramework* con = new SimBiConFramework(inputFile, NULL);


	//I store the names
	vect_traj_name.push_back("root");
	vect_traj_name.push_back("STANCE_Knee");
	vect_traj_name.push_back("SWING_Ankle");
	vect_traj_name.push_back("STANCE_Ankle");
	vect_traj_name.push_back("swing_foot");


	//this should load all the concerned trajectories
	for (int k = 0; k < (int)vect_traj_name.size(); ++k){
		Trajectory* cur_traj = con->getController()->getState(0)->getTrajectory(vect_traj_name[k].c_str());
		for (int j = 0; j < (int)cur_traj->components.size(); ++j){
			TrajectoryComponent* traj_comp = cur_traj->components[j];
			for (int i = 0; i < (int)traj_comp->baseTraj.getKnotCount(); ++i){
				variable_vector.push_back(traj_comp->baseTraj.getKnotValue(i));
			}
		}
	}


	//and delete the structure
	delete con;



	//we set the dimentionality
	setNumberOfVariables(variable_vector.size());

}


SimbiconOnjectiveFunction::ResultType SimbiconOnjectiveFunction::eval(const SearchPointType & input)const {
	SIZE_CHECK(input.size() == m_dimensions);

	static shark::RealVector last_vect;
	static double last_eval = -1;

	if (last_eval != -1){
		bool is_same = true;
		for (int i = 0; i < (int)input.size(); ++i){
			if (input[i] != last_vect[i]){
				is_same = false;
				break;
			}
		}
		//is we reach this point it means the point are the same so just cut the sim
		if (is_same){
			return last_eval;
		}
	}

	last_vect = input;

	//now I need to save the parameters back in the file so that the next eecution will see them
	SimBiConFramework* con = new SimBiConFramework(inputFile, NULL);

	//so we push the values in a structure
	int cur_pos = 0;
	for (int k = 0; k < (int)vect_traj_name.size(); ++k){
		Trajectory* cur_traj = con->getController()->getState(0)->getTrajectory(vect_traj_name[k].c_str());
		for (int j = 0; j < (int)cur_traj->components.size(); ++j){
			TrajectoryComponent* traj_comp = cur_traj->components[j];
			for (int i = 0; i < (int)traj_comp->baseTraj.getKnotCount(); ++i){
				traj_comp->baseTraj.setKnotValue(i, input[cur_pos]);
				++cur_pos;
			}
		}
	}

	//and we save the structure
	con->save(true, false);

	//clean the memory
	delete con;


	//test the simulation
	int return_val = system(exe_line.c_str());

	//read the result
	std::ifstream myfile;
	myfile.open("eval_result.txt");
	myfile >> std::fixed >> std::setprecision(8) >> last_eval;
	myfile.close();


	return last_eval;
}

