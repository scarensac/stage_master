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
#include <typeinfo>
#include <windows.h>

#include <conio.h>//for kbhit

const std::string cur_evo_name = "speed_07__361";


int _tmain(int argc, _TCHAR* argv[])
{
	std::setlocale(LC_ALL, "en_US.UTF-8");

	SetWindow(800, 300);



	//the first thing I do is to look for the configuration_data folder
	std::ostringstream oss;

	Globals::data_folder_path = get_folder_path("configuration_data", 5);
	Globals::binaries_folder_path = get_folder_path("Binaries", 5, "\\");

	oss << Globals::data_folder_path;
	oss << "init/";
	Globals::init_folder_path = oss.str();

	//try to launch the normal program
	//*
	if (argc > 1){
		
		Globals::evolution_mode = 1;
		SimGlobals::steps_before_evaluation = 10;
		SimGlobals::nbr_evaluation_steps = 5;
		Globals::animationRunning = 1;
		SimGlobals::liquid_density = 1000;
		SimGlobals::water_level = wcstof(argv[1], NULL);
		SimGlobals::velDSagittal = 0.7;

		
		for (int i = 2; i < argc; ++i){
			std::string  cur_arg= ws2s(std::wstring(argv[i]));
			if (cur_arg == "save"){

				SimGlobals::steps_before_evaluation = 50;
				Globals::save_mode = true;
				Globals::primary_save_config = "controllers/bipV2/primary_save_config.txt";
				Globals::secondary_save_config = "controllers/bipV2/learning_files_names.txt";
				if (argc >= i){
					std::string  save_controller = ws2s(std::wstring(argv[i+1]));
					Globals::save_mode_controller = save_controller;
				}
				break;
			}
		}

		Globals::ipm_alteration_cost = 0.1;
		
		
		launch_simulation(false,false);
	}
	else{
		//*

		SimGlobals::water_level = 0;
		SimGlobals::velDSagittal = 0.7;


		std::string primary_save_config = "controllers/bipV2/primary_save_config.txt";
		std::string secondary_save_config = "controllers/bipV2/learning_files_names.txt";

		update_saving_config(primary_save_config, secondary_save_config);


		//I don't want the secondary save in that program (just needed to fill that file for later use)
		//so I disactivate the secondary save
		Globals::primary_save_config = secondary_save_config;
		Globals::secondary_save_config = primary_save_config;

		//*
		std::ostringstream oss;
		oss << "bipV2/" << cur_evo_name << "/learning_walk_waterlvl" << SimGlobals::water_level << ".sbc";
		//Globals::save_mode_controller = oss.str();
		Globals::save_mode = true;
		Globals::useShader = false;
		//Globals::evolution_mode = 1;
		SimGlobals::steps_before_evaluation = 10;
		SimGlobals::nbr_evaluation_steps = 5;
		//Globals::animationRunning = 1;
		SimGlobals::liquid_density = 1000;
		launch_simulation(true,true);
		//*/

		/*
		SimGlobals::water_level = 0;

		double eval_sum = 0;
		int nbr_eval = 0;
		
		do{
			update_saving_config(primary_save_config, secondary_save_config);

			std::ostringstream oss;
			oss << Globals::binaries_folder_path;
			oss << "Release\\Evolution_program.exe " << SimGlobals::water_level;
			oss << " save bipV2/" << cur_evo_name << "/learning_walk_waterlvl" << SimGlobals::water_level << ".sbc";
			std::string save_line = oss.str();
			int result = execute_line(save_line);
			
			//read the result
			//the second return value is the one when that fucking BS of glut return a retarded exception when I clos ethe program
			if (result == 0 || result == -1073740771){
				std::cout << std::defaultfloat << "finishing save for water_lvl:" << SimGlobals::water_level << std::endl;
				double last_eval;
				std::ifstream myfile;
				myfile.open("eval_result.txt");
				myfile >> std::scientific  >> last_eval;
				myfile.close();
				std::cout << std::scientific << "evaluation result:" << last_eval << std::endl;
				if (last_eval < 10E12){
					nbr_eval++;
					eval_sum += last_eval;
				}
			}
			
			
			
			SimGlobals::water_level += 0.05;
		} while (SimGlobals::water_level<1.11);

		//now we write the avg eval
		std::cout << std::scientific << std::setprecision(8) << "average evaluation:" << eval_sum / (double)nbr_eval<<std::endl;

		//*/
		
		/*
		SimGlobals::water_level = 0.25;
		double water_limit = 0.6;
		do{
			std::cout << "starting evolution for water_lvl:" << SimGlobals::water_level << std::endl;
			cma_program(cur_evo_name);
			SimGlobals::water_level += 0.25;
		} while (SimGlobals::water_level<water_limit);
		//*/

		//
		std::cout << "execution successfully finished"<< std::endl;
		system("pause");

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



int cma_program(std::string save_folder_name) {

	// Adjust the floating-point format to scientific and increase output precision.
	std::cout.setf(std::ios_base::scientific);
	std::cout.precision(10);
	//*


	//we create the objective function
	SimbiconOnjectiveFunction objective_func;
	std::ostringstream oss;
	oss << Globals::binaries_folder_path;
	oss << "Release\\Evolution_program.exe " << SimGlobals::water_level;
	objective_func.exe_line = oss.str();
	oss << " save bipV2/" << save_folder_name << "/learning_walk_waterlvl" << SimGlobals::water_level << ".sbc";
	std::string save_line = oss.str();


	//we set the save configurations
	//we switch the save folder
	std::string primary_save_config = "controllers/bipV2/primary_save_config.txt";
	std::string secondary_save_config = "controllers/bipV2/learning_files_names.txt";


	oss.clear();
	oss.str("");
	oss << Globals::data_folder_path;
	oss << primary_save_config;

	std::ofstream myfile1(oss.str());
	if (myfile1.is_open())
	{
		myfile1 << save_folder_name << "/learning_walk_waterlvl" << SimGlobals::water_level << "_state.rs" << std::endl;
		myfile1 << save_folder_name << "/learning_walk_waterlvl" << SimGlobals::water_level << ".sbc" << std::endl;
	}
	myfile1.close();

	//*
	std::stringstream oss2;
	oss2 << Globals::data_folder_path;
	oss2 << secondary_save_config;
	std::ofstream myfile2(oss2.str());
	if (myfile2.is_open())
	{
		myfile2 << "learning_walk_state.rs" << std::endl;
		myfile2 << "learning_walk.sbc" << std::endl;
	}
	myfile2.close();
	//I don't want the secondary save in that program (just needed to fill that file for later use)
	//so I disactivate the secondary save
	Globals::primary_save_config = secondary_save_config;
	Globals::secondary_save_config = "";
	//*/



	// Initialize the optimizer for the objective function instance.
	shark::CMA cma;
	cma.init(objective_func);//the parameter here is the error function
	cma.setSigma(0.04); // Explicitely set initial globael step size.

	int nbr_iter = 0;
	// Iterate the optimizer until a solution of sufficient quality is found.

	



	double cur_val;
	bool first_time = true;
	int cur_save_trigger = 10;
	do {

		nbr_iter++;
		if (nbr_iter > 60){ break; }

		//this is used to update the starting pos (so it fit the movement better)
		if (nbr_iter >= cur_save_trigger){
			cur_save_trigger += 10;
			execute_line(save_line);
		}


		//evolve the parameters
		cma.step(objective_func);


		// Report information on the optimizer state and the current solution to the console.
		std::cout << nbr_iter << " :: " << objective_func.evaluationCounter() << " :: " << cma.solution().value << " :: " << cma.sigma() << std::endl;

		if (first_time){
			first_time = false;
		}
		else{
			if (cur_val < cma.solution().value){
				//if our new solution ain't better we don't save if (but still continue from it)
				continue;
			}
		}

		//save the current best solution
		cur_val = cma.solution().value;

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

		//I I detect the ipm alte I add it
		if (cur_pos < result.size()){
			SimGlobals::ipm_alteration_effectiveness = result[cur_pos];
			//limit it to 1 (since I want it to go down )
			if (SimGlobals::ipm_alteration_effectiveness > 1){
				SimGlobals::ipm_alteration_effectiveness = 1;
			}
			++cur_pos;
		}

		//*
		//also I'll prevent the system from doing the ondulations with the pelvis
		Trajectory* cur_traj = con->getController()->getState(0)->getTrajectory(objective_func.vect_traj_name[0].c_str());
		for (int j = 0; j < (int)cur_traj->components.size(); ++j){
			TrajectoryComponent* traj_comp = cur_traj->components[j];
			for (int i = 0; i < (int)traj_comp->baseTraj.getKnotCount(); ++i){
				traj_comp->baseTraj.setKnotValue(i, std::fmin(0.1, traj_comp->baseTraj.getKnotValue(i)));
			}
		}
		//*/
		//*
		//I only wanna learn the x component of the walk so I'l override the others with the value I had
		cur_traj = con->getController()->getState(0)->getTrajectory(objective_func.vect_traj_name[5].c_str());
		for (int j = 0; j < (int)cur_traj->components.size(); ++j){
			TrajectoryComponent* traj_comp = cur_traj->components[0];
			traj_comp->baseTraj.setKnotValue(0, 0.000340);
			traj_comp->baseTraj.setKnotValue(1, -0.100323);
			traj_comp->baseTraj.setKnotValue(2, -0.001158);

			traj_comp = cur_traj->components[2];
			traj_comp->baseTraj.setKnotValue(0, 0.0);
			traj_comp->baseTraj.setKnotValue(1, 0.015874);
			traj_comp->baseTraj.setKnotValue(2, 0.0);
		}

		//we switch to the primary save file
		Globals::primary_save_config = primary_save_config;


		//and we save the structure
		con->save(true, true);

		//and go back t the secondary save file
		Globals::primary_save_config = secondary_save_config;

		
		delete con;

		
		
	} while (cma.solution().value > 1E-20);
	//*/



	//std::cin.clear();
	//std::cin.ignore();

	//int a = 2;
	//std::cin >> a;

	return 0;
}

#include "AppGUI\UI.h"

//this function launch the simulation
void launch_simulation(bool use_tk_interface,bool use_gl_interface){
	char *argv2[] = { "Framework" };
	int argc2 = 1;

	//to parameter how we evaluate the solution
	Globals::useConsole = false;

	Globals::use_tk_interface = use_tk_interface;
	Globals::use_gl_interface = use_gl_interface;


	try{
		if (use_tk_interface){
			Tk_Main(argc2, argv2, Tcl_AppInit);
		}
		else{
			launch();

			if (!use_gl_interface){
				glutHideWindow();
			}

			//and start the main loop...
			glutMainLoop();
		}
	}
	catch (char* msg){
		logPrint("Exception: %s\n", msg);
		std::cout << "press any key to continue the execution" << std::endl;
		system("pause"); 
		exit(-1);
	}
	catch (std::exception& e){
		std::cout << e.what() << std::endl;
		std::cout << "press any key to continue the execution" << std::endl;
		system("pause");
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
	vect_traj_name.push_back("pelvis_torso");


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

	//then we add the step delta at the end
	//variable_vector.push_back(SimGlobals::ipm_alteration_effectiveness);

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

	//I I detect the ipm alte I add it
	if (cur_pos < input.size()){
		SimGlobals::ipm_alteration_effectiveness = input[cur_pos];
		//limit it to 1 (since I want it to go down )
		if (SimGlobals::ipm_alteration_effectiveness > 1){
			SimGlobals::ipm_alteration_effectiveness = 1;
		}
		++cur_pos;
	}
	else{
		SimGlobals::ipm_alteration_effectiveness = 1;
	}

	//*
	//also I'll prevent the system from doing the ondulations with the pelvis
	Trajectory* cur_traj = con->getController()->getState(0)->getTrajectory(vect_traj_name[0].c_str());
	for (int j = 0; j < (int)cur_traj->components.size(); ++j){
		TrajectoryComponent* traj_comp = cur_traj->components[j];
		for (int i = 0; i < (int)traj_comp->baseTraj.getKnotCount(); ++i){
			traj_comp->baseTraj.setKnotValue(i, std::fmin(0.1, traj_comp->baseTraj.getKnotValue(i)));
		}
	}
	//*/
	//*
	//I only wanna learn the x component of the walk so I'l override the others with the value I had
	cur_traj = con->getController()->getState(0)->getTrajectory(vect_traj_name[5].c_str());
	for (int j = 0; j < (int)cur_traj->components.size(); ++j){
		TrajectoryComponent* traj_comp = cur_traj->components[0];
		traj_comp->baseTraj.setKnotValue(0, 0.000340);
		traj_comp->baseTraj.setKnotValue(1, -0.100323);
		traj_comp->baseTraj.setKnotValue(2, -0.001158);

		traj_comp = cur_traj->components[2];
		traj_comp->baseTraj.setKnotValue(0, 0.0);
		traj_comp->baseTraj.setKnotValue(1, 0.015874);
		traj_comp->baseTraj.setKnotValue(2, 0.0);
	}
	//*/


	//and we save the structure
	con->save(true, false);

	//clean the memory
	delete con;


	//test the simulation
	int result=execute_line(exe_line);

	//read the result
    //the second return value is the one when that fucking BS of glut return a retarded exception when I clos ethe program
	if (result == 0 || result == -1073740771){
		std::ifstream myfile;
		myfile.open("eval_result.txt");
		myfile >> std::fixed >> std::setprecision(8) >> last_eval;
		myfile.close();
	}
	else{
		last_eval = 10E20;
		std::cout << "execution_failed, return_val:" <<result<<std::endl;
	}

	return last_eval;
}


int execute_line(std::string line){
	int return_val=system(line.c_str());
	return return_val;
}


#include <sys/stat.h>
std::string get_folder_path(std::string name, int lookup_nbr, std::string delim){
	std::stringstream oss; 

	struct stat st;
	for (int i=0;i<lookup_nbr+1;++i){
		//now the tactic will be to look at every folder and check if we find a data folder
		std::stringstream oss2;
		oss2 << oss.str();
		oss2 << name;
		if (stat(oss2.str().c_str(), &st) == 0 && st.st_mode == 16895) {
			oss2 << delim;
			return oss2.str();
		}

		if (stat("src", &st) == 0 && st.st_mode == 16895){
			std::cout << "the configuration_data folder canno't be found" << std::endl;
			system("pause");
			exit(69);
		}

		oss << ".."<<delim;
	} 
	std::cout << "the configuration_data folder canno't be found" << std::endl;
	system("pause");
	exit(69);

}

#include <codecvt>
std::wstring s2ws(const std::string& str)
{
	typedef std::codecvt_utf8<wchar_t> convert_typeX;
	std::wstring_convert<convert_typeX, wchar_t> converterX;

	return converterX.from_bytes(str);
}

std::string ws2s(const std::wstring& wstr)
{
	typedef std::codecvt_utf8<wchar_t> convert_typeX;
	std::wstring_convert<convert_typeX, wchar_t> converterX;

	return converterX.to_bytes(wstr);
}

void update_saving_config(std::string primary_config, std::string secondary_config)
{
	std::ostringstream oss;
	oss.clear();
	oss.str("");
	oss << Globals::data_folder_path;
	oss << primary_config;

	std::ofstream myfile1(oss.str());
	if (myfile1.is_open())
	{
		myfile1 << cur_evo_name << "/learning_walk_waterlvl" << SimGlobals::water_level << "_state.rs" << std::endl;
		myfile1 << cur_evo_name << "/learning_walk_waterlvl" << SimGlobals::water_level << ".sbc" << std::endl;
	}
	myfile1.close();

	//*
	std::stringstream oss2;
	oss2 << Globals::data_folder_path;
	oss2 << secondary_config;
	std::ofstream myfile2(oss2.str());
	if (myfile2.is_open())
	{
		myfile2 << "learning_walk_state.rs" << std::endl;
		myfile2 << "learning_walk.sbc" << std::endl;
	}
	myfile2.close();
}