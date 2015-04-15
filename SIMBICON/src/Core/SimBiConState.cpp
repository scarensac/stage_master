/*
	Simbicon 1.5 Controller Editor Framework, 
	Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
	All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

	This file is part of the Simbicon 1.5 Controller Editor Framework.

	Simbicon 1.5 Controller Editor Framework is free software: you can 
	redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Simbicon 1.5 Controller Editor Framework is distributed in the hope 
	that it will be useful, but WITHOUT ANY WARRANTY; without even the 
	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
	See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Simbicon 1.5 Controller Editor Framework. 
	If not, see <http://www.gnu.org/licenses/>.
*/

#include "SimBiConState.h"
#include <Utils/Utils.h>
#include "SimGlobals.h"
#include "SimBiController.h"

#include <string>
#include <sstream>
#include <vector>

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	//split the string
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}

	if (!elems.empty()){
		//remove the endl from the last element
		std::stringstream ss2(elems.back());
		elems.pop_back();
		while (std::getline(ss2, item, '\n')) {
			elems.push_back(item);
		}
	}
	return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, elems);
	return elems;
}

/** 
	Update this component to recenter it around the new given D and V trajectories
*/
void TrajectoryComponent::updateComponent(SimBiController* con, Joint* j, Trajectory1D& newDTrajX, Trajectory1D& newDTrajZ, Trajectory1D& newVTrajX, Trajectory1D& newVTrajZ, 
										   Trajectory1D* oldDTrajX, Trajectory1D* oldDTrajZ, Trajectory1D* oldVTrajX, Trajectory1D* oldVTrajZ, 
										  int nbSamples ) {

	if( bFeedback == NULL )
		return;

	double startPhi = 0;
	double endPhi = 0;
	if( baseTraj.getKnotCount() > 0 ) {
		startPhi = min( startPhi, baseTraj.getMinPosition() );
		endPhi = min( startPhi, baseTraj.getMaxPosition() );
	}
	if( newDTrajX.getKnotCount() > 0 ) {
		startPhi = max( startPhi, newDTrajX.getMinPosition() );
		endPhi = max( startPhi, newDTrajX.getMaxPosition() );
	}

	Trajectory1D result;
	Vector3d d0, v0, newD0, newV0;
	for( int i = 0; i < nbSamples; ++i ) {
		double interp = (double) i / (nbSamples - 1.0);
		double phi = startPhi * (1.0 - interp) + endPhi * interp;

		double baseAngle = 0;
		if (baseTraj.getKnotCount() > 0)
			baseAngle = baseTraj.evaluate_catmull_rom(phi);
		SimBiController::computeDorV( phi, &newDTrajX, &newDTrajZ, LEFT_STANCE, &newD0 );
		SimBiController::computeDorV( phi, &newVTrajX, &newVTrajZ, LEFT_STANCE, &newV0 );
		SimBiController::computeDorV( phi, oldDTrajX, oldDTrajZ, LEFT_STANCE, &d0 );
		SimBiController::computeDorV( phi, oldVTrajX, oldVTrajZ, LEFT_STANCE, &v0 );

		double feedback = computeFeedback(con, j, phi, newD0 - d0, newV0 - v0);

		if (reverseAngleOnLeftStance)
			baseAngle -= feedback;
		else
			baseAngle += feedback;

		result.addKnot( phi, baseAngle );
	}
	result.simplify_catmull_rom( 0.005 );
	baseTraj.copy( result );
}


/**
	This method is used to read the knots of a base trajectory from the file, where they are specified one (knot) on a line
*/
void TrajectoryComponent::writeBaseTrajectory(FILE* f){
	if (f == NULL)
		return;

	fprintf( f, "\t\t\t%s\n", getConLineString(CON_BASE_TRAJECTORY_START) );

	for( int i=0; i < baseTraj.getKnotCount(); ++i ) {
		fprintf( f, "\t\t\t\t%lf %lf\n", baseTraj.getKnotPosition(i), baseTraj.getKnotValue(i) );
	}

	fprintf( f, "\t\t\t%s\n", getConLineString(CON_BASE_TRAJECTORY_END) );
}

/**
	This method is used to write a trajectory to a file
*/
void TrajectoryComponent::writeTrajectoryComponent(FILE* f){
	if (f == NULL)
		return;

	fprintf( f, "\t\t%s\n", getConLineString(CON_TRAJ_COMPONENT) );

	fprintf( f, "\t\t\t%s %lf %lf %lf\n", getConLineString(CON_ROTATION_AXIS), 
				rotationAxis.x, rotationAxis.y, rotationAxis.z );

	if( reverseAngleOnLeftStance )
		fprintf( f, "\t\t\t%s left\n", getConLineString(CON_REVERSE_ANGLE_ON_STANCE) );
	else if( reverseAngleOnRightStance )
		fprintf( f, "\t\t\t%s right\n", getConLineString(CON_REVERSE_ANGLE_ON_STANCE) );

	if( bFeedback )
		bFeedback->writeToFile( f );

	writeBaseTrajectory(f);

	fprintf( f, "\t\t%s\n", getConLineString(CON_TRAJ_COMPONENT_END) );
}

/**
	This method is used to read a trajectory from a file
*/
void TrajectoryComponent::readTrajectoryComponent(FILE* f){
	if (f == NULL)
		throwError("File pointer is NULL - cannot read gain coefficients!!");

	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	char tmpString[200];

	//init the reference vector
	ref_trajectories.push_back(RefTrajectory(0.90, NULL));
	ref_trajectories.push_back(RefTrajectory(0.40, NULL));
	ref_trajectories.push_back(RefTrajectory(0.00, NULL));


	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer)>195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		int lineType = getConLineType(line);
		switch (lineType) {
			case CON_TRAJ_COMPONENT_END:
			{
				//we're done...
				//load the correct trajectory
				generate_trajectory();

				return;
			}
				break;
			case CON_COMMENT:
				break;
			case CON_ROTATION_AXIS:
				if (sscanf(line, "%lf %lf %lf", &this->rotationAxis.x, &this->rotationAxis.y, &this->rotationAxis.z)!=3)
					throwError("The axis for a trajectory is specified by three parameters!");
				this->rotationAxis.toUnit();
				break;
			case CON_FEEDBACK_START:
				//read the kind of feedback that is applicable to this state
				if (sscanf(line, "%s", tmpString) != 1)
					throwError("The kind of feedback to be used for a trajectory must be specified (e.g. linear)");
				delete bFeedback;
				bFeedback = NULL;
				if (strncmp(tmpString, "linear", 6) == 0){
					bFeedback = new LinearBalanceFeedback();
					bFeedback->loadFromFile(f);
				}else
					throwError("Unrecognized type of feedback: \'%s\'", line);
				break;
			case CON_BASE_TRAJECTORY_START:
			{
				//read in the base trajectory
				baseTraj.clear();
				SimBiConState::readTrajectory1D(f, baseTraj, CON_BASE_TRAJECTORY_END);

				//we check for what speed this trajectory is defined
				char *nline = lTrim(line);
				std::string res(nline);
				std::vector<std::string> tokens = split(res, ',');
				//set the trajectory for each token
				for (int i = 0; i < (int)tokens.size(); ++i){
					if (tokens[i] == "forward"){
						ref_trajectories[0].second = new Trajectory1D(baseTraj);
					}
					else if (tokens[i] == "slow_forward"){
						ref_trajectories[1].second = new Trajectory1D(baseTraj);
					}
					else if (tokens[i] == "static"){
						ref_trajectories[2].second = new Trajectory1D(baseTraj);
					}
					else if (tokens[i] == "" || tokens[i] == " "){
						//we ignore the empty tokens
						tokens.erase(tokens.begin()+i);
						--i;
					}
					else{
						exit(1256);
					}
				}


				if (tokens.size() == 0){
					//meaning we have a default trajectory
					//we check what are the still undefined ref trajectories (if none then we delete the ref trajctories system
					//and use a single trajectory system
					std::vector<int> vect_idx_undefined;
					for (int i = 0; i < (int)ref_trajectories.size(); ++i){
						if (ref_trajectories[i].second == NULL){
							vect_idx_undefined.push_back(i);
						}
					}

					if (vect_idx_undefined.size() == ref_trajectories.size()){
						//this mean on the default trajectory is defined so we switch back to the single trajectory system
						ref_trajectories.clear();
					}
					else{
						for (int i = 0; i < (int)vect_idx_undefined.size(); ++i){
							ref_trajectories[vect_idx_undefined[i]].second = new Trajectory1D(baseTraj);
						}
					}

				}
			}
				
				break;
			case CON_REVERSE_ANGLE_ON_STANCE:
				if (strncmp(trim(line), "left", 4) == 0)
					reverseAngleOnLeftStance = true;
				else if (strncmp(trim(line), "right", 5) == 0)
					reverseAngleOnRightStance = true;
				else 
					throwError("When using the \'startingStance\' keyword, \'left\' or \'right\' must be specified!");
				break;
			case CON_NOT_IMPORTANT:
				tprintf("Ignoring input line: \'%s\'\n", line); 
				break;
			default:
				throwError("Incorrect SIMBICON input file: \'%s\' - unexpected line.", buffer);
		}
	}
	throwError("Incorrect SIMBICON input file: No \'/trajectory\' found ", buffer);
}


/**
this method is use to generate the correct trajectory from the reference trajectories
*/
void TrajectoryComponent::generate_trajectory(){
	//I'll load the trajtectorie the nearest fromthe current speed for the start (until I code the combinaison funtion)
	if (ref_trajectories.size() > 0){
		/*
		//this version is a simple attribution to the nearest
		double min = std::abs(ref_trajectories[0].first - SimGlobals::velDSagittal);
		int idx_min = 0;
		for (int i = 1; i < (int)ref_trajectories.size(); ++i){
			double res = std::abs(ref_trajectories[i].first - SimGlobals::velDSagittal);
			if (res < min){
				min = res;
				idx_min = i;
			}
		}

		baseTraj = *(ref_trajectories[idx_min].second);
		//*/


		int idx_min = -1;
		for (int i = 0; i < (int)ref_trajectories.size(); ++i){
			if ((ref_trajectories[i].first - SimGlobals::velDSagittal) < 0){
				idx_min = i;
				break;
			}
		}

		if (idx_min == 0){
			baseTraj = *(ref_trajectories[0].second);
		}
		else if (idx_min == -1){
			baseTraj = *(ref_trajectories[ref_trajectories.size()-1].second);
		}
		else{
			//this case we are between 2 speed so we need to build a custon trajectory
			baseTraj.clear();
			double cur_phi = 0;

			//we need the distance from each trajectory
			double dv1 = ref_trajectories[idx_min - 1].first - SimGlobals::velDSagittal;
			dv1 *= dv1;
			double dv2 = ref_trajectories[idx_min].first - SimGlobals::velDSagittal;
			dv2 *= dv2;
			double dv = dv1 + dv2;

			double r = dv1 / dv;

			for (int i = 0; i < 11; ++i){
				double val1 = ref_trajectories[idx_min - 1].second->evaluate_catmull_rom(cur_phi);
				double val2 = ref_trajectories[idx_min].second->evaluate_catmull_rom(cur_phi);
				baseTraj.addKnot(cur_phi, val1*(1-r)+val2*r);
				cur_phi += 0.1;
			}
		}

		//*
		//this version will create a trajectorie by combining the 2 adjascent trajectories using a square distance


		//*/
	}
}

/*************************************************************************************/
/*                          TRAJECTORY COMPONENT END                                 */
/*************************************************************************************/



/**
	This method is used to write the knots of a strength trajectory to the file, where they are specified one (knot) on a line
*/
void Trajectory::writeStrengthTrajectory(FILE* f){
	if (f == NULL || strengthTraj == NULL)
		return;

	fprintf( f, "\t\t\t%s\n", getConLineString(CON_STRENGTH_TRAJECTORY_START) );

	for( int i=0; i < strengthTraj->getKnotCount(); ++i ) {
		fprintf( f, "\t\t\t\t%lf %lf\n", strengthTraj->getKnotPosition(i), strengthTraj->getKnotValue(i) );
	}

	fprintf( f, "\t\t\t%s\n", getConLineString(CON_STRENGTH_TRAJECTORY_END) );
}

/**
	This method is used to write a trajectory to a file
*/
void Trajectory::writeTrajectory(FILE* f){
	if (f == NULL)
		return;

	fprintf( f, "\t%s %s\n", getConLineString(CON_TRAJECTORY_START), jName );

	if (relToCharFrame)
		fprintf(f, "\t%s\n", getConLineString(CON_CHAR_FRAME_RELATIVE));

	if( strengthTraj != NULL )
		writeStrengthTrajectory( f );

	for( uint i=0; i < components.size(); ++i ) {
		fprintf( f, "\n" );
		components[i]->writeTrajectoryComponent( f );
	}

	fprintf( f, "\t%s\n", getConLineString(CON_TRAJECTORY_END) );
}

/**
	This method is used to read a trajectory from a file
*/
void Trajectory::readTrajectory(FILE* f){
	if (f == NULL)
		throwError("File pointer is NULL - cannot read gain coefficients!!");

	//have a temporary buffer used to read the file line by line...
	char buffer[200];

	TrajectoryComponent* newComponent;

	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer)>195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		int lineType = getConLineType(line);
		switch (lineType) {
			case CON_STRENGTH_TRAJECTORY_START:
				//read in the base trajectory
				if( strengthTraj != NULL )
					throwError( "Two strength trajectory, this is illegal!" );
				strengthTraj = new Trajectory1D();
				SimBiConState::readTrajectory1D(f, *strengthTraj, CON_STRENGTH_TRAJECTORY_END );
				break;
			case CON_TRAJECTORY_END:
				//we're done...
				return;
				break;
			case CON_CHAR_FRAME_RELATIVE:
				relToCharFrame = true;
				break;
			case CON_COMMENT:
				break;
			case CON_TRAJ_COMPONENT:
				//read in the base trajectory
				newComponent = new TrajectoryComponent();
				newComponent->readTrajectoryComponent(f);
				components.push_back(newComponent);
				break;
			case CON_NOT_IMPORTANT:
				tprintf("Ignoring input line: \'%s\'\n", line); 
				break;
			default:
				throwError("Incorrect SIMBICON input file: \'%s\' - unexpected line.", buffer);
		}
	}
	throwError("Incorrect SIMBICON input file: No \'/trajectory\' found ", buffer);
}

/**
	This method is used to read the state parameters from a file
*/
void SimBiConState::readState(FILE* f, int offset){
	if (f == NULL)
		throwError("File pointer is NULL - cannot read gain coefficients!!");

	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	Trajectory* tempTraj;


	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer)>195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		int lineType = getConLineType(line);
		switch (lineType) {
			case CON_STATE_END:
				//we're done...
				return;
				break;
			case CON_NEXT_STATE:
				if (sscanf(line, "%d", &this->nextStateIndex) != 1)
					throwError("An index must be specified when using the \'nextState\' keyword");
				this->nextStateIndex += offset;
				break;
			case CON_STATE_DESCRIPTION:
				strcpy(this->description, trim(line));
				break;
			case CON_STATE_TIME:
				if (sscanf(line, "%lf", &stateTime)!=1)
					throwError("The time that is expected to be spent in this state needs to be provided.");
				break;
			case CON_STATE_STANCE:
				reverseStance = false;
				keepStance = false;
				if (strncmp(trim(line), "left",4) == 0)
					stateStance = LEFT_STANCE;
				else
					if (strncmp(trim(line), "right", 5) == 0)
						stateStance = RIGHT_STANCE;
					else
						if (strncmp(trim(line), "reverse", 7) == 0)
							reverseStance = true;
						else if (strncmp(trim(line), "same", 4) == 0)
								keepStance = true;
							else
								throwError("When using the \'stateStance\' keyword, \'left\', \'right\' or \'reverse\' must be specified.");
				break;
			case CON_TRANSITION_ON:
				transitionOnFootContact = false;
				if (strncmp(trim(line), "footDown", 8) == 0)
					transitionOnFootContact = true;
				else
					if (strncmp(trim(line), "timeUp", 6) == 0)
						//nothn' to do, since this is the default
						;
					else
						throwError("When using the \'transitionOn\' keyword, \'footDown\' or \'timeUp\' must be specified.");
				break;
			case CON_TRAJECTORY_START:
				//create a new trajectory, and read its information from the file
				
				Trajectory* tempTraj;
				tempTraj = new Trajectory();
				strcpy(tempTraj->jName, trim(line));
				tempTraj->readTrajectory(f);
				this->sTraj.push_back(tempTraj);
				
				break;

			case CON_D_TRAJX_START:
				if( dTrajX != NULL )
					throwError( "Two dTrajX trajectory, this is illegal!" );
				dTrajX = new Trajectory1D();
				readTrajectory1D( f, *dTrajX, CON_D_TRAJX_END );
				break;

			case CON_D_TRAJZ_START:
				if( dTrajZ != NULL )
					throwError( "Two dTrajZ trajectory, this is illegal!" );
				dTrajZ = new Trajectory1D();
				readTrajectory1D( f, *dTrajZ, CON_D_TRAJZ_END );
				break;

			case CON_V_TRAJX_START:
				if( vTrajX != NULL )
					throwError( "Two vTrajX trajectory, this is illegal!" );
				vTrajX = new Trajectory1D();
				readTrajectory1D( f, *vTrajX, CON_V_TRAJX_END );
				break;

			case CON_V_TRAJZ_START:
				if( vTrajZ != NULL )
					throwError( "Two vTrajZ trajectory, this is illegal!" );
				vTrajZ = new Trajectory1D();
				readTrajectory1D( f, *vTrajZ, CON_V_TRAJZ_END );
				break;

			case CON_COMMENT:
				break;


			case CON_NOT_IMPORTANT:
				tprintf("Ignoring input line: \'%s\'\n", line);
				break;


			default:
				throwError("Incorrect SIMBICON input file: \'%s\' - unexpected line.", buffer);
		}
	}
	throwError("Incorrect SIMBICON input file: No \'/State\' found", buffer);
}




/**
	This method is used to write the state parameters to a file
*/
void SimBiConState::writeState(FILE* f, int index){
	if (f == NULL)
		return;

	fprintf( f, "%s %d\n", getConLineString(CON_STATE_START), index );

	fprintf( f, "\t%s %s\n", getConLineString(CON_STATE_DESCRIPTION), description );
	fprintf( f, "\t%s %d\n", getConLineString(CON_NEXT_STATE), nextStateIndex );
	fprintf( f, "\t%s %s\n", getConLineString(CON_TRANSITION_ON), 
		transitionOnFootContact?"footDown":"timeUp" );
	
	if( reverseStance )
		fprintf( f, "\t%s reverse\n", getConLineString(CON_STATE_STANCE) );
	else if( keepStance )
		fprintf( f, "\t%s same\n", getConLineString(CON_STATE_STANCE) );
	else if( stateStance == LEFT_STANCE )
		fprintf( f, "\t%s left\n", getConLineString(CON_STATE_STANCE) );
	else if( stateStance == RIGHT_STANCE )
		fprintf( f, "\t%s right\n", getConLineString(CON_STATE_STANCE) );

	fprintf( f, "\t%s %lf\n", getConLineString(CON_STATE_TIME), stateTime );
	
	fprintf( f, "\n" );

	if( dTrajX != NULL )
		writeTrajectory1D( f, *dTrajX, CON_D_TRAJX_START, CON_D_TRAJX_END );
	if( dTrajZ != NULL )
		writeTrajectory1D( f, *dTrajZ, CON_D_TRAJZ_START, CON_D_TRAJZ_END );
	if( vTrajX != NULL )
		writeTrajectory1D( f, *vTrajX, CON_V_TRAJX_START, CON_V_TRAJX_END );
	if( vTrajZ != NULL )
		writeTrajectory1D( f, *vTrajZ, CON_V_TRAJZ_START, CON_V_TRAJZ_END );

	fprintf( f, "\n" );

	for( uint i=0; i<sTraj.size(); ++i ) {
		fprintf( f, "\n" );
		sTraj[i]->writeTrajectory( f );	
	}
	
	fprintf( f, "%s\n", getConLineString(CON_STATE_END) );


}


/**
	This method is used to read the knots of a strength trajectory from the file, where they are specified one (knot) on a line
*/
void SimBiConState::readTrajectory1D(FILE* f, Trajectory1D& result, int endingLineType ){
	if (f == NULL)
		throwError("File pointer is NULL - cannot read gain coefficients!!");

	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	double temp1, temp2;

	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer)>195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		int lineType = getConLineType(line);
		if( lineType == endingLineType )
			//we're done...
			return;

		switch (lineType) {
			case CON_COMMENT:
				break;
			case CON_NOT_IMPORTANT:
				//we expect pairs of numbers, one pair on each row, so see if we have a valid pair
				if (sscanf(line, "%lf %lf", &temp1, &temp2) == 2){
					result.addKnot(temp1, temp2);
				}else
					tprintf("Ignoring input line: \'%s\'\n", line); 
				break;
			default:
				throwError("Incorrect SIMBICON input file: \'%s\' - unexpected line.", buffer);
		}
	}
	throwError("Incorrect SIMBICON input file: Trajectory not closed ", buffer);
}




/**
	This method is used to write a trajectory to the file
*/
void SimBiConState::writeTrajectory1D(FILE* f, Trajectory1D& result, int startingLineType, int endingLineType ){
	if (f == NULL)
		return;

	fprintf( f, "\t%s\n", getConLineString(startingLineType) );

	for( int i=0; i < result.getKnotCount(); ++i ) {
		fprintf( f, "\t\t%lf %lf\n", result.getKnotPosition(i), result.getKnotValue(i) );
	}

	fprintf( f, "\t%s\n", getConLineString(endingLineType) );
}


/** 
	Update all the trajectories to recenter them around the new given D and V trajectories
	Also save these new D and V trajectories.
*/
void SimBiConState::updateDVTrajectories(SimBiController* con, Joint* j, Trajectory1D& newDTrajX, Trajectory1D& newDTrajZ,Trajectory1D& newVTrajX, Trajectory1D& newVTrajZ, int nbSamples ) {

	int nbTraj = sTraj.size();
	for( int i = 0; i < nbTraj; ++i ) {		
		sTraj[i]->updateComponents( con, j, newDTrajX, newDTrajZ, newVTrajX, newVTrajZ, dTrajX, dTrajZ, vTrajX, vTrajZ, nbSamples );
	}
	
	if( dTrajX != NULL )
		delete dTrajX;
	if( dTrajZ != NULL )
		delete dTrajZ;
	if( vTrajX != NULL )
		delete vTrajX;
	if( vTrajZ != NULL )
		delete vTrajZ;
	dTrajX = new Trajectory1D( newDTrajX );
	dTrajZ = new Trajectory1D( newDTrajZ );
	vTrajX = new Trajectory1D( newVTrajX );
	vTrajZ = new Trajectory1D( newVTrajZ );
}

