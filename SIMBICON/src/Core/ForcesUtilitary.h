#pragma once

//#include "Core\SimGlobals.h"
#include <map>

/**
	the application point is defined in local coordinates but the F vector is defined in wolrd coordinates
*/

struct ForceStruct
{
	Point3d pt;
	Vector3d F;

	ForceStruct(){
		pt = Point3d(0, 0, 0);
		F = Vector3d(0, 0, 0);
	}
	
};

struct WaterImpact
{
	ForceStruct boyancy;
	Vector3d drag_torque;

	WaterImpact(){
		boyancy = ForceStruct();
		drag_torque = Vector3d();
	}
};