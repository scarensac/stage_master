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

#pragma once

#include <Utils/Utils.h>
#include <Physics/RBState.h>
#include <Physics/RBProperties.h>
#include <GLUtils/GLMesh.h>
#include <MathLib/TransformationMatrix.h>
#include <Physics/CollisionDetectionPrimitive.h>
#include <GLUtils/GLUtils.h>

class Force;
class ArticulatedFigure;


//define some drawing flags:
#define SHOW_MESH				0x0001
#define SHOW_BODY_FRAME			0x0002
#define SHOW_CD_PRIMITIVES		0x0008
#define SHOW_MIN_BDG_SPHERE		0x0010
#define SHOW_JOINTS				0x0020
#define SHOW_COLOURS			0x0040
#define SHOW_FRICTION_PARTICLES 0x0080

/*=========================================================================================================================================================================*
 | This is the implementation of a Rigid Body class. It holds all the attributes that characterize the rigid body (state information, collision detection primitives, etc).|
 | This class is used as the basis for an Articulated Rigid Body. Together with the PhysicalWorld class, this class is used to implement the dynamics of rigid bodies.     |
 | NOTE: It is assumed that the location of the center of mass of the object in local coordinates is (0,0,0) and that the principal moments of inertia are aligned to the  |
 | local coordinates x, y, and z axes!                                                                                                                                     |
 *=========================================================================================================================================================================*/

class RigidBody  {
friend class AbstractRBEngine;
friend class HingeJoint;
friend class UniversalJoint;
friend class BallInSocketJoint;
friend class ODEWorld;
friend class Character;
friend class SimBiController;
friend class Joint;
friend class PoseController;


protected:
	//--> the state of the rigid body: made up of the object's position in the world, its orientation and linear/angular velocities (stored in world coordinates)
	RBState state;
	//--> the physical properties of the rigid bodies: mass and inertia, stored in a convenient to use form
	RBProperties props;
	//--> an array with all the collision detection primitives that are relevant for this rigid body
	DynamicArray<CollisionDetectionPrimitive*> cdps;
	//--> the mesh(es) that are used when displaying this rigid body
	DynamicArray<GLMesh*> meshes;
	//--> the name of the rigid body - it might be used to reference the object for articulated bodies
	char name[100];
	//--> the id of the rigid body
	int id;

	//--> this transformation matrix is used to transform points/vectors from local coordinates to global coordinates. It will be updated using the state
	//information, and is therefore redundant, but it will be used to draw the object quickly. Everytime the state is updated, this matrix must also be updated!
//	TransformationMatrix toWorld;

public:
	/**
		Default constructor
	*/
	RigidBody(void);

	/**
		Default destructor
	*/
	virtual ~RigidBody(void);

	/**
		This method returns the coordinates of the point that is passed in as a parameter(expressed in local coordinates), in world coordinates.
	*/
	Point3d getWorldCoordinates(const Point3d& localPoint);

	/**
		This method is used to return the local coordinates of the point that is passed in as a parameter (expressed in global coordinates)
	*/
	Point3d getLocalCoordinates(const Point3d& globalPoint);

	/**
		This method is used to return the local coordinates of the vector that is passed in as a parameter (expressed in global coordinates)
	*/
	Vector3d getLocalCoordinates(const Vector3d& globalVector);

	/**
		This method returns the vector that is passed in as a parameter(expressed in local coordinates), in world coordinates.
	*/
	Vector3d getWorldCoordinates(const Vector3d& localVector);

	/**
		This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the
		resulting velocity will be expressed in world coordinates.
	*/
	Vector3d getAbsoluteVelocityForLocalPoint(const Point3d& localPoint);

	/**
		This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in global coordinates, and the
		resulting velocity will be expressed in world coordinates.
	*/
	Vector3d getAbsoluteVelocityForGlobalPoint(const Point3d& globalPoint);

	/**
		This method returns the world coordinates of the position of the center of mass of the object
	*/
	inline Point3d getCMPosition(){
		return state.position;
	}

	/**
		This method sets the world coordinate of the posision of the center of mass of the object
	*/
	inline void setCMPosition(const Point3d& newCMPos){
		state.position = newCMPos;		
	}

	/**
		This method returns the body's center of mass velocity
	*/
	inline Vector3d getCMVelocity(){
		return state.velocity;
	}

	/**
		This method sets the velocity of the center of mass of the object
	*/
	inline void setCMVelocity(const Vector3d& newCMVel){
		state.velocity = newCMVel;
	}

	/**
		this method sets the angular velocity of the body
	*/
	inline void setAngularVelocity(const Vector3d& newAVel){
		state.angularVelocity = newAVel;
	}

	/**
		This method returns the rigid body's coefficient of restitution
	*/
	inline double getRestitutionCoefficient(){
		return props.epsilon;
	}

	/**
		This method returns the rigid body's coefficient of restitution
	*/
	inline double getFrictionCoefficient(){
		return props.mu;
	}

	/**
		This method draws the current rigid body.
	*/
	virtual void draw(int flags);

	/**
		This method renders the rigid body in its current state as a set of vertices 
		and faces that will be appended to the passed OBJ file.

		vertexIdxOffset indicates the index of the first vertex for this object, this makes it possible to render
		multiple different meshes to the same OBJ file
		 
		Returns the number of vertices written to the file
	*/
	uint renderToObjFile(FILE* fp, uint vertexIdxOffset);


	/**
		This method loads all the pertinent information regarding the rigid body from a file.
	*/
	void loadFromFile(FILE* fp);

	/**
		Returns the mass of the rigid body
	*/
	inline double getMass(){
		return props.mass;
	}

	/**
		This method is used to compute the correct toWorld coordinates matrix based on the state of the rigid body
	*/
//	void updateToWorldTransformation();

	/**
		this method sets the id of the current rigid body.
	*/
	inline void setBodyID(int newID){
		this->id = newID;
	}

	/**
		this method returns the body's principal moments of inertia, about the principal axes (which correspond to the bodie's local coordinate
		frame axes)
	*/
	inline Vector3d getPMI(){
		return props.MOI_local;
	}

	/**
		this method returns the body's orientation
	*/
	inline Quaternion getOrientation(){
		return state.orientation;
	}

	/**
		this method sets the body's orientation
	*/
	inline void setOrientation(Quaternion q){
		state.orientation = q;
	}

	/**
		this method returns the body's angular velocity
	*/
	inline Vector3d getAngularVelocity(){
		return state.angularVelocity;
	}

	/**
		this method returns true if the current body is locked, false otherwised
	*/
	inline bool isLocked(){
		return props.isLocked;
	}

	/**
		this method returns false if this body is a simple rigid bory, false if it is an articulated figure
	*/
	virtual bool isArticulated(){
		return false;
	}

	/**
		this method is used to update the world positions of the collision detection primitives
	*/
	void updateWorldCDPs();

	virtual ArticulatedFigure* getAFParent(){
		return NULL;
	}

	/**
	this function is used to set the mesh color (in case u wanna change it during the execution, to follow the stance foot for exemple)
	*/
	void set_mesh_color(float r, float g, float b, float a){
		for (int i = 0; i < (int)meshes.size(); ++i){
			meshes[i]->setColour(r, g, b, a);
		}
	}

};

