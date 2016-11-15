#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "dart/dart.h"

class RigidBody;
class CollisionInterface;

class MyWorld {
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MyWorld();

	virtual ~MyWorld();

	void initializePinata();
    
	int getNumRigidBodies() {
		return mRigidBodies.size();
	}

	RigidBody* getRigidBody(int _index) {
		return mRigidBodies[_index];
	}
   
	// TODO: your simulation and collision handling code goes here
	void simulate();
	void collisionHandling();
	Eigen::Vector3d deriveContactPoint(RigidBody *_rb);
	double denominator(RigidBody *_rb);
	void updateMomentum(RigidBody *_rb);
        
	CollisionInterface* getCollisionDetector() {
		return mCollisionDetector;
	}

	int getSimFrames() const { 
		return mFrame; 
	}

	void setDartWorld(dart::simulation::WorldPtr _dartWorld) {
		mPinataWorld = _dartWorld;
	}

	dart::simulation::WorldPtr getPinataWorld() {
		return mPinataWorld;
	}

	double getTimeStep() {
		return mTimeStep;
	}

	void setExtForce(int _dir, double _mag) {
		mForce[_dir] = _mag;
	}
  
 protected:
	int mFrame;
	double mTimeStep;
	Eigen::Vector3d mGravity;
	std::vector<RigidBody*> mRigidBodies;
	CollisionInterface* mCollisionDetector; // Access to collision detection information
	dart::simulation::WorldPtr mPinataWorld;
	Eigen::Vector3d mForce;

	// Collision Contact Information
	Eigen::Vector3d point;
	Eigen::Vector3d normal;
	Eigen::Vector3d pVelocity;
	double j;
};

#endif
