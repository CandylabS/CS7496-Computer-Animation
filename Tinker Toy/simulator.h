#ifndef SIMULATOR_H
#define SIMULATOR_H

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/Dense>
#include <vector>
#include <string>

#include "particle.h"

// class containing objects to be simulated
class Simulator {
public:
	Simulator() {}			//dflt ctor - nothing goes in here!
    Simulator(int _sysType);
        
    void simulate();

	void integrate();


	int getNumParticles() {return mParticles.size();}

	Particle* getParticle(int index) {return &mParticles[index];}

	double getTimeStep() { return mTimeStep;}
	
	int getFrameNum() { return mFrameNumber; }
    
    void reset();

	void analyticSolve(Particle& p);
	void fwdEulerSolve(Particle& p);
	void midPointSolve(Particle& p);

	void part1Galileo();

	void part2TinkerToy();


	Eigen::Vector3d view_translation = Eigen::Vector3d(0, 0, 0);
	Eigen::Vector2d view_rotation = Eigen::Vector2d::Zero();
	bool mouse_down = false;
	Eigen::Vector2d mouse_click_position;


private:
    double mTimeStep;       // time step
	double mElapsedTime;    // time pased since beginning of simulation
    std::vector<Particle> mParticles;
	int sysType;
	int mFrameNumber;

	Eigen::MatrixXd mInverseMass;	// inverse Matrix for particle system
	Eigen::MatrixXd mJacobian;
	Eigen::MatrixXd dJacobian;
	Eigen::VectorXd dPos;
	Eigen::VectorXd mQ;
	Eigen::VectorXd mRambda;
	Eigen::VectorXd mC;
	Eigen::VectorXd dC;
	double mKs;
	double mKd;
	double mRadius;
	double mRod;
};

#endif  // SIMULATOR_H
