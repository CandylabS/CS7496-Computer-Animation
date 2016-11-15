#include "simulator.h"
#include <fstream>
#include <iostream>

using namespace std;
using namespace Eigen;

Simulator::Simulator(int _sysType) : sysType(_sysType), mTimeStep(0.0), mElapsedTime(0.0), mParticles(0), mFrameNumber(0) {
    if (sysType == 0) {			//galileo experiment
        // initialize the particles
        mParticles.resize(3);

        // Init particle positions (default is 0, 0, 0)
        mParticles[0].mPosition[0] = -0.3;
        mParticles[0].mPosition[1] = 20.0;
        mParticles[1].mPosition[0] = 0.0;
        mParticles[1].mPosition[1] = 20.0;
        mParticles[2].mPosition[0] = 0.3;
        mParticles[2].mPosition[1] = 20.0;

        // Init particle colors (default is red)
        mParticles[1].mColor = Eigen::Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
        mParticles[2].mColor = Eigen::Vector4d(0.2, 0.2, 0.9, 1.0); // Blue

        mTimeStep = 0.03;
    } else {	//tinker toy
        // initialize the particles
        mParticles.resize(2);

        // Init particle positions (default is 0, 0, 0)
        mParticles[0].mPosition[0] = 0.2;
        mParticles[1].mPosition[0] = 0.2;
        mParticles[1].mPosition[1] = -0.1;

        // Init Matrix and Vectors
        mInverseMass = MatrixXd::Zero(3 * mParticles.size(), 3 * mParticles.size());
        mJacobian = MatrixXd::Zero(2, 3 * mParticles.size());
        dJacobian = MatrixXd::Zero(2, 3 * mParticles.size());
        dPos = VectorXd::Zero(3 * mParticles.size());
        mQ = VectorXd::Zero(3 * mParticles.size());
        mRambda = VectorXd::Zero(2);	// 2 constraints
        mC = VectorXd::Zero(2);
        dC = VectorXd::Zero(2);

        // raiuds and rod length
        mRadius = 0.2;
        mRod = 0.1;
        mKs = 0.2;
        mKd = 0.2;

        mTimeStep = 0.0003;
    }
}


void Simulator::reset() {
    if (sysType == 0) {			//galileo experiment
        mParticles[0].mPosition[0] = -0.3;
        mParticles[0].mPosition[1] = 20.0;
        mParticles[1].mPosition[0] = 0.0;
        mParticles[1].mPosition[1] = 20.0;
        mParticles[2].mPosition[0] = 0.3;
        mParticles[2].mPosition[1] = 20.0;

    } else {			//tinker toy
        mParticles[0].mPosition[0] = 0.2;
        mParticles[0].mPosition[1] = 0.0;
        mParticles[1].mPosition[0] = 0.2;
        mParticles[1].mPosition[1] = -0.1;

    }

    for (unsigned int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mVelocity.setZero();
        mParticles[i].mAccumulatedForce.setZero();
    }

    mElapsedTime = 0;
    mFrameNumber = 0;

    view_translation.setZero();
    view_rotation.setZero();
}


//analytical solution
void Simulator::analyticSolve(Particle &p) {
    // TODO: Replace the following code with correct analytical solution using mElapsedTime
    // x = v * t + 1/2 * a * t * t
    p.mPosition[1] = 20.0 - 9.8 * pow((mElapsedTime + mTimeStep), 2) / 2;
}

//forward euler integrator
void Simulator::fwdEulerSolve(Particle &p) {
    // TODO: Replace the following code with fwd euler
    p.mPosition += p.mVelocity * mTimeStep;
    p.mVelocity += p.mAccumulatedForce / p.mMass * mTimeStep;
}

//midpoint integrator
void Simulator::midPointSolve(Particle &p) {
    // TODO: Replace the following code with midpoint method
    // middle point
    p.mVelocity += p.mAccumulatedForce / p.mMass * mTimeStep / 2;
    // Eigen::Vector3d mMiddleVel = p.mVelocity + p.mAccumulatedForce / p.mMass * mTimeStep / 2;
    // full step
    p.mPosition += p.mVelocity * mTimeStep;
    p.mVelocity += p.mAccumulatedForce / p.mMass * mTimeStep / 2;
}

//Part 1 : Galileo experiement
void Simulator::part1Galileo() {
    for (unsigned int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce[1] -= 9.8 * mParticles[i].mMass;
    }

    analyticSolve(mParticles[0]);
    fwdEulerSolve(mParticles[1]);
    midPointSolve(mParticles[2]);

    for (unsigned int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce.setZero();
    }
}

//Part 2 : Tinker Toy
void Simulator::part2TinkerToy() {
    // TODO: Replace the following code
    // apply force
    for (unsigned int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce[1] -= 9.8 * mParticles[i].mMass;
    }

    // compute mInverse Matrix W
    for (unsigned int i = 0; i < mParticles.size(); i++) {
        mInverseMass(i * 3, i * 3) = 1 / mParticles[i].mMass;
        mInverseMass(i * 3 + 1, i * 3 + 1) = 1 / mParticles[i].mMass;
        mInverseMass(i * 3 + 2, i * 3 + 2) = 1 / mParticles[i].mMass;
    }

    // compute Jacobian Matrix
    unsigned int i = 0; unsigned int j = 0;	// C1
    mJacobian(i, j * 3) = mParticles[0].mPosition[0];
    mJacobian(i, j * 3 + 1) = mParticles[0].mPosition[1];
    mJacobian(i, j * 3 + 2) = mParticles[0].mPosition[2];
    j++;
    mJacobian(i, j * 3) = 0.0;
    mJacobian(i, j * 3 + 1) = 0.0;
    mJacobian(i, j * 3 + 2) = 0.0;
    i++; j = 0;	// C2
    mJacobian(i, j * 3) = mParticles[0].mPosition[0] - mParticles[1].mPosition[0];
    mJacobian(i, j * 3 + 1) = mParticles[0].mPosition[1] - mParticles[1].mPosition[1];
    mJacobian(i, j * 3 + 2) = mParticles[0].mPosition[2] - mParticles[1].mPosition[2];
    j++;
    mJacobian(i, j * 3) = -(mParticles[0].mPosition[0] - mParticles[1].mPosition[0]);
    mJacobian(i, j * 3 + 1) = -(mParticles[0].mPosition[1] - mParticles[1].mPosition[1]);
    mJacobian(i, j * 3 + 2) = -(mParticles[0].mPosition[2] - mParticles[1].mPosition[2]);

    // compute derivative Jacobian
    i = 0; j = 0;	// dC1
    dJacobian(i, j * 3) = mParticles[0].mVelocity[0];
    dJacobian(i, j * 3 + 1) = mParticles[0].mVelocity[1];
    dJacobian(i, j * 3 + 2) = mParticles[0].mVelocity[2];
    j++;
    dJacobian(i, j * 3) = 0.0;
    dJacobian(i, j * 3 + 1) = 0.0;
    dJacobian(i, j * 3 + 2) = 0.0;
    i++; j = 0;		// dC2
    dJacobian(i, j * 3) = mParticles[0].mVelocity[0] - mParticles[1].mVelocity[0];
    dJacobian(i, j * 3 + 1) = mParticles[0].mVelocity[1] - mParticles[1].mVelocity[1];
    dJacobian(i, j * 3 + 2) = mParticles[0].mVelocity[2] - mParticles[1].mVelocity[2];
    j++;
    dJacobian(i, j * 3) = -(mParticles[0].mVelocity[0] - mParticles[1].mVelocity[0]);
    dJacobian(i, j * 3 + 1) = -(mParticles[0].mVelocity[1] - mParticles[1].mVelocity[1]);
    dJacobian(i, j * 3 + 2) = -(mParticles[0].mVelocity[2] - mParticles[1].mVelocity[2]);

    // compute q, Q
    for (unsigned int i = 0; i < mParticles.size(); i++) {
        dPos(i * 3) = mParticles[i].mVelocity[0];
        dPos(i * 3 + 1) = mParticles[i].mVelocity[1];
        dPos(i * 3 + 2) = mParticles[i].mVelocity[2];

        mQ(i * 3) = mParticles[i].mAccumulatedForce[0];
        mQ(i * 3 + 1) = mParticles[i].mAccumulatedForce[1];
        mQ(i * 3 + 2) = mParticles[i].mAccumulatedForce[2];
    }

    // compute C and dC
    mC(0) = mParticles[0].mPosition.dot(mParticles[0].mPosition) / 2 - pow(mRadius, 2) / 2;
    mC(1) = (mParticles[0].mPosition - mParticles[1].mPosition).dot(mParticles[0].mPosition - mParticles[1].mPosition) / 2 - pow(mRod, 2) / 2;
    dC(0) = mParticles[0].mPosition.dot(mParticles[0].mVelocity);
    dC(1) = mParticles[0].mPosition.dot(mParticles[0].mVelocity)
            - mParticles[0].mPosition.dot(mParticles[1].mVelocity)
            - mParticles[1].mPosition.dot(mParticles[0].mVelocity)
            + mParticles[1].mPosition.dot(mParticles[1].mVelocity);

    // compute rambda
    mRambda = (mJacobian * mInverseMass * mJacobian.transpose()).inverse() *
              (-dJacobian * dPos - mJacobian * mInverseMass * mQ - mKs * mC - mKd * dC);

    // apply constraint force
    mQ += mJacobian.transpose() * mRambda;

    for (unsigned int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce[0] = mQ(i * 3);
        mParticles[i].mAccumulatedForce[1] = mQ(i * 3 + 1);
        mParticles[i].mAccumulatedForce[2] = mQ(i * 3 + 2);
    }

    // update position and velocity
    for (unsigned int i = 0; i < mParticles.size(); i++) {
        // fwdEulerSolve(mParticles[i]);
        //can also try midpoint
        midPointSolve(mParticles[i]);
    }

    // clear force
    for (unsigned int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce.setZero();
    }
}



void Simulator::simulate() {
    if (sysType == 0) {
        part1Galileo();
    } else {
        part2TinkerToy();
    }

    mElapsedTime += mTimeStep;
    mFrameNumber++;
}







