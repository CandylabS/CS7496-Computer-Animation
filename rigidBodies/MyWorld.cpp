#include "MyWorld.h"
#include "RigidBody.h"
#include "CollisionInterface.h"
#include <iostream>

using namespace Eigen;
using namespace std;

MyWorld::MyWorld() {
    mFrame = 0;
    mTimeStep = 0.001;
    mGravity = Vector3d(0.0, -9.8, 0.0);
    mForce.setZero();
    // Create a collision detector
    mCollisionDetector = new CollisionInterface();

    // Create and intialize two default rigid bodies
    RigidBody *rb1 = new RigidBody(dart::dynamics::Shape::BOX, Vector3d(0.05, 0.05, 0.05));
    mCollisionDetector->addRigidBody(rb1, "box"); // Put rb1 in collision detector
    rb1->mPosition[0] = -0.3;
    rb1->mPosition[1] = -0.5;

    rb1->mAngMomentum = Vector3d(0.0, 0.01, 0.0);
    mRigidBodies.push_back(rb1);

    RigidBody *rb2 = new RigidBody(dart::dynamics::Shape::ELLIPSOID, Vector3d(0.06, 0.06, 0.06));
    mCollisionDetector->addRigidBody(rb2, "ellipse"); // Put rb2 in collision detector
    rb2->mPosition[0] = 0.3;
    rb2->mPosition[1] = -0.5;
    rb2->mAngMomentum = Vector3d(0.01, 0.0, 0.0);
    rb2->mColor = Vector4d(0.2, 0.8, 0.2, 1.0); // Blue
    mRigidBodies.push_back(rb2);
}

void MyWorld::initializePinata() {
    // Add pinata to the collison detector
    mCollisionDetector->addSkeleton(mPinataWorld->getSkeleton(0));

    // Add some damping in the Pinata joints
    int nJoints = mPinataWorld->getSkeleton(0)->getNumBodyNodes();

    for (int i = 0; i < nJoints; i++) {
        int nDofs = mPinataWorld->getSkeleton(0)->getJoint(i)->getNumDofs();

        for (int j = 0; j < nDofs; j++)
            mPinataWorld->getSkeleton(0)->getJoint(i)->setDampingCoefficient(j, 1.0);
    }

    // Weld two seems to make a box
    dart::dynamics::BodyNode *top = mPinataWorld->getSkeleton(0)->getBodyNode("top");
    dart::dynamics::BodyNode *front = mPinataWorld->getSkeleton(0)->getBodyNode("front");
    dart::dynamics::BodyNode *back = mPinataWorld->getSkeleton(0)->getBodyNode("back");
    dart::constraint::WeldJointConstraint *joint1 = new dart::constraint::WeldJointConstraint(top, front);
    dart::constraint::WeldJointConstraint *joint2 = new dart::constraint::WeldJointConstraint(top, back);
    mPinataWorld->getConstraintSolver()->addConstraint(joint1);
    mPinataWorld->getConstraintSolver()->addConstraint(joint2);
}

MyWorld::~MyWorld() {
    for (int i = 0; i < mRigidBodies.size(); i++)
        delete mRigidBodies[i];

    mRigidBodies.clear();

    if (mCollisionDetector)
        delete mCollisionDetector;
}

void MyWorld::simulate() {
    mFrame++;

    /* **************************************************************** */
    // TODO: The skeleton code has provided the integration of position and linear momentum,
    // your first job is to fill in the integration of orientation and angular momentum.
    for (int i = 0; i < mRigidBodies.size(); i++) {
        // derivative of position and linear momentum
        Eigen::Vector3d dPos = mRigidBodies[i]->mLinMomentum / mRigidBodies[i]->mMass;
        // mRigidBodies[i]->mVelocity = mRigidBodies[i]->mLinMomentum / mRigidBodies[i]->mMass;
        Eigen::Vector3d dLinMom = mRigidBodies[i]->mMass * mGravity + mRigidBodies[i]->mAccumulatedForce;

        // derivative of quaternion orientation and angular momentum
        mRigidBodies[i]->mInertia = mRigidBodies[i]->mOrientation * mRigidBodies[i]->mIbody * (mRigidBodies[i]->mOrientation).transpose();
        mRigidBodies[i]->mOmega = mRigidBodies[i]->mInertia.inverse() * mRigidBodies[i]->mAngMomentum;  // angular velocity
        Eigen::Quaterniond qw;  qw.w() = 0.0;   qw.vec() = 0.5 * mRigidBodies[i]->mOmega; // omega to quaterniond qw
        Eigen::Quaterniond dQ = qw * mRigidBodies[i]->mQuatOrient;
        Eigen::Vector3d dAngMom = mRigidBodies[i]->mInertia * mGravity + mRigidBodies[i]->mAccumulatedTorque;

        // update position and linear momentum
        mRigidBodies[i]->mPosition += dPos * mTimeStep;
        mRigidBodies[i]->mLinMomentum += mTimeStep * dLinMom;

        // update orientation and angular momentum
        mRigidBodies[i]->mQuatOrient.w() += mTimeStep * dQ.w();
        mRigidBodies[i]->mQuatOrient.vec() += mTimeStep * dQ.vec();
        mRigidBodies[i]->mQuatOrient.normalize();
        mRigidBodies[i]->mOrientation = (mRigidBodies[i]->mQuatOrient).toRotationMatrix();
        mRigidBodies[i]->mAngMomentum += mTimeStep * dAngMom;
    }

    /* **************************************************************** */

    // Reset accumulated force and torque to be zero after a complete integration
    for (int i = 0; i < mRigidBodies.size(); i++) {
        mRigidBodies[i]->mAccumulatedForce.setZero();
        mRigidBodies[i]->mAccumulatedTorque.setZero();
    }

    // Apply external force to the pinata
    mPinataWorld->getSkeleton(0)->getBodyNode("bottom")->addExtForce(mForce);
    mForce.setZero();

    // Simulate Pinata using DART
    mPinataWorld->step();

    // Run collision detector
    mCollisionDetector->checkCollision();

    // TODO: implement a collision handler
    collisionHandling();

    // Break the pinata if it has enough momentum
    if (mPinataWorld->getSkeleton(0)->getCOMLinearVelocity().norm() > 0.6)
        mPinataWorld->getConstraintSolver()->removeAllConstraints();
}

// TODO: fill in the collision handling function
void MyWorld::collisionHandling() {
    // restitution coefficient
    double epsilon = 0.8;

    /* **************************************************************** */
    // TODO: handle the collision events

    int nContacts = mCollisionDetector->getNumContacts(); // Get the number of contacts at the current time step

    for (int i = 0; i < nContacts; i++) {
        // access data from collision detector
        point = mCollisionDetector->getContact(i).point; // contact point
        normal = mCollisionDetector->getContact(i).normal; // normal vector of contact point
        RigidBody *A = mCollisionDetector->getContact(i).rb1;
        RigidBody *B = mCollisionDetector->getContact(i).rb2;
        pVelocity = mCollisionDetector->getContact(i).pinataVelocity;

        // pre-impulse normal velocity
        double vN = normal.dot(deriveContactPoint(A) - deriveContactPoint(B));

        Eigen::Vector3d rA, rB;
        double partA, partB;

        if (A != NULL)
            rA = point - A->mPosition;

        if (B != NULL)
            rB = point - B->mPosition;

        // compute j
        j = -(1 + epsilon) * vN / (denominator(A) + denominator(B));

        // update linear and angular momentum
        updateMomentum(A);  updateMomentum(B);
    }
}

Eigen::Vector3d MyWorld::deriveContactPoint(RigidBody *_rb) {
    Vector3d dP;

    if (_rb != NULL) {
        RigidBody *rb = _rb;
        // compute rA, B is pinata
        rb->mR = point - rb->mPosition;
        // compute pre-impulse linear velocity
        rb->mVelocity = rb->mLinMomentum / rb->mMass;
        // compute pre-impulse angular velocity
        rb->mInertia = rb->mOrientation * rb->mIbody * (rb->mOrientation).transpose();
        rb->mOmega = rb->mInertia.inverse() * rb->mAngMomentum;
        // combine linear and angular velocity
        dP = rb->mVelocity + rb->mOmega.cross(rb->mR);
    } else {
        dP = pVelocity;
    }

    return dP;
}

double MyWorld::denominator(RigidBody *_rb) {
    double denom;

    if (_rb != NULL) {
        RigidBody *rb = _rb;
        denom = 1 / rb->mMass + normal.dot(rb->mInertia.inverse() * rb->mR.cross(normal).cross(rb->mR));
    } else {
        denom = 0.0;
    }

    return denom;
}


void MyWorld::updateMomentum(RigidBody *_rb) {
    if (_rb != NULL) {
        RigidBody *rb = _rb;
        rb->mLinMomentum += j * normal;
        rb->mAngMomentum += rb->mR.cross(j * normal);
    }
}








