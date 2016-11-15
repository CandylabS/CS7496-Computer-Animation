#include "CollisionInterface.h"
#include "RigidBody.h"
#include <iostream>

using namespace Eigen;

CollisionInterface::CollisionInterface() {
    mCollisionChecker = new dart::collision::DARTCollisionDetector();
    mCollisionChecker->setNumMaxContacs(10);
}

CollisionInterface::~CollisionInterface() {
    if (mCollisionChecker)
        delete mCollisionChecker;
}


void CollisionInterface::addSkeleton(dart::dynamics::SkeletonPtr _skel) {
    int nNodes = _skel->getNumBodyNodes();

    for (int i = 0; i < nNodes; i++) {
        dart::dynamics::BodyNode *bn = _skel->getBodyNode(i);
        mCollisionChecker->addCollisionSkeletonNode(bn);
        mNodeMap[_skel->getBodyNode(i)] = NULL;
    }
}

void CollisionInterface::addRigidBody(RigidBody *_rb, const std::string &name) {
    dart::dynamics::SkeletonPtr skel = dart::dynamics::Skeleton::create(name);
    dart::dynamics::FreeJoint::Properties properties;
    properties.mName = "freeJoint";
    dart::dynamics::BodyNode *bn = skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
                                       nullptr, properties, dart::dynamics::BodyNode::Properties()).second;
    bn->addCollisionShape(_rb->mShape);
    bn->setCollidable(true);
    mCollisionChecker->addCollisionSkeletonNode(bn);
    mSkeletons.push_back(skel);
    mNodeMap[bn] = _rb;
}

void CollisionInterface::checkCollision() {
    updateBodyNodes();
    mCollisionChecker->detectCollision(true, true);
    postProcess();
}

void CollisionInterface::updateBodyNodes() {
    for (std::map<dart::dynamics::BodyNode *, RigidBody *>::iterator it = mNodeMap.begin(); it != mNodeMap.end(); ++it) {
        dart::dynamics::BodyNode *bn = it->first;
        RigidBody *rb = it->second;

        if (bn->getSkeleton()->getName() == "pinata")
            continue;

        Isometry3d W;
        W.setIdentity();
        W.linear() = rb->mOrientation;
        W.translation() = rb->mPosition;
        W.makeAffine();
        bn->getSkeleton()->getJoint("freeJoint")->setTransformFromParentBodyNode(W);
        bn->getSkeleton()->computeForwardKinematics(true, false, false);
    }
}

void CollisionInterface::postProcess() {
    mContacts.clear();
    int numContacts = mCollisionChecker->getNumContacts();

    mContacts.resize(numContacts);

    for (int i = 0; i < numContacts; i++) {
        mContacts[i].point = mCollisionChecker->getContact(i).point;
        mContacts[i].normal = mCollisionChecker->getContact(i).normal;
        mContacts[i].rb1 = mNodeMap[mCollisionChecker->getContact(i).bodyNode1.lock().get()];
        mContacts[i].rb2 = mNodeMap[mCollisionChecker->getContact(i).bodyNode2.lock().get()];

        if (mContacts[i].rb1 == NULL) {
            dart::dynamics::BodyNode *bd = mCollisionChecker->getContact(i).bodyNode1.lock().get();
            Vector3d localPoint = bd->getTransform().inverse() * mContacts[i].point;
            mContacts[i].pinataVelocity = bd->getLinearVelocity(localPoint);
        } else if (mContacts[i].rb2 == NULL) {
            dart::dynamics::BodyNode *bd = mCollisionChecker->getContact(i).bodyNode2.lock().get();
            Vector3d localPoint = bd->getTransform().inverse() * mContacts[i].point;
            mContacts[i].pinataVelocity = bd->getLinearVelocity(localPoint);
        } else {
            mContacts[i].pinataVelocity.setZero();
        }
    }
}
