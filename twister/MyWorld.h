#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>
#include "dart/dart.h"


class MyWorld {
  public:
    MyWorld();
    virtual ~MyWorld();
    dart::dynamics::SkeletonPtr getSkel() {
        return mSkel;
    }

    void solve();
    void createConstraint(int _index);
    void modifyConstraint(int _index, Eigen::Vector3d _deltaP);
    void removeConstraint(int _index);
    dart::dynamics::Marker *getMarker(int _index);

  protected:
    Eigen::VectorXd updateGradients();
    void createMarkers();

    dart::dynamics::SkeletonPtr mSkel;
    std::vector<dart::dynamics::Marker *> mMarkers;
    int size;   // total number of markers
    std::vector<Eigen::Vector3d> mC;
    std::vector<Eigen::MatrixXd> mJ;
    std::vector<Eigen::Vector3d> mTarget; // The target location of the constriant
    int *mConstrainedMarker; // The index of the constrained marker
};

#endif
