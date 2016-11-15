// some errors
/*
left/right thigh: segment fault 11
left/right pelvis: segment fault 11
middle abdomen: segment fault 11
left/right hands/bicep/scapula/head : many many errors
*/

// How to get the local coordinates of marker i?
Marker *mark = getMarker(i);
Vector3d pos = mark‐>getLocalCoords();

// How to get the current world coordinates of marker i?
Marker *mark = mSkel‐>getMarker(i); 
Vector3d pos = mark‐>getWorldCoords();

// How to access the bodynode where marker i resides?
Marker *mark = mSkel‐>getMarker(i); 
BodyNode *node = mark‐>getBodyNode();

// How to get the the joint between bodynode, bn, and its parent?
Joint *jt = bn‐>getParentJoint();

// How to get the the parent bodynode of bodynode, bn?
BodyNode *node = bn‐>getParentBodyNode();

// How to get the number of dofs in a joint, jt?
int nDofs = jt‐>getNumDofs;

// How to get the transformation chain from the root to bodynode bn?
Matrix4d mat = bn‐>getTransform().matrix();

// How to get the transformation chain from the parent of bodynode bn to itself?
Matrix4d mat = bn‐>getTransform(bn‐>getParentBodyNode()).matrix();

// How to get the transformation for j‐th dof in a joint jt?
Matrix4d mat = jt‐>getTransform(j).matrix();

// How to get the derivative matrix wrt j‐th dof in a joint jt?
Matrix4d mat = jt‐>getTransformDerivative(j);

// How to get the global index of the j‐th dof in a joint jt?
jt‐>getSkeletonIndex(j);

// How to get the transformation from the parent bodynode of a joint jt to itself?
Matrix4d parentToJoint = jt‐>getTransformFromParentBodyNode().matrix();

// How to get the transformation from the joint jt to its child bodynode?
Matrix4d jointToChild = jt‐>getTransformFromChildBodyNode().inverse().matrix();

// How to get the root bodynode of a skeleton, skel?
BodyNodePtr root = skel‐>getRootBodyNode();