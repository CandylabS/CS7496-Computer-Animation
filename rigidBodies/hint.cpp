// Get the number of contacts at the current time step
int nContacts = mCollisionDetector‐>getNumContacts();

// Get the world coordinate of the first contact point at the current time step   
Vector3d point = mCollisionDetector‐>getContact(0).point;

// Get the normal vector of the third contact point at the current time step. The vector is expressed in the world space.
Vector3d normal = mCollisionDetector‐>getContact(2).normal;

// Get the pointer to the rigid body A involved in the second contact. If A is a null pointer, that means it is the pinata. Otherwise, it will return a non‐null pointer.
RigidBody* A = mCollisionDetector‐>getContact(1).rb1;

// Get the pointer to the rigid body B involved in the second contact. If B is a null pointer, that means it is the pinata. Otherwise, it will return a non‐null pointer.
RigidBody* B = mCollisionDetector‐>getContact(1).rb2;

// Get the velocity of the colliding point on the pinata in the world space. If neither rb1 nor rb2 is the pinata, it will return (0, 0, 0).
Vector3d pVelocity = mCollisionDetector‐>getContact(0).pinataVelocity;


/******* About quaternions *******/
// Eigen library provides the data type of Quaternion and many useful operators. For example:
// Create an identify quaternion 
Eigen::Quaterniond q1; 
q1.setIdentity();

// Convert a quaternion to a rotation matrix 
Eigen::Matrix3d rotMatrix = q.toRotationMatrix();

// Normalize a quaternion 
q.normalize();
 
// Access the scalar part and the vector part of the quaternion 
double w = q.w();
Eigen::Vector3d v = q.vec();

// Add two quaternions 
Eigen::Quaterniond sum; 
sum.w() = q1.w() + q2.w(); 
sum.vec() = q1.vec() + q2.vec();

// Multiply a scalar with a quaternion 
Eigen::Quaterniond scaledQ; 
scaledQ.w() = 2.0 * q.w(); 
scaledQ.vec() = 2.0 * w.vec();
