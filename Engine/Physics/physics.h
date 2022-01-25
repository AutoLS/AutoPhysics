#ifndef PHYSICS_H
#define PHYSICS_H

struct RigidBody
{
    Vector3 position;
    Vector3 velocity;
    Vector3 force;
    
    float inverse_mass;
    
    Quaternion orientation;
    Vector3 angular_velocity;
    Vector3 torgue;
    Mat3 inverse_inertia;

    float restitution;
    float friction; 
};

#endif 