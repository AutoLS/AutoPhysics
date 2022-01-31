#ifndef PHYSICS_H
#define PHYSICS_H

#include "../math.h"
#include "../shape.h"

static float physics_current_time = 0.0f;
static float physics_time_accumlator = 0;
static float physics_dt = 1.0f / 60.0f;

static Vector3 physics_gravity = {0, -98, 0};
static float physics_damping_factor = 0.95f;

struct RigidBody
{
    Shape shape;

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

    bool freeze_orientation;
};

#include "constraints.h"
#include "sat.h"

RigidBody create_body(Shape shape, Vector3 p, Vector3 v, float mass);

void set_gravity(Vector3 g);
void set_damping_factor(float k);

void integrate_for_velocity(RigidBody* body, float dt);
void integrate_for_position(RigidBody* body, float dt);
void apply_impulse(DistanceConstraint* c);

#endif 