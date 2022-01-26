#include "physics.h"

RigidBody create_body(Vector3 p, Vector3 v, float mass)
{
    RigidBody body = {};
    body.position = p;
    body.velocity = v;
    body.inverse_mass = 1.0f / mass;
    body.inverse_inertia = mat3_identity();

    return body;
}

void set_gravity(Vector3 g)
{
    physics_gravity = g;
}

void set_damping_factor(float k)
{
    physics_damping_factor = k;
}

void integrate_for_velocity(RigidBody* body, float dt)
{
    if(body->inverse_mass > 0)
    {
        body->velocity += physics_gravity * dt;
    }

    body->velocity += body->force * body->inverse_mass * dt;
    body->velocity = body->velocity * physics_damping_factor;

    body->angular_velocity += body->inverse_inertia * body->torgue * dt;
    body->angular_velocity = body->angular_velocity * physics_damping_factor;
}

void integrate_for_position(RigidBody* body, float dt)
{
    body->position += body->velocity * dt;
    body->orientation = body->orientation + make_quaternion(body->angular_velocity * dt * 0.5f, 0.0f);

    body->orientation = normalize(body->orientation);
}