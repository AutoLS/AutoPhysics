#ifndef CONSTRAINT_H
#define CONSTRAINT_H

struct Constraint
{
    RigidBody* body_a;
    RigidBody* body_b;

    float target_length;

    Vector3 rel_pos_a;
    Vector3 rel_pos_b;
};

#endif