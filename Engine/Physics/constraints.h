#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "../math.h"

struct DistanceConstraint
{
    RigidBody* body_a;
    RigidBody* body_b;

    float target_length;

    Vector3 rel_pos_a;
    Vector3 rel_pos_b;
};

DistanceConstraint make_distance_constraint(RigidBody* body_a, RigidBody* body_b, Vector3 global_a, Vector3 global_b)
{
    DistanceConstraint c = {};
    c.body_a = body_a;
    c.body_b = body_b;

    Vector3 ab = global_b - global_a;
    c.target_length = length(ab);

    Vector3 r1 = global_a - body_a->position;
    Vector3 r2 = global_b - body_b->position;

    c.rel_pos_a = transpose(to_mat3(body_a->orientation)) * r1;
    c.rel_pos_b = transpose(to_mat3(body_b->orientation)) * r2;

    return c;
}

#endif