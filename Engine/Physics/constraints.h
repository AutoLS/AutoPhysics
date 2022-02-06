#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "../math.h"

enum ConstraintType
{
    CONTACT,
    DISTANCE
};

struct DistanceConstraint
{
    RigidBody* body_a;
    RigidBody* body_b;

    float target_length;

    Vector3 rel_pos_a;
    Vector3 rel_pos_b;
};

struct Constraint
{
    ConstraintType type;
    void* constraint;
};

DistanceConstraint set_distance_constraint(RigidBody* body_a, RigidBody* body_b, Vector3 global_a, Vector3 global_b)
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

Constraint create_distance_constraint(RigidBody* body_a, RigidBody* body_b, Vector3 global_a, Vector3 global_b)
{
    Constraint c = {};
    c.type = ConstraintType::DISTANCE;
    c.constraint = malloc(sizeof(DistanceConstraint));
    *(DistanceConstraint*)c.constraint = set_distance_constraint(body_a, body_b, global_a, global_b);

    return c;
}

void destroy_constraint(Constraint* c)
{
    if(c->constraint != 0)
    {
        free(c->constraint);
    }
}

#endif