#ifndef MANIFOLD_H
#define MANIFOLD_H

#include <vector>
#include "math.h"

struct Contact
{
    float b;

    Vector3 normal;
    float depth;

    Vector3 rel_pos_a;
    Vector3 rel_pos_b;

    float sum_impulse_contact;
    Vector3 sum_impulse_friction;
};

struct Manifold 
{
    RigidBody* body_a;
    RigidBody* body_b;

    bool collided;
    Vector3 normal;
    float depth;
    Vector3 mtv;
    Vector3 contact_a;
    Vector3 contact_b;
    int index_a;
    int index_b;

    std::vector<Vector3> cp;
    std::vector<Contact> contacts;
};

void add_contact(Manifold* manifold, Vector3 p)
{
    Vector3 r1 = p - manifold->body_a->position;
    Vector3 r2 = p - manifold->body_b->position;

    Contact contact;
    contact.rel_pos_a = r1;
    contact.rel_pos_b = r2;
    contact.normal = manifold->normal;
    contact.depth = manifold->depth;
    
    manifold->contacts.push_back(contact);
}

void update_contact(Manifold* m, Contact* c)
{
    c->sum_impulse_contact = 0;
    c->sum_impulse_friction = V3();
    c->b = 0;

    float baumgarte_scalar = 0.1f;
    float baumgarte_slop = 0.001f;
    float penetration_slop = min(c->depth + baumgarte_slop, 0);

    c->b += -(baumgarte_scalar / physics_dt) * penetration_slop;

    float elasticity = m->body_a->restitution * m->body_b->restitution;
    float elasticity_term = dot(c->normal, m->body_a->velocity + 
                            cross(c->rel_pos_a, m->body_a->angular_velocity) - 
                            m->body_b->velocity -
                            cross(c->rel_pos_b, m->body_b->angular_velocity));

    
    c->b += (elasticity * elasticity_term) / m->contacts.size();
}

void solve_contact_constraint(Manifold* m, Contact* c)
{
    Vector3 r1 = c->rel_pos_a;
    Vector3 r2 = c->rel_pos_b;

    //Resolution
#if 1
    Vector3 j1 = m->body_a->inverse_mass == 0 ? V3() : -c->normal;
    Vector3 j2 = m->body_a->inverse_mass == 0 ? V3() : cross(r1, c->normal);
    Vector3 j3 = m->body_b->inverse_mass == 0 ? V3() : c->normal;
    Vector3 j4 = m->body_b->inverse_mass == 0 ? V3() : -cross(r2, c->normal);
    
    float k = m->body_a->inverse_mass + dot(j2, m->body_a->inverse_inertia * j2) +
              m->body_b->inverse_mass + dot(j4, m->body_a->inverse_inertia * j4);

    if(k != 0)
    {
        float effective_mass = 1.0f / k;

        float jv = dot(j1, m->body_a->velocity) +
                   dot(j2, m->body_a->angular_velocity) +
                   dot(j3, m->body_b->velocity) +
                   dot(j4, m->body_b->angular_velocity);

        float beta = 0.5f;
        float restitution = m->body_a->restitution * m->body_b->restitution;
        Vector3 rel_vel = -m->body_a->velocity - cross(m->body_a->angular_velocity, r1) +
                           m->body_b->velocity + cross(m->body_b->angular_velocity, r2);
        
        float closing_vel = dot(rel_vel, c->normal);
        float b = -(beta / physics_dt) * c->depth + restitution * closing_vel;
        
        float lambda = effective_mass * (-(jv + b));
        lambda = max(lambda, 0);

        m->body_a->velocity += m->body_a->inverse_mass * j1 * lambda;
        m->body_b->velocity += m->body_b->inverse_mass * j3 * lambda;
        m->body_a->angular_velocity += m->body_a->inverse_inertia * j2 * lambda;
        m->body_b->angular_velocity += m->body_b->inverse_inertia * j4 * lambda;
    }

#else
    Vector3 va = m->body_a->velocity + cross(m->body_a->angular_velocity, r1);
    Vector3 vb = m->body_b->velocity + cross(m->body_b->angular_velocity, r2);

    Vector3 vab = vb - va;

    Vector3 mI = cross(m->body_a->inverse_inertia * cross(r1, c->normal), r1) + 
                 cross(m->body_b->inverse_inertia * cross(r2, c->normal), r2);

    float constraint_mass = m->body_a->inverse_mass + 
                            m->body_b->inverse_mass +
                            dot(c->normal, mI);

    if(constraint_mass > 0)
    {
        float jn = max(-dot(vab, c->normal) + c->b, 0.0f);
        jn = jn / constraint_mass;

        m->body_a->velocity -= c->normal * (jn * m->body_a->inverse_mass);
        m->body_b->velocity += c->normal * (jn * m->body_b->inverse_mass);

        m->body_a->angular_velocity -= m->body_a->inverse_inertia * cross(r1, c->normal * jn);
        m->body_b->angular_velocity += m->body_b->inverse_inertia * cross(r2, c->normal * jn);
    }

    //Friction
    Vector3 tangent = vab - c->normal * dot(vab, c->normal);
    float tangent_length = length(tangent);

    if(tangent_length > 1e-6f)
    {
        tangent = tangent / tangent_length;
        float d = dot(tangent, cross(m->body_a->inverse_inertia * cross(r1, tangent), r1) + cross(m->body_b->inverse_inertia * cross(r2, tangent), r2));

        float friction_mass = m->body_a->inverse_mass + m->body_b->inverse_mass + d;

        if(friction_mass > 0)
        {
            float friction_coeff = m->body_a->friction * m->body_b->friction;
            float jt = -dot(vab, tangent) * friction_coeff;
            jt = jt / friction_mass;

            m->body_a->velocity -= tangent * (jt * m->body_a->inverse_mass);
            m->body_b->velocity += tangent * (jt * m->body_b->inverse_mass);

            m->body_a->angular_velocity -= m->body_a->inverse_inertia * cross(r1, tangent * jt);
            m->body_b->angular_velocity += m->body_b->inverse_inertia * cross(r2, tangent * jt);
        }                      
    }
#endif
}

#endif