#include "physics.h"

RigidBody create_body(Shape shape, Vector3 p, Vector3 v, float mass)
{
    RigidBody body = {};
    body.position = p;
    body.velocity = v;
    body.orientation = make_quaternion({}, 1.0f);
    body.inverse_mass = mass > 0 ? 1.0f / mass : 0;
    Mat3 inertia_tensor = {};

    if(mass != 0)
    {
        float oneTwelve = 1.0f / 12.0f;
        float xx = shape.dim.x * shape.dim.x;
        float yy = shape.dim.y * shape.dim.y;
        inertia_tensor._11 = oneTwelve * mass * (yy);
        inertia_tensor._22 = oneTwelve * mass * (xx);
        inertia_tensor._33 = oneTwelve * mass * (xx + yy);
        body.inverse_inertia = inverse(inertia_tensor);
    }
    
    body.shape = shape;
    update_shape(&body.shape, p, body.orientation);

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
    body->velocity.x = body->velocity.x * physics_damping_factor;

    if(!body->freeze_orientation)
    {
        body->angular_velocity += body->inverse_inertia * body->torgue * dt;
        body->angular_velocity = body->angular_velocity * physics_damping_factor;
    }
}

void integrate_for_position(RigidBody* body, float dt)
{
    body->position += body->velocity * dt;
    
    if(!body->freeze_orientation)
    {
        body->orientation = body->orientation + make_quaternion(body->angular_velocity * dt * 0.5f, 0) * body->orientation;
    }
    update_shape(&body->shape, body->position, body->orientation);
    body->orientation = normalize(body->orientation);
}

void solve_distance_constraint(DistanceConstraint* c, float dt)
{
/*NOTE: 
    Generalized velocity constraint is JV + b = 0
    where J is the jacobian, V is the velocity and b is the bias

    M*deltaV = L * lambda is the impulse equation
    From the plane equation we know that L is equal to Jt
    After some algebra manipulation we get these
    DeltaV = V2 - V1 = Minv * Jt * lambda
    lambda = -(JV1 + b) / (J * Minv * Jt)
    b = C * B/h where h is deltaT and B is Beta, C is the constraint 

    Distance constraint is C' = −d · v1 + −(r1 × d) · ω1 + d · v2 + (r2 × d) · ω2
    J = [-d -(r1 x d) d (r2 x d)]
    V = [v1 w1 v2 w2]
*/
    RigidBody* body_a = c->body_a;
    RigidBody* body_b = c->body_b;

    Vector3 r1 = to_mat3(body_a->orientation) * c->rel_pos_a;
    Vector3 r2 = to_mat3(body_b->orientation) * c->rel_pos_b;

    Vector3 global_a = r1 + body_a->position;
    Vector3 global_b = r2 + body_b->position;

    Vector3 ab = global_b - global_a;
    Vector3 n = normalize(ab);

    Vector3 vel_a = body_a->velocity + cross(body_a->angular_velocity, r1);
    Vector3 vel_b = body_b->velocity + cross(body_b->angular_velocity, r2);

    float rel_vel = dot(vel_a - vel_b, n); 

    float inverse_constraint_mass = body_a->inverse_mass + body_b->inverse_mass;
    float inverse_constraint_inertia = dot(n, 
    cross(body_a->inverse_inertia * cross(r1, n), r1) + 
    cross(body_b->inverse_inertia * cross(r2, n), r2));

    float constraint_mass = inverse_constraint_mass + inverse_constraint_inertia;

    if(constraint_mass > 0)
    {
        float b = 0.0f;
        float distance_offset = length(ab) - c->target_length;
        float baumgarte_scalar = 0.1f;
        b = -(baumgarte_scalar / dt) * distance_offset;

        float jn = -(rel_vel + b) / constraint_mass;

        body_a->velocity += n * (body_a->inverse_mass * jn);
        body_b->velocity -= n * (body_b->inverse_mass * jn);
        
        if(!body_a->freeze_orientation)
            body_a->angular_velocity += (body_a->inverse_inertia * cross(r1, n * jn));
        if(!body_b->freeze_orientation)
            body_b->angular_velocity += (body_b->inverse_inertia * cross(r2, n * jn));
    }
}

void apply_impulse(Constraint* c, float dt)
{
    switch(c->type)
    {
        case ConstraintType::DISTANCE:
        {
            solve_distance_constraint((DistanceConstraint*)c->constraint, dt);
        } break;
    }
}
