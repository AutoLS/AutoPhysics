#ifndef SAT_H
#define SAT_H

#include "../shape.h"

struct ClippingEdge
{
    Vector3 edge;
    Vector3 v1;
    Vector3 v2;
    Vector3 max;
};

std::vector<Vector3> generate_test_axes(Shape* shape)
{
    std::vector<Vector3> axes;

    for(int i = 0; i < shape->vertices_count; ++i)
    {
        Vector3 p1 = shape->global_vertices[i];
        Vector3 p2 = shape->global_vertices[i + 1 == shape->vertices_count ? 0 : i + 1];

        Vector3 edge = p2 - p1;
        Vector3 n = V3(cross(edge.xy, -1));
        axes.push_back(normalize(n));
    }

    return axes;
}

Vector2 project_to_axis(Vector3 axis, Shape* shape)
{
    float min = dot(axis, shape->global_vertices[0]);
    Vector2 result = {min, min};
    for(int i = 1; i < shape->vertices_count; ++i)
    {
        float dot_product = dot(axis, shape->global_vertices[i]);
        if(dot_product < result.x)
        {
            result.x = dot_product;
        }
        else if(dot_product > result.y)
        {
            result.y = dot_product;
        }
    }

    return result;
}

bool is_projected_overlap(Vector2 a, Vector2 b)
{
    return !(b.y < a.x || a.y < b.x); 
}

float get_overlap_from_projection(Vector2 a, Vector2 b)
{
    return min(a.y, b.y) - max(a.x, b.x);
}

//NOTE: Winding order is counter clockwise
ClippingEdge find_best_edge(Shape* shape, Vector3 n)
{
    ClippingEdge result = {};
    int index = get_furthest_point_index_in_direction(shape, n);
    int index_prev = index - 1 < 0 ? shape->vertices_count-1 : index - 1;
    int index_next = index + 1 == shape->vertices_count ? 0 : index + 1;

    result.max = shape->global_vertices[index];

    Vector3 v = shape->global_vertices[index]; 
    Vector3 v_next = shape->global_vertices[index_next]; 
    Vector3 v_prev = shape->global_vertices[index_prev]; 

    Vector3 l = normalize(v - v_next);
    Vector3 r = normalize(v - v_prev);

    if(dot(r, n) <= dot(l, n))
    {
        result.v1 = v_prev;
        result.v2 = v;
        result.edge = result.v2 - result.v1;
        return result;
    }
    else
    {
        result.v1 = v;
        result.v2 = v_next;
        result.edge = result.v2 - result.v1;
        return result;
    }
}

std::vector<Vector3> clip(Vector3 v1, Vector3 v2, Vector3 n, float o)
{
    std::vector<Vector3> cp;
    float d1 = dot(n, v1) - o;
    float d2 = dot(n, v2) - o;

    if(d1 >= 0) cp.push_back(v1);
    if(d2 >= 0) cp.push_back(v2);

    if(d1 * d2 < 0)
    {
        Vector3 e = v2 - v1;
        float u = d1 / (d1 - d2);
        e = e * u;
        e += v1;

        cp.push_back(e);
    }

    return cp;
}

std::vector<Vector3> generate_contact_points(Shape* shape_a, Shape* shape_b, Vector3 normal)
{
    std::vector<Vector3> cp;
    ClippingEdge e1 = find_best_edge(shape_a, normal);
    ClippingEdge e2 = find_best_edge(shape_b, -normal);
    
    ClippingEdge ref, inc;
    bool flip = false;
    if(abs(dot(e1.edge, normal)) <= abs(dot(e2.edge, normal)))
    {
        ref = e1;
        inc = e2;
    }
    else
    {
        ref = e2;
        inc = e1;
        flip = true;
    }

    Vector3 refv = normalize(ref.edge);

    float o1 = dot(refv, ref.v1);
    cp = clip(inc.v1, inc.v2, refv, o1);
    if(cp.size() < 2) return {};

    float o2 = dot(refv, ref.v2);
    cp = clip(cp[0], cp[1], -refv, -o2);
    if(cp.size() < 2) return {};

    //NOTE: if we flipped we have to use the left hand orthogonal vector otherwise use right hand
    Vector3 ref_n = flip ? V3(reverse_perp(refv.xy)) : V3(perp(refv.xy));

    if(flip) ref_n = -ref_n;

    float max = dot(ref_n, ref.max);

    float d0 = dot(ref_n, cp[0]);
    float d1 = dot(ref_n, cp[1]);

    if(d1 - max < 0)
    {
        cp.erase(cp.begin() + 1);
    }
    if(d0 - max < 0)
    {
        cp.erase(cp.begin());
    }

    return cp;
}

bool test_SAT(RigidBody* body_a, RigidBody* body_b, Manifold* manifold) 
{
    Shape* shape_a = &body_a->shape; 
    Shape* shape_b = &body_b->shape;

    float overlap = FLT_MAX;
    Vector3 smallest;

    std::vector<Vector3> axes_a = generate_test_axes(shape_a);
    std::vector<Vector3> axes_b = generate_test_axes(shape_b);

    for(int i = 0; i < axes_a.size(); ++i)
    {
        Vector2 projection_a = project_to_axis(axes_a[i], shape_a);
        Vector2 projection_b = project_to_axis(axes_a[i], shape_b);

        if(!is_projected_overlap(projection_a, projection_b))
        {
            return false;
        }
        else
        {
            float o = get_overlap_from_projection(projection_a, projection_b);
            if(o < overlap)
            {
                overlap = o;
                smallest = projection_a.y > projection_b.y ? -axes_a[i] : axes_a[i];
            }
        }
    }

    for(int i = 0; i < axes_b.size(); ++i)
    {
        Vector2 projection_a = project_to_axis(axes_b[i], shape_a);
        Vector2 projection_b = project_to_axis(axes_b[i], shape_b);

        if(!is_projected_overlap(projection_a, projection_b))
        {
            return false;
        }
        else
        {
            float o = get_overlap_from_projection(projection_a, projection_b);
            if(o < overlap)
            {
                overlap = o;
                smallest = projection_a.y > projection_b.y ? -axes_b[i] : axes_b[i];
            }
        }
    }

    manifold->body_a = body_a;
    manifold->body_b = body_b;
    manifold->normal = smallest;
    manifold->depth = overlap;
    manifold->mtv = smallest * overlap;
    manifold->cp = generate_contact_points(shape_a, shape_b, manifold->normal);
    for(int i = 0; i < manifold->cp.size(); ++i)
    {
        add_contact(manifold, manifold->cp[i]);
    }

    return true;
}

#endif 