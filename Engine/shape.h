#ifndef SHAPE_H
#define SHAPE_H

//#include <math.h>
#include "math.h"

enum ShapeType
{
    RECTANGLE,
    TRIANGLE,
    RIGHT_TRIANGLE,
    CIRCLE,
    CUSTOM,
};

//NOTE: Y axis positive is up
struct Shape 
{
    Vector3* local_vertices;
    Vector3* global_vertices;
    int vertices_count;

    Vector3 center_pos;
    Vector3 dim;
    
    float radius;

    //float orientation;
    ShapeType type;
};

struct SimplexPoint
{
    Vector3 p;
    Vector3 sa;
    Vector3 sb;  
};

struct Simplex
{
    SimplexPoint a, b, c;
    Vector3 dir;
};

struct Contact
{
    Vector3 global_a;
    Vector3 global_b;

    Vector3 local_a;
    Vector3 local_b;

    Vector3 normal;

    float depth;

    int index_a;
    int index_b;
};

struct Manifold 
{
    bool collided;
    Vector3 normal;
    float depth;
    Vector3 mtv;
    Vector3 contact_a;
    Vector3 contact_b;
    int index_a;
    int index_b;

    std::vector<Vector3> cp;
};

struct Edge
{
    int index;
    int other_index;

    Vector3 normal;
    float dist;
};

struct ClippingEdge
{
    Vector3 edge;
    Vector3 v1;
    Vector3 v2;
    Vector3 max;
};

Shape create_shape(Vector3 dim, ShapeType type = ShapeType::RECTANGLE, Vector3* vertices = 0);

void reset_shape_vertices(Shape* shape);
void update_shape(Shape* shape, Vector3 pos, Vector3 dim, Vector3 axis = {}, float angle = 0);

Vector3 get_furthest_vertex(Shape* shape);
Vector3 get_furthest_point_in_direction(Shape* shape, Vector3 dir);
float get_shape_radius(Shape* shape);

bool gjk_2d(Shape* shape_a, Shape* shape_b, Manifold* manifold);

Edge find_closest_edge(std::vector<SimplexPoint>* simplex);
void epa_2d(Simplex* s, Shape* shape_a, Shape* shape_b, Manifold* manifold);

Vector3 get_furthest_vertex(Shape* shape)
{
    float max = -FLT_MAX;
    int index = 0;

    for(int i = 0; i < shape->vertices_count; ++i)
    {
        float d = length(shape->global_vertices[i] - shape->center_pos);
        if(d > max)
        {
            max = d;
            index = i;
        }
    } 

    return shape->global_vertices[index];
}

Vector3 get_furthest_point_in_direction(Shape* shape, Vector3 dir)
{
    float max = -FLT_MAX;
    u32 index = 0;

    for(int i = 0; i < shape->vertices_count; ++i)
    {
        float dot_product = dot(shape->global_vertices[i], dir);
        if(dot_product > max)
        {
            max = dot_product;
            index = i;
        }
    }

    return shape->global_vertices[index];
}

int get_furthest_point_index_in_direction(Shape* shape, Vector3 dir)
{
    float max = -FLT_MAX;
    u32 index = 0;

    for(int i = 0; i < shape->vertices_count; ++i)
    {
        float dot_product = dot(shape->global_vertices[i], dir);
        if(dot_product > max)
        {
            max = dot_product;
            index = i;
        }
    }

    return index;
}

float get_shape_radius(Shape* shape)
{
    return length(get_furthest_vertex(shape) - shape->center_pos);
}

Shape create_shape(Vector3 dim, ShapeType type, Vector3* vertices)
{
    Shape shape = {};
    shape.type = type;
    shape.dim = dim;

    switch(shape.type)
    {
        case ShapeType::RECTANGLE:
        {
            //NOTE: Vertices order start from top left, clockwise
            shape.vertices_count = 4;
            shape.local_vertices = (Vector3*)malloc(sizeof(Vector3) * shape.vertices_count);
            shape.global_vertices = (Vector3*)malloc(sizeof(Vector3) * shape.vertices_count);
            shape.local_vertices[0] = V3(-0.5f, 0.5f, 0);
            shape.local_vertices[1] = V3(0.5f, 0.5f, 0);
            shape.local_vertices[2] = V3(0.5f, -0.5f, 0);
            shape.local_vertices[3] = V3(-0.5f, -0.5f, 0);
        } break;
        case ShapeType::TRIANGLE:
        {
            /* NOTE: Vertices order
                1
                *   *
                        *
                *           *
                                *
                *                   2
                                *
                *           *
                        *    
                *   *
                3 
            */
            shape.vertices_count = 3;
            shape.local_vertices = (Vector3*)malloc(sizeof(Vector3) * shape.vertices_count);
            shape.global_vertices = (Vector3*)malloc(sizeof(Vector3) * shape.vertices_count);
            shape.local_vertices[0] = V3(-0.5f, 0.5f, 0);
            shape.local_vertices[1] = V3(0.5f, 0, 0);
            shape.local_vertices[2] = V3(-0.5f, -0.5f, 0);
        } break;
        case ShapeType::RIGHT_TRIANGLE:
        {
            /* NOTE: Vertices order
                1
                * *
                *  *
                *   *
                *    *
                3 * * 2 
            */
            shape.vertices_count = 3;
            shape.local_vertices = (Vector3*)malloc(sizeof(Vector3) * shape.vertices_count);
            shape.global_vertices = (Vector3*)malloc(sizeof(Vector3) * shape.vertices_count);
            shape.local_vertices[0] = V3(-0.5f, 0.5f, 0);
            shape.local_vertices[1] = V3(0.5f, -0.5f, 0);
            shape.local_vertices[2] = V3(-0.5f, -0.5f, 0);
        } break;
        case ShapeType::CUSTOM:
        {
            shape.local_vertices = vertices;
        } break;
    }

    return shape;
}

void reset_shape_vertices(Shape* shape)
{
    //TODO: Handle custom shapes
    switch(shape->type)
    {
        case ShapeType::RECTANGLE:
        {
            shape->local_vertices[0] = V3(-0.5f, 0.5f, 0);
            shape->local_vertices[1] = V3(0.5f, 0.5f, 0);
            shape->local_vertices[2] = V3(0.5f, -0.5f, 0);
            shape->local_vertices[3] = V3(-0.5f, -0.5f, 0);
        } break;
        case ShapeType::TRIANGLE:
        {
            shape->local_vertices[0] = V3(-0.5f, 0.5f, 0);
            shape->local_vertices[1] = V3(0.5f, 0, 0);
            shape->local_vertices[2] = V3(-0.5f, -0.5f, 0);
        } break;
        case ShapeType::RIGHT_TRIANGLE:
        {
            shape->local_vertices[0] = V3(-0.5f, 0.5f, 0);
            shape->local_vertices[1] = V3(0.5f, -0.5f, 0);
            shape->local_vertices[2] = V3(-0.5f, -0.5f, 0);
        } break;
    }
}

void update_shape(Shape* shape, Vector3 pos, Vector3 dim, Vector3 axis, float angle)
{
    if (shape->vertices_count)
    {
        Mat4 transform = mat4_identity();
        transform = mat4_scale(transform, dim);
        if(angle) transform = mat4_rotate(transform, axis, angle);
        transform = mat4_translate(transform, pos);

        for(int i = 0; i < shape->vertices_count; ++i)
        {
            Vector4 vertex = V4(shape->local_vertices[i], 1);
            vertex = transform * vertex;
            shape->global_vertices[i] = {vertex.x, vertex.y, vertex.z};
        }
    }

    shape->radius = get_shape_radius(shape);
    shape->center_pos = pos;
    shape->dim = dim;
}

void update_shape(Shape* shape, Vector3 pos, Quaternion q)
{
    if(shape->vertices_count)
    {
        Mat4 transform = mat4_identity();
        transform = mat4_scale(transform, shape->dim);
        transform = transform * to_mat4(q);
        transform = mat4_translate(transform, pos);

        for(int i = 0; i < shape->vertices_count; ++i)
        {
            Vector4 vertex = V4(shape->local_vertices[i], 1);
            vertex = transform * vertex;
            shape->global_vertices[i] = {vertex.x, vertex.y, vertex.z};
        }
    }
}

SimplexPoint support_point(Shape* shape_a, Shape* shape_b, Vector3 dir)
{
    SimplexPoint point = {};
    point.sa = get_furthest_point_in_direction(shape_a, dir);
    point.sb = get_furthest_point_in_direction(shape_b, -dir);
    point.p = point.sa - point.sb;

    return point;
}

//NOTE: Reference https://dyn4j.org/2010/04/gjk-gilbert-johnson-keerthi/
bool gjk_2d(Shape* shape_a, Shape* shape_b, Manifold* manifold)
{
    Simplex s;
    s.dir = shape_b->center_pos - shape_a->center_pos;
    s.c = support_point(shape_a, shape_b, s.dir);
    s.dir = -s.c.p;

    s.b = support_point(shape_a, shape_b, s.dir);
    if(dot(s.b.p, s.dir) < 0)
    {
        *manifold = {};
        return false;
    }

    Vector3 bc = s.c.p - s.b.p;
    s.dir = cross(cross(bc, -s.b.p), bc);

    if(length(s.dir) == 0)
    {
        s.dir = cross(bc, V3(1, 0, 0));
        if(length(s.dir) == 0)
        {
            s.dir = cross(bc, V3(0, 0, -1));
        }
    }

    int max_iteration = 0;
    while(max_iteration < 50)
    {
        s.a = support_point(shape_a, shape_b, s.dir);
        if(dot(s.a.p, s.dir) < 0)
        {
            manifold->collided = false;
            manifold->mtv = {};
            return false;
        }

        Vector3 ao = -s.a.p;
        Vector3 ab = s.b.p - s.a.p;
        Vector3 ac = s.c.p - s.a.p;

        Vector3 ac_perp = triple_product(ab, ac, ac);

        if(dot(ac_perp, ao) >= 0)
        {
            s.dir = ac_perp;
        }
        else
        {
            Vector3 ab_perp = triple_product(ac, ab, ab);

            if(dot(ab_perp, ao) < 0)
            {
                manifold->collided = true;
                //epa
                epa_2d(&s, shape_a, shape_b, manifold);
                return true;
            }

            s.c = s.b;

            s.dir = ab_perp;
        }

        s.b = s.a;
        ++max_iteration;
    }

    //NOTE: if iteration is above 50 then we assume there's something went wrong and there's no collision
    *manifold = {};
    return false;
}

Edge find_closest_edge(std::vector<SimplexPoint>* simplex)
{
    Edge edge = {};

    edge.dist = FLT_MAX;

    for(int i = 0; i < simplex->size(); ++i)
    {
        int j = i + 1 == simplex->size() ? 0 : i+1;

        Vector3 a = simplex->at(i).p;
        Vector3 b = simplex->at(j).p;

        Vector3 e = b - a;
        Vector3 oa = a;

        Vector3 e_perp = V3(perp(e.xy), 0);
        Vector3 n = normalize(e_perp);

        assert(!(n == V3()));

        float dist = dot(n, a);
        if(dist < edge.dist)
        {
            edge.dist = dist;
            edge.index = j;
            edge.other_index = i;
            edge.normal = n;
        }
    }

    return edge;
}

//NOTE: https://dyn4j.org/2010/05/epa-expanding-polytope-algorithm/
//      https://dyn4j.org/2010/04/gjk-distance-closest-points/

#define EPA_MAX_ITERATION 64
#define EPA_TOLERANCE 0.0001f

void epa_2d(Simplex* s, Shape* shape_a, Shape* shape_b, Manifold* manifold)
{
    std::vector<SimplexPoint> simplex;
    simplex.push_back(s->c);
    simplex.push_back(s->b);
    simplex.push_back(s->a);

    int max_iteration = 0;

    while(max_iteration < EPA_MAX_ITERATION)
    {
        Edge edge = find_closest_edge(&simplex);
        SimplexPoint new_point = support_point(shape_a, shape_b, edge.normal);
        float dot_product = dot(new_point.p, edge.normal);

        if(dot_product - edge.dist < EPA_TOLERANCE)
        {
            manifold->normal = -edge.normal;
            manifold->depth = dot_product;

            Vector3 l = simplex[edge.index].p - simplex[edge.other_index].p;

            if(l == V3())
            {
                manifold->contact_a = simplex[edge.other_index].sa;
                manifold->contact_b = simplex[edge.other_index].sa;
            }
            else
            {
                float ll = dot(l, l);
                float al = dot(simplex[edge.other_index].p, l);

                float lambda2 = -al / ll;
                float lambda1 = 1 - lambda2;

                if(lambda1 < 0)
                {
                    manifold->contact_a = simplex[edge.index].sa;
                    manifold->contact_b = simplex[edge.index].sb;
                } 
                else if(lambda2 < 0)
                {
                    manifold->contact_a = simplex[edge.other_index].sa;
                    manifold->contact_b = simplex[edge.other_index].sb;
                }
                else
                {
                    manifold->contact_a = lambda1 * simplex[edge.other_index].sa + lambda2 * simplex[edge.index].sa;
                    manifold->contact_b = lambda1 * simplex[edge.other_index].sb + lambda2 * simplex[edge.index].sb;
                }
            }

            manifold->mtv = manifold->normal * manifold->depth;

            break;
        }
        else
        {
            simplex.insert(simplex.begin() + edge.index, new_point);
        }
    }
} 

std::vector<Vector3> generate_test_axes(Shape* shape)
{
    std::vector<Vector3> axes;

    for(int i = 0; i < shape->vertices_count; ++i)
    {
        Vector3 p1 = shape->global_vertices[i];
        Vector3 p2 = shape->global_vertices[i + 1 == shape->vertices_count ? 0 : i + 1];

        Vector3 edge = p2 - p1;
        Vector3 n = V3(perp(edge.xy));
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

ClippingEdge find_best_edge(Shape* shape, Vector3 n)
{
    ClippingEdge result = {};
    int index = get_furthest_point_index_in_direction(shape, n);
    int index_left = index - 1 < 0 ? shape->vertices_count-1 : index - 1;
    int index_right = index + 1 == shape->vertices_count ? 0 : index + 1;

    result.max = shape->global_vertices[index];

    Vector3 v = shape->global_vertices[index]; 
    Vector3 v_left = shape->global_vertices[index_left]; 
    Vector3 v_right = shape->global_vertices[index_right]; 

    Vector3 l = normalize(v - v_left);
    Vector3 r = normalize(v - v_right);

    if(dot(r, n) <= dot(l, n))
    {
        result.edge = r;
        result.v1 = v;
        result.v2 = v_right;
        return result;
    }
    else
    {
        result.edge = l;
        result.v1 = v_left;
        result.v2 = v;
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

std::vector<Vector3> generate_contacts(Shape* shape_a, Shape* shape_b, Vector3 normal)
{
    std::vector<Vector3> cp;
    ClippingEdge e1 = find_best_edge(shape_a, -normal);
    ClippingEdge e2 = find_best_edge(shape_b, normal);
    
    ClippingEdge ref, inc;
    bool flip = false;
    if(abs(dot(e1.edge, normal)) <= abs(dot(e2.edge, normal)))
    {
        ref = e2;
        inc = e1;
    }
    else
    {
        ref = e1;
        inc = e2;
        flip = true;
    }

    Vector3 refv = normalize(e1.edge);

    float o1 = dot(refv, ref.v1);
    cp = clip(inc.v1, inc.v2, refv, o1);
    if(cp.size() < 2) return {};

    float o2 = dot(refv, ref.v2);
    cp = clip(cp[0], cp[1], -refv, -o2);
    if(cp.size() < 2) return {};

    Vector3 ref_n = V3(perp(ref.edge.xy));
    if(flip) ref_n = -ref_n;

    float max = dot(ref_n, ref.max);

    if(dot(ref_n, cp[1]) - max < 0)
    {
        cp.erase(cp.begin() + 1);
    }
    if(dot(ref_n, cp[0]) - max < 0)
    {
        cp.erase(cp.begin());
    }

    return cp;
}

bool test_SAT(Shape* shape_a, Shape* shape_b, Manifold* manifold) 
{
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
                smallest = projection_a.y > projection_b.y ? axes_a[i] : -axes_a[i];
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
                smallest = projection_a.y > projection_b.y ? axes_b[i] : -axes_b[i];
            }
        }
    }

    manifold->normal = smallest;
    manifold->depth = overlap;
    manifold->mtv = smallest * overlap;
    manifold->cp = generate_contacts(shape_a, shape_b, manifold->normal);

    return true;
}

#endif 