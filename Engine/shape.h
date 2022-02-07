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

Shape create_shape(Vector3 dim, ShapeType type = ShapeType::RECTANGLE, Vector3* vertices = 0);

void reset_shape_vertices(Shape* shape);
void update_shape(Shape* shape, Vector3 pos, Vector3 dim, Vector3 axis = {}, float angle = 0);

Vector3 get_furthest_vertex(Shape* shape);
Vector3 get_furthest_point_in_direction(Shape* shape, Vector3 dir);
float get_shape_radius(Shape* shape);

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

int get_furthest_local_point_index_in_direction(Shape* shape, Vector3 dir)
{
    float max = -FLT_MAX;
    u32 index = 0;

    for(int i = 0; i < shape->vertices_count; ++i)
    {
        float dot_product = dot(shape->local_vertices[i], dir);
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
            //NOTE: Vertices order start from top left, counter-clockwise
            shape.vertices_count = 4;
            shape.local_vertices = (Vector3*)malloc(sizeof(Vector3) * shape.vertices_count);
            shape.global_vertices = (Vector3*)malloc(sizeof(Vector3) * shape.vertices_count);
            shape.local_vertices[0] = V3(-0.5f, 0.5f, 0);
            shape.local_vertices[1] = V3(-0.5f, -0.5f, 0);
            shape.local_vertices[2] = V3(0.5f, -0.5f, 0);
            shape.local_vertices[3] = V3(0.5f, 0.5f, 0);
        } break;
        case ShapeType::TRIANGLE:
        {
            /* NOTE: Vertices order
                2
                *   *
                        *
                *           *
                                *
                *                   1
                                *
                *           *
                        *    
                *   *
                3 
            */
            shape.vertices_count = 3;
            shape.local_vertices = (Vector3*)malloc(sizeof(Vector3) * shape.vertices_count);
            shape.global_vertices = (Vector3*)malloc(sizeof(Vector3) * shape.vertices_count);
            shape.local_vertices[0] = V3(0.5f, 0, 0);
            shape.local_vertices[1] = V3(-0.5f, 0.5f, 0);
            shape.local_vertices[2] = V3(-0.5f, -0.5f, 0);
        } break;
        case ShapeType::RIGHT_TRIANGLE:
        {
            /* NOTE: Vertices order
                2
                * *
                *  *
                *   *
                *    *
                3 * * 1 
            */
            shape.vertices_count = 3;
            shape.local_vertices = (Vector3*)malloc(sizeof(Vector3) * shape.vertices_count);
            shape.global_vertices = (Vector3*)malloc(sizeof(Vector3) * shape.vertices_count);
            shape.local_vertices[0] = V3(0.5f, -0.5f, 0);
            shape.local_vertices[1] = V3(-0.5f, 0.5f, 0);
            shape.local_vertices[2] = V3(-0.5f, -0.5f, 0);
        } break;
        case ShapeType::CUSTOM:
        {
            shape.local_vertices = vertices;
            shape.global_vertices = vertices;
        } break;
    }

    return shape;
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

#endif 