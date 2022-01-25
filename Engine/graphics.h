#ifndef GRAPHICS_H
#define GRAPHICS_H

struct DisplayInfo
{
	int refresh_rate;
	float game_update_hz;
	float ms_per_frame;
	float target_sec_per_frame;
	float fps;
};

struct Graphics
{
    SDL_Window* window;
    DisplayInfo display_info;
    Vector2 win_dim;
};

struct TextureData
{
    GLuint id;
    int width;
    int height;
};

struct ShapeDataGL
{
    u32 vao;
    u32 vbo;
    u32 ebo;

    Vector3* vertices;
    u32* indices;

    u32 n_verticies;
    u32 n_vertices_total;
    u32 n_indices;
    GLenum usage;
};

#endif 