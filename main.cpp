#include "Engine/core.h"

int wmain()
{
    init_core("Physics Engine", 1280, 720, (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE));

    Vector3 rect_shape_vertices[] =
	{
		V3(0.5f, 0.5f),   V3(1, 1),	//Top Right		
		V3(0.5f, -0.5f),  V3(1, 0),	//Bottom Right
		V3(-0.5f, -0.5f), V3(),		//Bottom Left
		V3(-0.5f, -0.5f), V3(),		//Bottom Left
		V3(-0.5f, 0.5f),  V3(0, 1),	//Top Left
		V3(0.5f, 0.5f),   V3(1, 1)	//Top Right	
	};

	ShapeDataGL rect_shape_data = {};
	gl_gen_shape(&rect_shape_data, rect_shape_vertices, 6, ARRAY_SIZE(rect_shape_vertices), 0, 0);
	gl_upload_shape_data(&rect_shape_data);
	gl_set_attrib_pointer(&rect_shape_data, 0, 3, 6, 0);
	gl_set_attrib_pointer(&rect_shape_data, 1, 3, 6, 3);

	u32 basic_renderer = gl_create_shader(gl_load_shader_source("shaders/basic_vert.glsl"),
										  gl_load_shader_source("shaders/basic_frag.glsl"));

    while(handle_events())
    {
        gl_clear(V4(0, 0, 0, 1));

        float new_time = get_current_time_in_seconds();
		float frame_time = new_time - physics_current_time;
		physics_current_time = new_time;

		physics_time_accumlator += frame_time;

        while(physics_time_accumlator >= physics_dt)
        {
            physics_time_accumlator -= physics_dt;
        }

        lock_fps(get_refresh_rate());

        Mat4 projection = mat4_ortho(0, 1280, 0, 720, -1, 1);
		gl_set_mat4(basic_renderer, "Projection", projection);
		gl_set_mat4(basic_renderer, "View", mat4_identity());

        gl_draw(basic_renderer, rect_shape_data, 0, true, V3(100, 100), V3(100, 100));
        SDL_GL_SwapWindow(app_core.graphics.window);
        update_clock(&app_core.clock);
    }
}