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

	RigidBody player = create_body(create_shape({100, 100}), {100, 300}, {}, 5);
	RigidBody box = create_body(create_shape({50, 50}), {200, 400}, {}, 3);
	RigidBody wall = create_body(create_shape({1000, 100}), {640, 200}, {}, 0);

	DistanceConstraint simple_constraint = set_distance_constraint(&player, &box, player.position + V3(50, 0), box.position + V3(-25, 0));

    while(handle_events())
    {
        gl_clear(V4(0, 0, 0, 1));

        float new_time = get_current_time_in_seconds();
		float frame_time = new_time - physics_current_time;
		physics_current_time = new_time;

		physics_time_accumlator += frame_time;
		Manifold m = {};
        while(physics_time_accumlator >= physics_dt)
        {	
			if(test_SAT(&player.shape, &wall.shape, &m))
			{
				printf("collided! n_contacts: %d\n", (int)m.cp.size());
				print_vec3(m.normal, "Normal");
				for(int i = 0; i < m.cp.size(); ++i)
				{
					print_vec3(m.cp[i], "CP");
				}
			}

			integrate_for_velocity(&player, physics_dt);
			integrate_for_velocity(&box, physics_dt);

			solve_distance_constraint(&simple_constraint, physics_dt);

			integrate_for_position(&player, physics_dt);
			integrate_for_position(&box, physics_dt);

            physics_time_accumlator -= physics_dt;
        }
		
		player.force = {};
		Vector3 force = {};
		float weight = 10000;
	 	if(key_down(SDL_SCANCODE_LEFT))
		{
			force += {-1, 0};
		}
		if(key_down(SDL_SCANCODE_RIGHT))
		{
			force += {1, 0};
		}
		if(key_down(SDL_SCANCODE_UP))
		{
			force += {0, 1};
		}
		if(key_down(SDL_SCANCODE_DOWN))
		{
			force += {0, -1};
		}

		player.force += normalize(force) * weight;

        lock_fps(get_refresh_rate());

        Mat4 projection = mat4_ortho(0, 1280, 0, 720, -1, 1);
		gl_set_mat4(basic_renderer, "Projection", projection);
		gl_set_mat4(basic_renderer, "View", mat4_identity());

        gl_draw(basic_renderer, rect_shape_data, 0, true, player.position, player.shape.dim, player.orientation);
        gl_draw(basic_renderer, rect_shape_data, 0, true, box.position, box.shape.dim, box.orientation, V4(1, 0, 0, 1));
		gl_draw(basic_renderer, rect_shape_data, 0, true, wall.position, wall.shape.dim, wall.orientation, V4(0, 0, 1, 1)); 
		for(int i = 0; i < m.cp.size(); ++i)
		{
			gl_draw(basic_renderer, rect_shape_data, 0, true, m.cp[i], {10,10}, V4(1, 1, 0, 1)); 
		}

        SDL_GL_SwapWindow(app_core.graphics.window);
        update_clock(&app_core.clock);
    }
}