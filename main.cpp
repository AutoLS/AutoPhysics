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
	
	// Vector3* custom_rect_vertices_1 = (Vector3*)malloc(sizeof(Vector3) * 4);
	// custom_rect_vertices_1[0] = V3(9, 7, 0);
	// custom_rect_vertices_1[1] = V3(5, 11, 0);
	// custom_rect_vertices_1[2] = V3(2, 8, 0);
	// custom_rect_vertices_1[3] = V3(6, 4, 0);

	// Vector3* custom_rect_vertices_2 = (Vector3*)malloc(sizeof(Vector3) * 4);
	// custom_rect_vertices_2[0] = V3(12, 5, 0);
	// custom_rect_vertices_2[1] = V3(4, 5, 0);
	// custom_rect_vertices_2[2] = V3(4, 2, 0);
	// custom_rect_vertices_2[3] = V3(12, 2, 0);

	// Shape test_shape_1 = create_shape({}, ShapeType::CUSTOM, custom_rect_vertices_1);
	// test_shape_1.vertices_count = 4;
	// Shape test_shape_2 = create_shape({}, ShapeType::CUSTOM, custom_rect_vertices_2);
	// test_shape_2.vertices_count = 4;

	// RigidBody test_box_1 = {};
	// test_box_1.shape = test_shape_1;
	// RigidBody test_box_2 = {};
	// test_box_2.shape = test_shape_2;

	RigidBody player_body = create_body(create_shape({100, 100}), {100, 350}, {}, 3);
	player_body.color = V4(1, 1, 1, 1);
	player_body.type = DebugType::PLAYER;
	player_body.friction = 0.5f;
	//player_body.freeze_orientation = true;
	RigidBody box = create_body(create_shape({50, 50}), {200, 450}, {}, 1);
	box.color = V4(1, 0, 0, 1);
	RigidBody wall = create_body(create_shape({1000, 100}), {640, 200}, {}, 0);
	wall.color = V4(0.5f, 0.5f, 0.5f, 1);
	wall.freeze_orientation = true;
	wall.friction = 0.5f;
	RigidBody wall2 = create_body(create_shape({500, 100}), {640, 450}, {}, 0);
	wall2.color = V4(0.5f, 0.5f, 0.5f, 1);
	wall2.freeze_orientation = true;
	wall2.friction = 0.5f;

	std::vector<RigidBody> bodies;
	bodies.push_back(player_body);
	bodies.push_back(box);
	bodies.push_back(wall);
	bodies.push_back(wall2);

	Constraint test_constraint = create_distance_constraint(&player_body, &box, player_body.position + V3(50, 0), box.position + V3(-25, 0));

	bool angular_motion = false;

	std::vector<Manifold> manifolds;

    while(handle_events())
    {
        gl_clear(V4(0, 0, 0, 1));
		
		manifolds.clear();

		if(mouse_ended_down(MOUSE_BUTTON_LEFT))
		{
			RigidBody b = create_body(create_shape({50, 50}), V3(get_mouse_position_flipped()), {}, 1);
			b.color = V4(randf(0, 1.0f), randf(0, 1.0f), randf(0, 1.0f), 1); 
			b.restitution = 0;
			bodies.insert(bodies.begin(), b);
		}

		for(RigidBody& body : bodies)
		{
			if(body.type == DebugType::PLAYER)
			{
				body.force = {};
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
				if(key_ended_down(SDL_SCANCODE_SPACE))
				{
					angular_motion = !angular_motion;
				}

				body.force += normalize(force) * weight;

				if(angular_motion)
					body.angular_velocity += V3(0, 0, 1) * physics_dt;
				else
					body.angular_velocity = {};
			}
		}
		

        float new_time = get_current_time_in_seconds();
		float frame_time = new_time - physics_current_time;
		physics_current_time = new_time;

		physics_time_accumlator += frame_time;

		for(int i = 0; i < bodies.size(); ++i)
		{
			for(int j = i+1; j < bodies.size(); ++j)
			{
				Manifold m = {};
				if(test_SAT(&bodies[i], &bodies[j], &m))
				{
					manifolds.push_back(m);
				}
			}
		}

        while(physics_time_accumlator >= physics_dt)
        {	
			
			for(Manifold& m : manifolds)
			{
				for(Contact& c : m.contacts)
				{
					update_contact(&m, &c);
				}
			}

			for(RigidBody& body : bodies)
			{
				integrate_for_velocity(&body, physics_dt);
			}

			for(Manifold& m : manifolds)
			{
				for(Contact& c : m.contacts)
				{
					solve_contact_constraint(&m, &c);
				}
			}

			//apply_impulse(&test_constraint, physics_dt);

			for(RigidBody& body : bodies)
			{
				integrate_for_position(&body, physics_dt);
			}

            physics_time_accumlator -= physics_dt;
        }

        lock_fps(get_refresh_rate());

        Mat4 projection = mat4_ortho(0, 1280, 0, 720, -1, 1);
		gl_set_mat4(basic_renderer, "Projection", projection);
		gl_set_mat4(basic_renderer, "View", mat4_identity());

		for(RigidBody& body : bodies)
		{
			gl_draw(basic_renderer, rect_shape_data, 0, true, body.position, body.shape.dim, body.orientation, body.color);
		}

		for(Manifold& m : manifolds)
		{
			for(int i = 0; i < m.cp.size(); ++i)
			{
				gl_draw(basic_renderer, rect_shape_data, 0, true, m.cp[i], {8,8}, V4(1, 1, 0, 1)); 
			}
		}

        SDL_GL_SwapWindow(app_core.graphics.window);
        update_clock(&app_core.clock);
    }
}