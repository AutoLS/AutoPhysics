#include "graphics.h"

int get_window_refresh_rate(SDL_Window* window)
{
    SDL_DisplayMode mode;
    int display_index = SDL_GetWindowDisplayIndex(window);
    int default_refresh_rate = 60;
    if (SDL_GetDesktopDisplayMode(display_index, &mode) != 0)
    {
        return default_refresh_rate;
    }
    if (mode.refresh_rate == 0)
    {
        return default_refresh_rate;
    }

    return mode.refresh_rate;
}

DisplayInfo get_display_info(SDL_Window* window)
{
    DisplayInfo display = {};
    display.refresh_rate = get_window_refresh_rate(window);
    display.game_update_hz = (float)display.refresh_rate;
    display.ms_per_frame = 1000.0f / display.game_update_hz;
    display.target_sec_per_frame = 1.0f / display.game_update_hz;

    return display;
}

float get_refresh_rate()
{
    return (float)app_core.graphics.display_info.refresh_rate;
}

Vector2 get_win_dim()
{
    return app_core.graphics.win_dim;
}

void init_graphics(Graphics* graphics, const char* title, int width, int height, SDL_WindowFlags flags)
{
    const char* glsl_version = "#version 330";
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
	SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

	graphics->window = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, flags);
	SDL_GLContext gl_context = SDL_GL_CreateContext(graphics->window);
    if (!gl_context)
    {
        printf("Error: GL_Context create failed!\n");
    }
    else
    {
	    SDL_GL_MakeCurrent(graphics->window, gl_context);
        GLenum glewError = glewInit();
        if(glewError != GLEW_OK)
        {
            printf("Error loading glew!\n");
        }
    }

    graphics->display_info = get_display_info(graphics->window);
    graphics->win_dim = {(float)width, (float)height};

	// IMGUI_CHECKVERSION();
	// ImGui::CreateContext();
	// ImGui::StyleColorsDark();
	// ImGui_ImplSDL2_InitForOpenGL(graphics->window, gl_context);
	// ImGui_ImplOpenGL3_Init(glsl_version);
}

void gl_gen_shape(ShapeDataGL* shape, Vector3* vertices, int count, int total_count, u32* indices, int n_indices, GLenum usage = GL_STATIC_DRAW)
{
    glGenVertexArrays(1, &shape->vao);
    glGenBuffers(1, &shape->vbo);

    if(indices)
    {
        glGenBuffers(1, &shape->ebo);
        shape->indices = indices;
        shape->n_indices = n_indices;
    }

    shape->vertices = vertices;
    shape->n_verticies = count;
    shape->n_vertices_total = total_count;
    shape->usage = usage;
}

void gl_upload_shape_data(ShapeDataGL* shape)
{
    glBindVertexArray(shape->vao);
    glBindBuffer(GL_ARRAY_BUFFER, shape->vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vector3) * shape->n_vertices_total, shape->vertices, shape->usage);

    if(shape->indices)
    {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, shape->ebo);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(u32) * shape->n_indices, 
					 shape->indices, shape->usage);
    }

    glBindVertexArray(0);
}

void gl_set_attrib_pointer(ShapeDataGL* shape, u32 location, int size, GLsizei stride, int offset)
{
    glBindVertexArray(shape->vao);
	glVertexAttribPointer(location, size, GL_FLOAT, GL_FALSE, 
						  stride * sizeof(float), (void*)(sizeof(float)*offset));
	glEnableVertexAttribArray(location);
	glBindVertexArray(0);
}

void gl_set_int(u32 shader, char* location, int a)
{
    glUseProgram(shader);
    glUniform1i(glGetUniformLocation(shader, location), a);
}

void gl_set_vec4(u32 shader, char* location, Vector4 a)
{
    glUseProgram(shader);
    glUniform4f(glGetUniformLocation(shader, location), a.x, a.y, a.z, a.w);
}

void gl_set_mat4(u32 Shader, char* Location, Mat4 A, GLenum IsTranspose = GL_TRUE)
{
	glUseProgram(Shader);
	glUniformMatrix4fv(glGetUniformLocation(Shader, Location), 1, IsTranspose, (float*)A.E);
}

char* gl_load_shader_source(char* path)
{
	FILE* shader_source_file = fopen(path, "rb");
	char* buffer = 0;
	long length; 
	
	if(!shader_source_file)
	{
		printf("File does not exist.\n");
	}
	else
	{
		fseek(shader_source_file, 0L, SEEK_END);
		length = ftell(shader_source_file);
		fseek(shader_source_file, 0L, SEEK_SET);
		
		buffer = (char*)calloc(length+1, sizeof(char));
		if(buffer)
		{
			fread(buffer, sizeof(char), length, shader_source_file);
		}
		else
		{
			printf("Buffer is empty. Error.\n");
		}
	}
	fclose(shader_source_file);
	
	return buffer;
}

u32 gl_create_shader(char* vertex_shader_source, char* fragment_shader_source)
{
	u32 vertext_shader;
	vertext_shader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertext_shader, 1, &vertex_shader_source, 0);
	glCompileShader(vertext_shader);
	
	int shader_compile_success;
	char info_log[512];
	
	glGetShaderiv(vertext_shader, GL_COMPILE_STATUS, &shader_compile_success);
	if(!shader_compile_success)
	{
		glGetShaderInfoLog(vertext_shader, 512, 0, info_log);
		printf("VERTEX SHADER COMPILATION FAILED: %s", info_log);
	}
	
	u32 fragment_shader;
	fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragment_shader, 1, &fragment_shader_source, 0);
	glCompileShader(fragment_shader);
	
	glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &shader_compile_success);
	if(!shader_compile_success)
	{
		glGetShaderInfoLog(fragment_shader, 512, 0, info_log);
		printf("FRAGMENT SHADER COMPILATION FAILED: %s", info_log);
	}
	
	u32 program = glCreateProgram();
	glAttachShader(program, vertext_shader);
	glAttachShader(program, fragment_shader);
	glLinkProgram(program);
	
	int LinkingSuccess;
	glGetProgramiv(program, GL_LINK_STATUS, &LinkingSuccess);
	if(!LinkingSuccess)
	{
		glGetProgramInfoLog(program, 512, 0, info_log);
		printf("PROGRAM LINKING FAILED: %s", info_log);
	}
	
	glDeleteShader(vertext_shader);
	glDeleteShader(fragment_shader);  
	
	return program;
}

bool load_texture(const char* path, TextureData* texture, bool flip = false)
{
    stbi_set_flip_vertically_on_load(flip);
    
    // Load from file
    unsigned char* image_data = stbi_load(path, &texture->width, &texture->height, NULL, 4);
    if (image_data == NULL)
        return false;

    // Create a OpenGL texture identifier
    GLuint image_texture;
    glGenTextures(1, &image_texture);
    glBindTexture(GL_TEXTURE_2D, image_texture);

    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); // This is required on WebGL for non power-of-two textures
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); // Same

    // Upload pixels into texture
#if defined(GL_UNPACK_ROW_LENGTH) && !defined(__EMSCRIPTEN__)
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
#endif
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texture->width, texture->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
    stbi_image_free(image_data);

    texture->id = image_texture;

    return true;
}

void gl_draw(u32 shader, ShapeDataGL& shape, u32 texture, bool blend, Vector3 pos, Vector3 dim, Quaternion orientation, Vector4 color = {1, 1, 1, 1})
{
    glUseProgram(shader);
    gl_set_vec4(shader, "Color", color);

    Mat4 transform = mat4_identity();
    transform = mat4_scale(transform, dim);
    transform = transform * to_mat4(orientation);
    transform = mat4_translate(transform, pos);

    gl_set_mat4(shader, "Transform", transform);

    if(blend)
    {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    if(glIsTexture(texture) == GL_TRUE)
    {
        gl_set_int(shader, "Texture", 0);
        gl_set_int(shader, "IsTexture", 1);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);
    }
    else
    {
        gl_set_int(shader, "IsTexture", 0);
    }

    glBindVertexArray(shape.vao);
    if(shape.indices)
    {
        glDrawElements(GL_TRIANGLES, shape.n_indices, GL_UNSIGNED_INT, 0);
    }
    else
    {
        glDrawArrays(GL_TRIANGLES, 0, shape.n_verticies);
    }

    glDisable(GL_BLEND);
    glBindVertexArray(0);
    glUseProgram(0);
}

void gl_draw(u32 shader, ShapeDataGL& shape, u32 texture, bool blend, Vector3 pos, Vector3 dim, Vector4 color = {1, 1, 1, 1}, float theta = 0, Vector3 axis = {0, 0, 1})
{
    glUseProgram(shader);
    gl_set_vec4(shader, "Color", color);

    Mat4 transform = mat4_identity();
    transform = mat4_scale(transform, dim);
    if(theta) transform = mat4_rotate(transform, axis, theta);
    transform = mat4_translate(transform, pos);

    gl_set_mat4(shader, "Transform", transform);

    if(blend)
    {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    if(glIsTexture(texture) == GL_TRUE)
    {
        gl_set_int(shader, "Texture", 0);
        gl_set_int(shader, "IsTexture", 1);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);
    }
    else
    {
        gl_set_int(shader, "IsTexture", 0);
    }

    glBindVertexArray(shape.vao);
    if(shape.indices)
    {
        glDrawElements(GL_TRIANGLES, shape.n_indices, GL_UNSIGNED_INT, 0);
    }
    else
    {
        glDrawArrays(GL_TRIANGLES, 0, shape.n_verticies);
    }

    glDisable(GL_BLEND);
    glBindVertexArray(0);
    glUseProgram(0);
}

void gl_clear(Vector4 clear_color)
{
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}