#ifndef CORE_H
#define CORE_H

#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef uint32_t u32;
typedef uint64_t u64;
typedef int s32;
typedef int64_t s64;

#define ARRAY_SIZE(arr) (sizeof(arr)/sizeof(arr[0]))

//TODO: Add imgui supprt later
// #include "imgui/imgui.h"
// #include "imgui/imgui_impl_sdl.h"
// #include "imgui/imgui_impl_opengl3.h"

#include <SDL.h>
#include <SDL_image.h>
#include <SDL_TTF.h>
#include <SDL_Mixer.h>
#include <SDL_Net.h>
#include <GL/glew.h>
#include <SDL_opengl.h>
#include "stb_image.cpp"
#include "dirent.h"

#include "math.h"
#include "util.h"
#include "Physics/physics.cpp"

#include "time.h"
#include "graphics.h"
#include "input.h"
//#include "gui_util.h"

struct FileData 
{
	void* data;
	uint64_t size;
};

struct Core
{
	Clock clock;
    Graphics graphics;
	Input input;
};

Core app_core = {};

#include "graphics.cpp"
#include "input.cpp"

int init_core(const char* title, int width, int height, SDL_WindowFlags flags)
{
    if (SDL_Init(SDL_INIT_EVERYTHING))
	{
		printf("Error: %s\n", SDL_GetError());
        return -1;
	}

	init_clock(&app_core.clock);
	init_graphics(&app_core.graphics, title, width, height, flags);

    return 0;
}

bool handle_events()
{
	bool running = true;
	reset_controller_input(&app_core.input);
	SDL_Event event;
	while (SDL_PollEvent(&event)) 
	{
		//ImGui_ImplSDL2_ProcessEvent(&event);
		if (event.type == SDL_QUIT) running = false;	
		if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(app_core.graphics.window)) 
			running = false;
#if 1
		switch(event.type)
		{
			case SDL_KEYDOWN:
			case SDL_KEYUP:
			{
				bool is_down = event.key.state == SDL_PRESSED;
				bool was_down;

				if (event.key.state == SDL_RELEASED)
				{
					was_down = true;
				}
				else if (event.key.repeat != 0)
				{
					was_down = true;
				}

				if(event.key.repeat == 0)
				{
					auto key = event.key.keysym.scancode;

					process_key(&app_core.input, key, is_down, was_down);
				}
			} break;
			case SDL_MOUSEBUTTONDOWN:
			case SDL_MOUSEBUTTONUP:
			{
				bool is_down = event.button.state == SDL_PRESSED;
				bool was_down;

				if (event.button.state == SDL_RELEASED)
				{
					was_down = true;
				}

				MouseButtons button = (MouseButtons)event.button.button;
				process_button(&app_core.input, button, is_down, was_down);
			} break;
		}
#endif
	}

	return running;
}

void lock_fps(float target_fps)
{
	Clock* clock = &app_core.clock;
	if(target_fps > 0)
	{
		float frame_complete_time = get_seconds_elapsed(clock->last_count, SDL_GetPerformanceCounter(), clock->sec_per_count);

		float target_sec_per_frame = 1.0f / target_fps;

		if(frame_complete_time < target_sec_per_frame)
		{
			int sleep_time = (int)((target_sec_per_frame - frame_complete_time) * 1000) - 1;
			if(sleep_time > 0)
			{
				SDL_Delay(sleep_time);
				while(get_seconds_elapsed(clock->last_count, SDL_GetPerformanceCounter(), clock->sec_per_count) < target_sec_per_frame);
			}
		}
	}

	clock->end_count = SDL_GetPerformanceCounter();
}

float get_fps()
{
	Clock* clock = &app_core.clock; 
	if(clock->counter_elapsed == 0) return -1;
	float fps = (float)clock->performance_frequency / (float)clock->counter_elapsed;
	return fps;
}

//NOTE: Not sure if these are supposed to be in util.. 
bool read_directory(const char* path, std::vector<std::string>* list, bool omit_file_extension = false)
{
	DIR* directory = opendir(path);
	dirent* entry = 0;

	int count = 0;

	bool result = false;

	if(directory)
	{
		while((entry = readdir(directory)) != 0)
		{
			if(count > 1)
			{
				char* entry_name = entry->d_name;
				std::string file_name = std::string(entry_name);
				if(omit_file_extension)
				{
					file_name = file_name.substr(0, file_name.find_last_of("."));
					list->push_back(file_name);
				}
				else
				{
					list->push_back(file_name);
				}
			}
			++count;
		}
		closedir(directory);

		result = true;
	}
	else
	{
		printf("Unable to locate directory!\n");
	}

	return result;
}

void file_read_lines(const char* path, std::vector<std::string>*lines)
{
	std::ifstream file(path);
	std::string str;
	while(std::getline(file, str))
	{
		lines->push_back(str);
	}
	file.close();
}

#endif