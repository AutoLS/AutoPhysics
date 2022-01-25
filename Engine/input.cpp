#include "input.h"

void process_keypress(ButtonState* new_state, bool is_down, bool was_down)
{
	new_state->is_down = is_down;
	new_state->ended_down = is_down;
	new_state->up = was_down;
	new_state->was_down = was_down;
	new_state->half_transitions += 1;
}

void process_key(Input* input, SDL_Scancode key, bool is_down, bool was_down)
{
    auto button_state = input->controller.keys.find(key);
    if(button_state != input->controller.keys.end())
    {
        process_keypress(&button_state->second, is_down, was_down);
    }
    else
    {
        ButtonState new_state;
        process_keypress(&new_state, is_down, was_down);
        input->controller.keys.insert({key, new_state});
    }
}

void process_button(Input* input, MouseButtons button, bool is_down, bool was_down)
{
    auto button_state = input->controller.mouse_buttons.find(button);
    if(button_state != input->controller.mouse_buttons.end())
    {
        process_keypress(&button_state->second, is_down, was_down);
    }
    else
    {
        ButtonState new_state;
        process_keypress(&new_state, is_down, was_down);
        input->controller.mouse_buttons.insert({button, new_state});
    }
}

void reset_controller_input(Input* input)
{
    ControllerInput* controller = &input->controller;
    for(auto it = controller->keys.begin(); it != controller->keys.end(); ++it)
    {
        it->second.ended_down = false;
        it->second.was_down = false;
    }
    for(auto it = controller->mouse_buttons.begin(); it != controller->mouse_buttons.end(); ++it)
    {
        it->second.ended_down = false;
        it->second.was_down = false;
    }
}

Vector2 get_mouse_position()
{
    Vector2 win_dim = get_win_dim();
    int win_x, win_y;
    SDL_GetWindowPosition(app_core.graphics.window, &win_x, &win_y);

    int x, y;
    SDL_GetGlobalMouseState(&x, &y);

    Vector2 pos = {};
    pos.x = clamp((float)(x - win_x), 0, win_dim.x);
    pos.y = clamp((float)(y - win_y), 0, win_dim.y);

    return pos;
}

Vector2 get_mouse_position_flipped()
{
    Vector2 win_dim = get_win_dim();
    Vector2 pos = get_mouse_position();
    pos.y = win_dim.y - pos.y;

    return pos;
}

bool key_down(SDL_Scancode key)
{
    ControllerInput* controller = &app_core.input.controller;
    auto button_state = controller->keys.find(key);
    if(button_state != controller->keys.end())
    {
        return button_state->second.is_down;
    }
    else
    {
        return false;
    }
}

bool key_up(SDL_Scancode key)
{
    ControllerInput* controller = &app_core.input.controller;
    auto button_state = controller->keys.find(key);
    if(button_state != controller->keys.end())
    {
        return button_state->second.up;
    }
    else
    {
        return false;
    }
}

bool key_ended_down(SDL_Scancode key)
{
    ControllerInput* controller = &app_core.input.controller;
    auto button_state = controller->keys.find(key);
    if(button_state != controller->keys.end())
    {
        return button_state->second.ended_down;
    }
    else
    {
        return false;
    }
}

bool key_pressed(SDL_Scancode key)
{
    ControllerInput* controller = &app_core.input.controller;
    auto button_state = controller->keys.find(key);
    if(button_state != controller->keys.end())
    {
        return button_state->second.pressed;
    }
    else
    {
        return false;
    }
}

bool mouse_ended_down(MouseButtons button)
{
    ControllerInput* controller = &app_core.input.controller;
    auto button_state = controller->mouse_buttons.find(button);
    if(button_state != controller->mouse_buttons.end())
    {
        return button_state->second.ended_down;
    }
    else
    {
        return false;
    }
}