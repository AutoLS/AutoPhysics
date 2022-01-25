#ifndef INPUT_H
#define INPUT_H

enum MouseButtons
{
    MOUSE_BUTTON_LEFT = 1,
	MOUSE_BUTTON_MIDDLE = 2,
	MOUSE_BUTTON_RIGHT = 3,
	MOUSE_BUTTON_X1 = 4,
	MOUSE_BUTTON_X2 = 5
};

struct ButtonState
{
    bool is_down, was_down, ended_down;
    bool pressed;
    bool up;

    int half_transitions;
};

struct ControllerInput
{
    std::map<SDL_Scancode, ButtonState> keys;
    std::map<MouseButtons, ButtonState> mouse_buttons;
};

struct Input
{
    ControllerInput controller;
    bool scrolling;
    int wheel_delta;
};

#endif