#ifndef GUI_UTIL_H
#define GUI_UTIL_H

#include "graphics.h"

void gui_image(TextureData* texture, Vector2 dim, Vector2 uv_min, Vector2 uv_max)
{
    ImGui::Image((void*)(intptr_t)texture->id, ImVec2(dim.x, dim.y), ImVec2(uv_min.x, uv_min.y), ImVec2(uv_max.x, uv_max.y));
}

#endif 