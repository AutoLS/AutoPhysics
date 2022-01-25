#ifndef UTIL_H
#define UTIL_H

std::vector<std::string> split(std::string str, std::string delimiter)
{
    std::vector<std::string> result;
    std::string copy = str;
    size_t pos = 0;
    while((pos = copy.find(delimiter)) != std::string::npos)
    {
        std::string token = copy.substr(0, pos);
        result.push_back(token);
        copy.erase(0, pos + delimiter.length());
    }

    result.push_back(copy);

    return result;
}

//TODO: This is the most stupidiest code I came up with atm
Vector4 rec_str_to_vec4(std::vector<std::string> rect_data, float width, float height)
{
    Vector4 result = {};
    if(rect_data.size() != 4) return result;

    result.x = (float)atof(rect_data[0].c_str()) / (float)width;
    result.y = (float)atof(rect_data[1].c_str()) / (float)height;
    result.z = (float)atof(rect_data[2].c_str()) / (float)width;
    result.w = (float)atof(rect_data[3].c_str()) / (float)height;

    return result;
}

#endif 