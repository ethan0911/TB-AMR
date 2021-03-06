#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <GLFW/glfw3.h>
#include "imgui.h"

struct Colormap {
    std::string name;
    // An RGBA8 1D image
    std::vector<uint8_t> colormap;
    bool use_opacity;

    Colormap(const std::string &name, const std::vector<uint8_t> &img,
             const bool use_opacity = false);
    Colormap(const std::string &infile, const bool use_opacity = false);

    void save(const std::string &fname);
};

class TransferFunctionWidget {
    struct vec2f {
        float x, y;

        vec2f(float c = 0.f);
        vec2f(float x, float y);
        vec2f(const ImVec2 &v);

        float length() const;

        vec2f operator+(const vec2f &b) const;
        vec2f operator-(const vec2f &b) const;
        vec2f operator/(const vec2f &b) const;
        vec2f operator*(const vec2f &b) const;
        operator ImVec2() const;
    };

    std::vector<Colormap> colormaps;
    size_t selected_colormap = 0;
    std::vector<uint8_t> current_colormap;

    std::vector<vec2f> alpha_control_pts = {vec2f(0.f), vec2f(1.f)};
    size_t selected_point = -1;

    bool colormap_changed = true;
    GLuint colormap_img = -1;

    std::array<float, 2> val_range;
    std::array<float, 2> current_range;

public:
    TransferFunctionWidget(float val_min, float val_max);

    // Add a colormap preset. The image should be a 1D RGBA8 image
    void add_colormap(const Colormap &map);

    // Add the transfer function UI into the currently active window
    void draw_ui();

    // Returns true if the colormap was updated since the last
    // call to draw_ui
    bool changed() const;

    // Get back the RGBA8 color data for the transfer function
    std::vector<uint8_t> get_colormap();

    // Get back the RGBA32F color data for the transfer function
    std::vector<float> get_colormapf();

    std::array<float, 2> get_current_range();

    // Get back the RGBA32F color data for the transfer function
    // as separate color and opacity vectors
    void get_colormapf(std::vector<float> &color, std::vector<float> &opacity);

private:
    void update_gpu_image();

    void update_colormap();

    void load_embedded_preset(const uint8_t *buf, size_t size, const std::string &name);
};

