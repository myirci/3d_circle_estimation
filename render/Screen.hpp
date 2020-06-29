#ifndef SCREEN_HPP
#define SCREEN_HPP

struct screen {
    int w, h;
    int vpw, vph;
    int x0, y0;
    double depth_near_val;
    double depth_far_val;
    screen(int width, int height) : w(width), h(height),
        vpw(width), vph(height), x0(0), y0(0),
        depth_near_val(0.0), depth_far_val(1.0) { }
};

#endif // SCREEN_HPP
