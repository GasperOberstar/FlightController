#pragma once

struct Pt1{
    float k=0;
    float output=0;

    bool init(float fc, float dt, float x);
    float apply(float x);
    [[nodiscard]] bool setCutoff(float fc, float dt);
    void reset(float x);
};