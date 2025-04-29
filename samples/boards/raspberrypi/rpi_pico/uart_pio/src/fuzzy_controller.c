// Thao Do 2025

#include <stdint.h>
#include <math.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include "common.h"

#define E_NORMALIZE (1/250.0f)
#define DELTA_E_NORMALIZE (0.0001f)
#define NUMBER_OF_RULES 25

static float fuzzy_rule_e[5][3] = {
    {-1, -0.6, -0.2},
    {-0.6, -0.2, 0},
    {-0.2, 0, 0.2},
    {0, 0.2, 0.6},
    {0.2, 0.6, 1}
};

static float fuzzy_rule_delta_e[5][3] = {
    {-1, -0.9, -0.5},
    {-0.9, -0.5, 0},
    {-0.5, 0, 0.5},
    {0, 0.5, 0.9},
    {0.5, 0.9, 1}
};

static float fuzzy_delta_u[] = {-1.0f, -0.95f, -0.6f, 0.0f, 0.6f, 0.95f, 1.0f};

static int32_t fuzzy_rule[NUMBER_OF_RULES][3] = {
    {0, 0, 0},
     {0, 1, 0},
     {0, 2, 1},
     {0, 3, 2},
     {0, 4, 1},
     {1, 0, 0},
     {1, 1, 1},
     {1, 2, 2},
     {1, 3, 3},
     {1, 4, 4},
     {2, 0, 1},
     {2, 1, 2},
     {2, 2, 3},
     {2, 3, 4},
     {2, 4, 5},
     {3, 0, 2},
     {3, 1, 3},
     {3, 2, 4},
     {3, 3, 5},
     {3, 4, 6},
    { 4, 0, 3},
    { 4, 1, 4},
    { 4, 2, 5},
    { 4, 3, 6},
    { 4, 4, 6},
};

static float triangular_mf(float x, float a, float b, float c)
{
    if (x < a || x > c) {
        return 0.0f;
    } else if (x <= b) {
        return (x - a) / (b - a);
    } else {
        return (c - x) / (c - b);
    }
}
static float trapezoidal_mf(float x, float a, float b, float c, float d)
{
    if (x < a || x > d) {
        return 0.0f;
    } else if (x <= b) {
        return (x - a) / (b - a);
    } else if (x <= c) {
        return 1.0f;
    } else {
        return (d - x) / (d - c);
    }
}

int32_t fuzzy_control_init(fuzzy_controller *fuzzy, float e_coff, float de_coff, int32_t sample_time)
{
    if (fuzzy == NULL) {
        return -1; // Error: fuzzy controller pointer is NULL
    }

    // Set the sample time
    fuzzy->sample_time = sample_time;

    // Set the coefficients
    fuzzy->e_cofficient = e_coff;
    fuzzy->de_cofficient = de_coff;

    // Initialize previous error and output
    fuzzy->pre_e = 0;
    fuzzy->pre_output = 0;

    return 0;
}


float fuzzy_control(fuzzy_controller *ctrl, float set_point, float measured_value)
{
    float e = set_point - measured_value;
    float du = 0;
    float delta_e = (e - ctrl->pre_e) / ctrl->sample_time;
    float numerator = 0;
    float denominator = 0;
    float e_membership[5] = { 0 };
    float delta_e_membership[5] = { 0 };

    ctrl->pre_e = e;

    e = e * ctrl->e_cofficient;
    delta_e = delta_e * ctrl->de_cofficient;

    for (uint32_t i = 0; i < ARRAY_SIZE(e_membership); i++) {
        if (i == 0) {
            e_membership[i] = trapezoidal_mf(e, -2, fuzzy_rule_e[i][0], fuzzy_rule_e[i][1], fuzzy_rule_e[i][2]);
            delta_e_membership[i] = trapezoidal_mf(delta_e, -2, fuzzy_rule_delta_e[i][0], fuzzy_rule_delta_e[i][1], fuzzy_rule_delta_e[i][2]);
        } else if (i == (ARRAY_SIZE(e_membership) - 1)) {
            e_membership[i] = trapezoidal_mf(e, fuzzy_rule_e[i][0], fuzzy_rule_e[i][1], fuzzy_rule_e[i][2], 2);
            delta_e_membership[i] = trapezoidal_mf(delta_e, fuzzy_rule_delta_e[i][0], fuzzy_rule_delta_e[i][1], fuzzy_rule_delta_e[i][2], 2);
        } else {
            e_membership[i] = triangular_mf(e, fuzzy_rule_e[i][0], fuzzy_rule_e[i][1], fuzzy_rule_e[i][2]);
            delta_e_membership[i] = triangular_mf(delta_e, fuzzy_rule_delta_e[i][0], fuzzy_rule_delta_e[i][1], fuzzy_rule_delta_e[i][2]);
        }
    }

    for (uint32_t i = 0; i < NUMBER_OF_RULES; i++) {
        int e_index = fuzzy_rule[i][0];
        int delta_e_index = fuzzy_rule[i][1];
        int output_index = fuzzy_rule[i][2];
        denominator += e_membership[e_index] * delta_e_membership[delta_e_index];
        numerator += e_membership[e_index] * delta_e_membership[delta_e_index] * fuzzy_delta_u[output_index];
    }

    if (denominator == 0) {
        du = 0;
    } else {
        du = numerator / denominator;
    }

    // update control signal
    float u = ctrl->pre_output + du;
    if (u >= 100.0f) {
        u = 100.0f;
    } else if (u <= 0.0f) {
        u = 0.0f;
    }
    ctrl->pre_output = u;
    return u;
}
