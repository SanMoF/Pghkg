#ifndef _FILTER_H
#define _FILTER_H

#include <cstring> // memcpy, memset
#include <algorithm>

#define MAX_NUM_SIZE 8
#define MAX_DEN_SIZE 8

class Filter
{
public:
    Filter();
    ~Filter();

    // b_arr: numerador (feed-forward) length num_coeff_b
    // a_arr: denominador (feedback) length num_coeff_a (a[0] normalmente 1)
    void setup(const float b_arr[], const float a_arr[], unsigned char num_coeff_b, unsigned char num_coeff_a,
               float x_init = 0.0f, float y_init = 0.0f);

    float apply(float input_data);

private:
    float _b[MAX_NUM_SIZE];
    float _a[MAX_DEN_SIZE];
    float _x[MAX_NUM_SIZE]; // past inputs
    float _y[MAX_DEN_SIZE]; // past outputs

    // _num_coeff[0] = nb (b count), _num_coeff[1] = na (a count)
    unsigned char _num_coeff[2];
};

#endif // _FILTER_H
