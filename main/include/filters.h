#ifndef FIR_H
#define FIR_H
#include <stdlib.h>
#include <math.h>


class Fir{
    private:
        float *coeff;
        int order;
        int *buffer;
        int buffer_size;
        int buffer_index;  
    public:
        Fir(float *coeff, int order);
        float filter(float input);
    
};

Fir::Fir(float *coeff, int order){
    this->coeff = coeff;
    this->order = order;
    this->buffer_size = order;
    this->buffer = (int*)malloc(sizeof(int)*buffer_size);
    this->buffer_index = 0;
    for (int i = 0; i < buffer_size; i++)
    {
        buffer[i] = 0;
    }
}

float Fir::filter(float input){
    buffer[buffer_index] = input;
    float output = 0;
    for (int i = 0; i < order; i++)
    {
        output += coeff[i]*buffer[(buffer_index+i)%buffer_size];
    }
    buffer_index = (buffer_index+1)%buffer_size;
    return output;
}

class Iir{
    private:
        float alpha;
        float prev_output;


    public:
        Iir(float alpha);
        float filter(float input);
        int16_t filter(int16_t input);
    
};
Iir::Iir(float alpha)
{
    // currently only first order
    this->alpha = alpha;
    this->prev_output = 0;
}

int16_t Iir::filter(int16_t input){
    prev_output = alpha*input + (1-alpha)*prev_output;
    return int16_t(prev_output);
}

float Iir::filter(float input){
    prev_output = alpha*input + (1-alpha)*prev_output;
    return prev_output;
}

#endif