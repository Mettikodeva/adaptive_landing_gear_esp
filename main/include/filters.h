#ifndef FIR_H
#define FIR_H
#include <stdlib.h>
#include <math.h>
#include <vector>


#define IIR_WINDOW 10

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
    protected:
        float alpha;
        float prev_output;
        int WINDOW = 13;
        std::vector<int16_t> buffer;
        std::vector<int16_t> median_tmp;
    public:
        Iir(float alpha);
        float filter(float input);
        int16_t filter(int16_t input);
        int16_t getMedian();
        

};

Iir::Iir(float alpha)
{
    // currently only first order
    this->alpha = alpha;
    this->prev_output = 0;
}

int16_t IRAM_ATTR Iir::getMedian(){
    median_tmp.clear();
    auto it = std::next(buffer.begin(), buffer.size());
    std::move(buffer.begin(), it, std::back_inserter(median_tmp));

    buffer.erase(buffer.begin(), it);
    std::sort(median_tmp.begin(), median_tmp.end());

    return median_tmp.at(median_tmp.size()/2);
}

int16_t IRAM_ATTR Iir::filter(int16_t input){
    buffer.push_back(input);
    while(buffer.size() > WINDOW){
        buffer.erase(buffer.begin());
    }

    size_t size = buffer.size();
    for(size_t i = 1; i < size; ++i){
        buffer.push_back(alpha * buffer[i] + (1 - alpha) * buffer[i - 1]);
    }
    return getMedian();
}

float Iir::filter(float input){
    prev_output = alpha*input + (1-alpha)*prev_output;
    return prev_output;
}

#endif