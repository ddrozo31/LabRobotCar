#ifndef LowPass2ndOrder_h
#define LowPass2ndOrder_h
#include <Arduino.h>

class LowPass2ndOrder
{
  private:
    float a[2];
    float b[3];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[3]; // Raw values
    float y[3]; // Filtered values

  public:
    // Contructor  
    LowPass2ndOrder();
    // Methods
    void init(float f0, float fs, bool adaptive);
    void setCoef();
    
    float filt(float xn);

};

#endif
