#ifndef VECTOR_FIELD_H
#define VECTOR_FIELD_H

#include <vector>
#include <cassert>
#include <cmath>

typedef  std::vector<double> vctr;

vctr flow(vctr x, vctr u)
{
    int n=2;
    int m=1;
    assert(x.size()==n && u.size()==m);        
    vctr dx(n);
    
    //double integrator
    dx.at(0) = x.at(1);
    dx.at(1) = -sin(x.at(0))+u.at(0);
    
    return dx;
}


#endif