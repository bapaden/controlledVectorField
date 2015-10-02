#ifndef COST_H
#define COST_H

#include <vector>
#include <cassert>
#include <cmath>
#include "math_utils.h"


double cost(traj candidate)
{
    int n=candidate.path[0].size();
    int m=candidate.ctrl[0].size();
    int count = candidate.time.size();
    double cost=0;
    double dt=0;
    double x1,x2;
    
    printf("points on path: %d\n",count);
    printf("control dim: %d\n",m);
    printf("state dim: %d\n",n);
    
    for(int i=0;i<count-1;i++)
    {
        dt=candidate.time[i+1]-candidate.time[i];
        x1=candidate.path[i][0];
        x2=candidate.path[i][1];
        cost+=dt+dt*(x1*x1+x2*x2);
    }
    
    return cost;
}


#endif