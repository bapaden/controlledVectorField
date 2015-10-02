#ifndef SYSTEM_H
#define SYSTEM_H


#include <string.h>
#include <limits>
#include <ctime>
#include <vector>
#include "math_utils.h"
#include "vector_field.h"



//controlled dynamical system
class RK4
{
public:
    //Solution to controlled ode
    traj sol;
    //pointer to cost function
    vctr (*f)(vctr, vctr);
    //maximum time step
    double dt_max;
    //dimension of the state space
    unsigned int n;
    //dimension of input space
    unsigned int m;
    
    //single integration step with RK4
    void step(vctr& x_plus, vctr& x, vctr u, double dt)
    {      
        
        vctr k1=f(x,u);
        vctr k2=f(add(x,mult(0.5*dt,k1)),u);
        vctr k3=f(add(x,mult(0.5*dt,k2)),u);
        vctr k4=f(add(x,mult(dt,k3)),u);
        vctr klast=add(add(add(k1,mult(2.0,k2)),mult(2.0,k3)),k4);
        
        x_plus = add(x,mult(dt*0.16666666666,klast));
    }
    
    //integrate forward for tspan from x0 with constant u
    traj sim(double tspan, vctr x0, vctr u)
    {
        //compute minimum number of steps to satisfy dt<dt_max
        double num_steps=ceil(tspan/dt_max);
        double dt=tspan/num_steps;
        //resize solution
        sol.path.resize(num_steps+1);
        sol.time.resize(num_steps+1);
        sol.ctrl.resize(num_steps+1);
        
        //set initial condition
        sol.path.at(0)=x0;
        sol.ctrl.at(0)=u;
        //integrate
        for(int i=0;i<num_steps;i++)
        {
            step(sol.path.at(i+1),sol.path.at(i),u,dt);
            sol.ctrl.at(i+1)=u;
            sol.time.at(i+1)=sol.time.at(i)+dt;
        }
        return sol;
    }
    
    //Write sol out on the screen
    void print_traj(traj sol)
    {
        printf("\n***** Solution to simulation *****\n");
        printf("time:  state:");
        printf("\n");
        for(int i=0;i<sol.time.size();i++)
        {
            printf("%4.2f:  (",sol.time.at(i));
            for(int j=0;j<(sol.path[0]).size();j++)
            {
                printf("%4.2f, ",sol.path[i][j]);
            }
            printf(")\n");
        }
        printf("**********************************\n");
    }
    
    //constructor
    RK4(vctr (*_f)(vctr, vctr),double _dt_max, int _n, int _m)
    {
        
        f = _f;
        dt_max=_dt_max;
        n = _n;
        m = _m;
    }
};


#endif






























































