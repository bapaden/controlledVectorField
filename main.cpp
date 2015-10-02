#include <cstdio>
#include "numerical_integration.h"
#include "cost.h"

int main(int argc, char **argv) {
    
    vctr x0(2);
    vctr u(1);
    x0.at(0)=0.1;
    x0.at(1)=0.0;
    u.at(0)=0.0;
    
    double dt_max=0.1;
    double state_dim=2;
    double control_dim=1;
    double tspan=2.0*M_PI;
    RK4 sys(&flow,dt_max,state_dim,control_dim);
    traj sol = sys.sim(tspan,x0,u);
    
    sys.print_traj(sol);
    printf("cost: %f\n",cost(sol));
    
    //Plot result
    vctr path(sol.time.size());
    for(int i=0;i<sol.path.size();i++)
    {
        path.at(i)=sol.path[i][0];
    }
    
    plt::plot(sol.time,path);
    plt::xlim(0, (int) ceil(sol.time.back()));
    plt::show();
    //plt::save("./basic.png");
    
    
    return 0;
}
