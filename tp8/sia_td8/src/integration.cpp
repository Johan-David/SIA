#include "integration.h"
#include <iostream>
using namespace std;

void explicitEulerStep (ODESystem *system, double dt) {
    // TODO: implement this
    VectorXd state = VectorXd(system->getDimensions());
    system->getState(state);
    VectorXd deriv = VectorXd(system->getDimensions());
    system->getDerivative(deriv);
    state = state + dt*deriv;
    system->setState(state);
}

void midPointStep (ODESystem *system, double dt) {
    // TODO: implement this
    VectorXd currentS = VectorXd(system->getDimensions());
    system->getState(currentS);

    VectorXd deriv = VectorXd(system->getDimensions());
    system->getDerivative(deriv);
    VectorXd state = VectorXd(system->getDimensions());
    system->getState(state);

    state = state + (dt*deriv)/2.0;
    system->setState(state);

    system->getDerivative(deriv);

    currentS = currentS + dt*deriv;
    system->setState(currentS);
}
