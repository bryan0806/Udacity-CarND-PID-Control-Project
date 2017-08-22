#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
    Kp=Kp_;
    Ki=Ki_;
    Kd=Kd_;
}

void PID::UpdateError(double cte) {
    double error = cte;
    cout << "error:" << error << endl;
}

double PID::TotalError() {
}

