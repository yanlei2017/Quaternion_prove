#ifndef DCM_H_
#define DCM_H_
struct eluer
    {
        double phi_;
        double theta_;
        double psi_;
    };

struct quad
{
    double q0;
    double q1;
    double q2;
    double q3;
};

 struct dcm{
        double R00;
        double R01;
        double R02;
        double R10;
        double R11;
        double R12;
        double R20;
        double R21;
        double R22;
}; 

#endif