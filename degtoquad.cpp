#include <iostream>
#include "dcm.h"
#include <math.h>
 
 #define   DEGTORAD M_PI/double(180.0);
quad eulertoquad(eluer in)
{
    quad output;
    double cosPhi_2 = double(cos(in.phi_ / double(2.0)));//X轴
    double cosTheta_2 = double(cos(in.theta_ / double(2.0)));//Y轴
    double cosPsi_2 = double(cos(in.psi_ / double(2.0)));//Z轴
    double sinPhi_2 = double(sin(in.phi_ / double(2.0)));
    double sinTheta_2 = double(sin(in.theta_ / double(2.0)));
    double sinPsi_2 = double(sin(in.psi_ / double(2.0)));

    output.q0=cosPhi_2 * cosTheta_2 * cosPsi_2 +
               sinPhi_2 * sinTheta_2 * sinPsi_2;    
    output.q1=sinPhi_2 * cosTheta_2 * cosPsi_2 -
               cosPhi_2 * sinTheta_2 * sinPsi_2;
    output.q2=cosPhi_2 * sinTheta_2 * cosPsi_2 +
               sinPhi_2 * cosTheta_2 * sinPsi_2;
    output.q3=cosPhi_2 * cosTheta_2 * sinPsi_2 -
               sinPhi_2 * sinTheta_2 * cosPsi_2;

    return output;
}
dcm quadtodcm(quad input)
{
    double q0=input.q0;
    double q1=input.q1;
    double q2=input.q2;
    double q3=input.q3;
    dcm output;
    output.R00=double(1.0)-2*q2*q2-2*q3*q3;
    output.R01=2*q1*q2-2*q0*q3;
    output.R02=2*q1*q3+2*q0*q2;
    output.R10=2*q1*q2+2*q0*q3;
    output.R11=double(1.0)-2*q1*q1-2*q3*q3;
    output.R12=2*q2*q3-2*q0*q1;
    output.R20=2*q1*q3-2*q0*q2;
    output.R21=2*q2*q3+2*q0*q1;
    output.R22=double(1.0)-2*q1*q1-2*q2*q2;
    return output;
}

quad quadx(quad p,quad q)
{
    quad output;
    double p0=p.q0;
    double p1=p.q1;
    double p2=p.q2;
    double p3=p.q3;
    
    double q0=q.q0;
    double q1=q.q1;
    double q2=q.q2;
    double q3=q.q3;
    output.q0=p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3;
    output.q1=p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2;
    output.q2=p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1;
    output.q3=p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0;
     
     return output;
}

eluer quadtoeluer(quad input)
{
    eluer output;
    const double radtodeg=double(180.0)/M_PI;
    output.phi_=radtodeg*atan2(2*(input.q0*input.q1+input.q2*input.q3),1-2*(pow(input.q1,2)+pow(input.q2,2)));
    output.theta_=radtodeg*asin(2*(input.q0*input.q2-input.q1*input.q3));
    output.psi_=radtodeg*atan2(2*(input.q0*input.q3+input.q1*input.q2),1-2*(pow(input.q2,2)+pow(input.q3,2)));
    return output;
}

int main(int argc, char const *argv[])
{

    using namespace std;
    cout.setf(cout.showpoint);
    cout.setf(ios::fixed);
    cout.precision(10);

    cout<<"input roll pitch yaw :";

    eluer input_;
    cin>>input_.phi_;
    cin>> input_.theta_;
    cin>> input_.psi_;
    input_.phi_*=DEGTORAD;
    input_.theta_*=DEGTORAD;
    input_.psi_*=DEGTORAD;

    quad output=eulertoquad(input_);
    cout<<"Convert euler to quad,the converted Quaternion: "<<endl;
    cout<<"q0: "<<output.q0<<endl<<"q1: "<<output.q1<<endl<<"q2: "
    <<output.q2<<endl<<"q3: "<<output.q3<<endl<<endl;

    eluer out_el=quadtoeluer(output);
    cout<<"Convert quad to euler,the converted euler: "<<endl;
    cout<<"Roll : "<<out_el.phi_<<endl<<"Pitch ："<<out_el.theta_<<endl<<"Yaw : "<<out_el.psi_<<endl<<endl;
   
    dcm  output_dcm=quadtodcm(output);
    cout<<"Convert quad to dcm,the converted dcm: "<<endl;
    cout<<output_dcm.R00<<'\t'<<output_dcm.R01<<'\t'<<output_dcm.R02<<endl;
    cout<<output_dcm.R10<<'\t'<<output_dcm.R11<<'\t'<<output_dcm.R12<<endl;
    cout<<output_dcm.R20<<'\t'<<output_dcm.R21<<'\t'<<output_dcm.R22<<endl<<endl;
    
    quad quad1=quadx(output,output); 
    cout<<"Multiply the converted quad of input euler,the multiplied quad: "<<endl;
    cout<<"q0: "<<quad1.q0<<endl<<"q1: "<<quad1.q1<<endl<<"q2: "
    <<quad1.q2<<endl<<"q3: "<<quad1.q3<<endl<<endl;

    eluer quad1_elr=quadtoeluer(quad1);
    cout<<"Convert the multiplied quad to euler,the converted euler: "<<endl;
    cout<<"Roll : "<<quad1_elr.phi_<<endl<<"Pitch ："<<quad1_elr.theta_<<endl<<"Yaw : "<<quad1_elr.psi_<<endl<<endl;
    return 0;
}
