#ifndef MODELFUNCTIONS_H
#define MODELFUNCTIONS_H
#include "gpar_kalman/Quaternion.h"
#include "Kalman.h"

#define SystemTs 0.02f
static float mag_field;

namespace AttitudeEstimation {


static void StateJacobian(const VectorXd& Xk,const VectorXd& Uk,void* Res){
	MatrixKalman* M = static_cast<MatrixKalman*>(Res);

	(*M)(0,0) = 1.0f;
	(*M)(0,1) = -Uk[0]*SystemTs/2;
	(*M)(0,2) = -Uk[1]*SystemTs/2;
	(*M)(0,3) = -Uk[2]*SystemTs/2;

	(*M)(1,0) = Uk[0]*SystemTs/2;
	(*M)(1,1) = 1.0f;
	(*M)(1,2) = Uk[2]*SystemTs/2;
	(*M)(1,3) = -Uk[1]*SystemTs/2;

	(*M)(2,0) = Uk[1]*SystemTs/2;
	(*M)(2,1) = -Uk[2]*SystemTs/2;
	(*M)(2,2) = 1.0f;
	(*M)(2,3) = Uk[0]*SystemTs/2;

	(*M)(3,0) = Uk[2]*SystemTs/2;
	(*M)(3,1) = Uk[1]*SystemTs/2;
	(*M)(3,2) = -Uk[0]*SystemTs/2;
	(*M)(3,3) = 1.0f;

    //std::cout << "Jf: " << *M << std::endl;

}


static void MeasurementJacobian(const VectorXd& Xk,const VectorXd& Uk,void* Res){
	MatrixKalman* M = static_cast<MatrixKalman*>(Res);

	float m = mag_field*0.9440f; // B * cos(m_incl)
	float n = mag_field*0.3298f; // B * sin(m_incl)

	static const float g = 9.8;

	(*M)(0,0) = -Xk[2]*g;
	(*M)(0,1) = Xk[3]*g;
	(*M)(0,2) = -Xk[0]*g;
	(*M)(0,3) = Xk[1]*g;


	(*M)(1,0) = Xk[1]*g;
	(*M)(1,1) = Xk[0]*g;
	(*M)(1,2) = Xk[3]*g;
	(*M)(1,3) = Xk[2]*g;


	(*M)(2,0) = Xk[0]*g;
	(*M)(2,1) = -Xk[1]*g;
	(*M)(2,2) = -Xk[2]*g;
	(*M)(2,3) = Xk[3]*g;


	(*M)(3,0) = Xk[0]*m + (-Xk[2]*n);
	(*M)(3,1) = Xk[1]*m + (Xk[3]*n);
	(*M)(3,2) = -Xk[2]*m + (-Xk[0]*n);
	(*M)(3,3) = -Xk[3]*m + (Xk[1]*n);


	(*M)(4,0) = -Xk[3]*m + (Xk[1]*n);
	(*M)(4,1) = Xk[2]*m + (Xk[0]*n);
	(*M)(4,2) = Xk[1]*m + (Xk[3]*n);
	(*M)(4,3) = -Xk[0]*m + (Xk[2]*n);


	(*M)(5,0) = Xk[2]*m + (Xk[0]*n);
	(*M)(5,1) = Xk[3]*m + (-Xk[1]*n);
	(*M)(5,2) = Xk[0]*m + (-Xk[2]*n);
	(*M)(5,3) = Xk[1]*m + (Xk[3]*n);

    (*M) = (*M)*2;

    //std::cout << "JH: " << *M << std::endl;

	//arm_mat_scale_f32(M, 2.0f, M);

}

void StateFunction(const VectorXd& Xk,const VectorXd& Uk,void* Res){
	VectorXd* M = static_cast<VectorXd*>(Res);

	MD::Quaternion qk(Xk[0],Xk[1],Xk[2],Xk[3]);
	MD::Quaternion qk_w(0,Uk[0],Uk[1],Uk[2]);
	MD::Quaternion qk1;
	qk1 = qk*qk_w;
	qk1 = qk1* (SystemTs/2);
	qk1 = qk1 + qk;


(*M)[0] = qk1.w;
(*M)[1] = qk1.v.x;
(*M)[2] = qk1.v.y;
(*M)[3] = qk1.v.z;

//std::cout << "State Est: " <<*M << std::endl;

}

void MeasurementFunction(const VectorXd& Xk,const VectorXd& Uk,void* Res){
	VectorXd* M = static_cast<VectorXd*>(Res);

	double m = mag_field*0.9440f; // B * cos(m_incl)
	double n = mag_field*0.3298f; // B * sin (m_incl)


	MD::Quaternion qk(Xk[0],Xk[1],Xk[2],Xk[3]);
	MD::Quaternion qg(0,0,0,9.8); //gravity
	MD::Quaternion qm(0,m,0,n); //gravity
	MD::Quaternion qa,qb;

	qa.v = qk.RotateFrame(qg.v);
	qb.v = qk.RotateFrame(qm.v);


	(*M)[0] = qa.v.x;
	(*M)[1] = qa.v.y;
	(*M)[2] = qa.v.z;
	(*M)[3] = qb.v.x;
	(*M)[4] = qb.v.y;
	(*M)[5] = qb.v.z;

    //std::cout << "Measure Est : "<<*M << std::endl;

}

double GetMagField(const geometry_msgs::Vector3& mag){
    return(sqrt(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z));
}

}


#endif
