#include "gpar_kalman/Kalman.h"


namespace Kalman {

EKF::EKF(unsigned int n_states_,unsigned int n_inputs_, unsigned int n_outputs_)
: n_states(n_states_), n_inputs(n_inputs_), n_outputs(n_outputs_)
{


	Jf.resize(n_states,n_states);
	Jh.resize(n_outputs,n_states);
	Qn.resize(n_states,n_states);
	Rn.resize(n_outputs,n_outputs);
	Kk.resize(n_states,n_outputs);
//	Pk.resize(n_states,n_states);

	Xest.resize(n_states);
	Yest.resize(n_outputs);
	U.resize(n_inputs);
	Y.resize(n_outputs);

	I = MatrixXd::Identity(n_states,n_states);
	Pk = MatrixXd::Identity(n_states,n_states);

}

EKF::~EKF(){

}

//ver forma melhor para entradas
void EKF::Predict(const double* Input){

	for(unsigned int k=0;k<n_inputs;++k){
	 U(k) = Input[k];
   }

   //std::cout << "U: " <<U << std::endl;

	StateFun(Xest,U,&Xest);
	Jacobian_F(Xest,U,&Jf);

	
	Pk = Jf*Pk*Jf.transpose() + Qn;
}

void EKF::Update(const double* Measurement){

	for(unsigned int k=0;k<n_outputs;++k){
		Y(k) = Measurement[k];
	}

	//std::cout << "M: " << Y << std::endl;

MeasureFun(Xest,U,&Yest);
Jacobian_H(Xest,U,&Jh);

E = Y-Yest;

S = Jh*Pk*Jh.transpose() + Rn;
Kk = Pk*Jh.transpose()*S.inverse();
Xest = Xest + Kk*E;

Pk = (I-Kk*Jh)*Pk;
}

void EKF::GetEstimatedStates(double* states){
for(unsigned int i=0;i<n_states;++i)
states[i] = Xest[i];
}

















}
