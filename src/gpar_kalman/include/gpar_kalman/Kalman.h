#ifndef KALMAN_H
#define KALMAN_H
#include "Eigen/Dense"

#include <iostream>

using namespace Eigen;


typedef void (*KalmanFunctionPt)(const VectorXd& Xk,const VectorXd& Uk,void* Res); // M é entrada,saída ou uma matriz

typedef Matrix<double,Dynamic,Dynamic,RowMajor> MatrixKalman;
//classe para implementação do filtro de kalman estendido
namespace Kalman {

class EKF {
	public:
		EKF(unsigned int n_states,unsigned int n_inputs, unsigned int n_outputs);
		~EKF();

		//Ponteiros para functions
		inline void SetStateFunction(KalmanFunctionPt F) {
		StateFun = F;
		}
		inline void  SetStateJacobian(KalmanFunctionPt F){
		Jacobian_F = F;
		}
		inline void  SetMeasurementJacobian(KalmanFunctionPt F){
		Jacobian_H = F;
		}
		inline void SetMeasurementFunction(KalmanFunctionPt F){
		MeasureFun = F;
		}

		inline void SetRn(const double* Rn_){
			for(unsigned int i=0;i<n_outputs*n_outputs;++i)
			Rn(i) = Rn_[i];
			
			std::cout << "Measurement Covariance: " << std::endl << Rn << std::endl;
		}
		inline void SetQn(const double* Qn_){

			for(unsigned int i=0;i<n_states*n_states;++i)
			Qn(i) = Qn_[i];

			std::cout << "Process Covariance: " << std::endl << Qn << std::endl;
		}

		inline void SetX0(const double* x0){
		for(unsigned int i=0;i<n_states;++i)
		 Xest[i] = x0[i];
		}			

		void Predict(const double* input); //TODO fazer template ?
		void Update(const double* output);
		void GetEstimatedStates(double* states);



	private:

	const unsigned int n_states;
	const unsigned int n_outputs;
  	const unsigned int n_inputs;

	KalmanFunctionPt StateFun;
	KalmanFunctionPt MeasureFun;
	KalmanFunctionPt Jacobian_F;
	KalmanFunctionPt Jacobian_H;

	MatrixKalman Jf; //Jacobian F
	MatrixKalman Jh; // Jacobian H

	MatrixKalman Qn,Rn; //Covariancias
	MatrixKalman Kk,Pk; //Ganho de Kalman e Covariancia erro
	MatrixKalman S; //Inversa

	VectorXd Xest,Y,Yest,U,E; //x estimado, saida Y, Y estimado, entrada U, err

	MatrixXd I; // Identidade


};


}


#endif
