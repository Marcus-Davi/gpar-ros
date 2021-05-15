#ifndef KALMAN_H
#define KALMAN_H
#include "Eigen/Dense"

#include <iostream>
#include <memory>

using namespace Eigen;

//classe para implementação do filtro de kalman estendido
namespace Kalman
{
	using MatrixKalman = Matrix<double, Dynamic, Dynamic, RowMajor>;
	using VectorKalman = Eigen::VectorXd;
	using FunctionPtr = void (*)(const VectorKalman &Xk, const VectorKalman &Uk, void *Res);


	// Extented Kalman Filter
	class EKF
	{
	public:
		using Ptr = std::shared_ptr<EKF>;
		using ConstPtr = std::shared_ptr<const EKF>;

		EKF(unsigned int n_states_, unsigned int n_inputs_, unsigned int n_outputs_);
		~EKF();

		//Ponteiros para functions
		inline void SetStateFunction(FunctionPtr F)
		{
			StateFun = F;
		}
		inline void SetStateJacobian(FunctionPtr F)
		{
			Jacobian_F = F;
		}
		inline void SetMeasurementJacobian(FunctionPtr F)
		{
			Jacobian_H = F;
		}
		inline void SetMeasurementFunction(FunctionPtr F)
		{
			MeasureFun = F;
		}

		inline void SetRn(const double *Rn_)
		{
			for (unsigned int i = 0; i < n_outputs * n_outputs; ++i)
				Rn(i) = Rn_[i];

			std::cout << "Measurement Covariance: " << std::endl
					  << Rn << std::endl;
		}

		inline void SetRn(const MatrixKalman &Rn_)
		{
			if (Rn_.cols() != n_outputs || Rn_.rows() != n_outputs)
			{
				std::cerr << "Wrong Rn dimension.. Setting Identity." << std::endl;
				Rn.setIdentity();
				return;
			}

			Rn = Rn_;
		}
		inline void SetQn(const double *Qn_)
		{

			for (unsigned int i = 0; i < n_states * n_states; ++i)
				Qn(i) = Qn_[i];

			std::cout << "Process Covariance: " << std::endl
					  << Qn << std::endl;
		}

		inline void SetQn(const MatrixKalman &Qn_)
		{
			if (Qn_.cols() != n_states || Qn_.rows() != n_states)
			{
				std::cerr << "Wrong Qn dimension.. Setting Identity." << std::endl;
				Qn.setIdentity();
				return;
			}

			Qn = Qn_;
		}

		// inline void setQn(const Eigen::)

		inline void SetX0(const double *x0)
		{
			for (unsigned int i = 0; i < n_states; ++i)
				Xest[i] = x0[i];
		}

		void Predict(const VectorKalman& input); //TODO fazer template ?
		void Update(const VectorKalman& output);
		void GetEstimatedStates(VectorKalman &states);

	private:
		const unsigned int n_states;
		const unsigned int n_outputs;
		const unsigned int n_inputs;

		FunctionPtr StateFun;
		FunctionPtr MeasureFun;
		FunctionPtr Jacobian_F;
		FunctionPtr Jacobian_H;

		MatrixKalman Jf; //Jacobian F
		MatrixKalman Jh; // Jacobian H

		MatrixKalman Qn, Rn; //Covariancias
		MatrixKalman Kk, Pk; //Ganho de Kalman e Covariancia erro
		MatrixKalman S;		 //Inversa

		VectorXd Xest, Y, Yest, U, E; //x estimado, saida Y, Y estimado, entrada U, err

		MatrixXd I; // Identidade
	};


	// Linear Kalman Filter
	class LKF
	{
	public:
		using Ptr = std::shared_ptr<LKF>;
		using ConstPtr = std::shared_ptr<const LKF>;

		LKF(unsigned int n_states_, unsigned int n_inputs_, unsigned int n_outputs_);
		~LKF();

		//Ponteiros para functions
		inline void setA(const MatrixKalman &A_)
		{
			// TODO Checks
			A = A_;
		}

		inline void setB(const MatrixKalman &B_)
		{
			// TODO Checks
			B = B_;
		}

		inline void setC(const MatrixKalman &C_)
		{
			// TODO Checks
			C = C_;
		}

		inline void SetRn(const double *Rn_)
		{
			for (unsigned int i = 0; i < n_outputs * n_outputs; ++i)
				Rn(i) = Rn_[i];

			std::cout << "Measurement Covariance: " << std::endl
					  << Rn << std::endl;
		}

		inline void SetRn(const MatrixKalman &Rn_)
		{
			if (Rn_.cols() != n_outputs || Rn_.rows() != n_outputs)
			{
				std::cerr << "Wrong Rn dimension.. Setting Identity." << std::endl;
				Rn.setIdentity();
				return;
			}

			Rn = Rn_;
		}
		inline void SetQn(const double *Qn_)
		{

			for (unsigned int i = 0; i < n_states * n_states; ++i)
				Qn(i) = Qn_[i];

			std::cout << "Process Covariance: " << std::endl
					  << Qn << std::endl;
		}

		inline void SetQn(const MatrixKalman &Qn_)
		{
			if (Qn_.cols() != n_states || Qn_.rows() != n_states)
			{
				std::cerr << "Wrong Qn dimension.. Setting Identity." << std::endl;
				Qn.setIdentity();
				return;
			}

			Qn = Qn_;
		}

		// inline void setQn(const Eigen::)

		inline void SetX0(const VectorKalman &x0)
		{
			Xest = x0;
		}

		void Predict(const VectorKalman& input); //TODO fazer template ?
		void Update(const VectorKalman& output);
		void GetEstimatedStates(VectorKalman &states);

	private:
		const unsigned int n_states;
		const unsigned int n_outputs;
		const unsigned int n_inputs;

		MatrixKalman A,B,C;

		MatrixKalman Qn, Rn; //Covariancias
		MatrixKalman Kk, Pk; //Ganho de Kalman e Covariancia erro
		MatrixKalman S;		 //Inversa

		VectorKalman Xest, Y, Yest, U, E; //x estimado, saida Y, Y estimado, entrada U, err

		MatrixXd I; // Identidade
	};

}

#endif
