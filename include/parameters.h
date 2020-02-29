#ifndef PARAMETER_H
#define PARAMETER_H

namespace ccn
{

	class ParameterList{ //算法用到的参数列表
	public:
		int Overlap_Registration_KNN;
		float Overlap_Registration_OverlapRatio;

		float Cloud_Registration_Downsampledis_ALS;
		float Cloud_Registration_Downsampledis_TLS;
		float Cloud_Registration_Downsampledis_MLS;
		float Cloud_Registration_Downsampledis_BPLS;

		int Cloud_Registration_MaxIterNumber;
		float Cloud_Registration_OverlapSearchRadius;
		bool Cloud_Registration_UseTrimmedRejector;
		bool Cloud_Registration_UseReciprocalCorres;
		float Cloud_Registration_PerturbateValue;
		int Cloud_Registration_CovarianceK;
		float Cloud_Registration_MinOverlapForReg;

		int Graph_Optimization_MaxIterNumber;

	protected:

	private:

	};
}

#endif //PARAMETER_H