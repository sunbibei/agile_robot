#include "repository/estimator/SimpleTDEstimator.h"

namespace agile_robot {

SimpleTDEstimator::SimpleTDEstimator()
{
}


SimpleTDEstimator::~SimpleTDEstimator()
{
}

void SimpleTDEstimator::input(double _new_data)
{
	int i, j;

	for (i = 9; i >0; i--)
	{
		j = i - 1;
		data[i] = data[j];
	}

	data[0] = _new_data;
}

bool SimpleTDEstimator::eval() {

	double sum = 0;
	double Z;
	double ww[5] = { 0 };
	// ���㻬��ƽ��ֵ, ���ػ���ƽ��ֵd[]//
	for (int i = 0; i < 10; i++)
	{
		sum = 0;
		for (int j = i; j < i + 5; j++)
		{
			sum += data[j];
		}

		d[i] = sum / 5;

		if (i + 5 == 10)
		{
			i = i + 5;
		}
	}

	//�ж��Ƿ����//
	for (int i = 0; i < 5; i++)
	{
		ww[i] = d[i];
	}

	
	for (int i = 0; i < 5; i++)
		for (int j=i+1; j < 5; j++)
		{
			if (ww[i] > ww[j])
			{
				Z = ww[j];
				ww[j] = ww[i];
				ww[i] = Z;
			}
		};

	if (ww[0] < 1500)
	{
		if (ww[4] > 1500)
		{
			if (d[4] - d[0] > 0)
			{
				sw = true;
			}
			else
			{
				sw = false;
			}
		}
		else
		{
			sw = false;
		}
	}
	else
	{
		sw = true;
	}

	return sw;

	LOG_WARNING << "You don't should call this method with super's empty implement";
	//return false;
}

}  /* end namespace middleware */


