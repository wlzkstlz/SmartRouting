#include"commonAlg.h"
#include"math.h"


CCommonAlg::CCommonAlg()
{
}

CCommonAlg::~CCommonAlg()
{
}

double CCommonAlg::calcVectAngel(double x, double y)
{
	double angel;
	if (x < 0)
	{
		angel = atan(y / x) + M_PI;
		return angel;
	}
	else if (x > 0)
	{
		angel = atan(y / x);
		if (angel < 0)
		{
			angel += 2 * M_PI;
		}
		return angel;
	}
	else if (y > 0)
	{
		return 0.5*M_PI;
	}
	else
		return 1.5*M_PI;
}

void CCommonAlg::rotateVect(double& x, double& y, double deta_angel)
{
	double xx = x, yy = y;
	x = xx*cos(deta_angel) + yy*(-sin(deta_angel));
	y = xx*(sin(deta_angel)) + yy*cos(deta_angel);
}

void CCommonAlg::rotateVect(float& x, float& y, float deta_angel)
{
	float xx = x, yy = y;
	x = xx*cos(deta_angel) + yy*(-sin(deta_angel));
	y = xx*(sin(deta_angel)) + yy*cos(deta_angel);
}

double CCommonAlg::calcAngelDiffAbs(double angel1, double angel2)
{
	double angel_diff = fabs(angel1-angel2);
	if (angel_diff>M_PI)
	{
		angel_diff = 2.0*M_PI - angel_diff;
	}
	return angel_diff;
}

double CCommonAlg::calcAngelDiff(double angel1, double angel2)
{
	double angel_diff = angel1 - angel2;
	while (angel_diff>M_PI)
	{
		angel_diff -= 2 * M_PI;
	}
	while (angel_diff<-M_PI)
	{
		angel_diff += 2 * M_PI;
	}

	return angel_diff;
}