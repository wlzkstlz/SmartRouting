#pragma once
class CCommonAlg
{
public:
	CCommonAlg();
	~CCommonAlg();

	//���������յ����꣬���[0,2pi)
	static double calcVectAngel(double x, double y);

	//��������x,y����תdeta_angel
	static void rotateVect(double& x, double& y,double deta_angel);

	//��������x,y����תdeta_angel
	static void rotateVect(float& x, float& y, float deta_angel);

	//��������ȡֵ��Χ[0,2pi)�ĽǶ�ֵ���������ʸ���ļнǵľ���ֵ
	static double calcAngelDiffAbs(double angel1,double angel2);

	//��������ȡֵ��Χ[0,2pi)�ĽǶ�ֵ���������ʸ���ļн�
	static double calcAngelDiff(double angel1, double angel2);
private:

};


