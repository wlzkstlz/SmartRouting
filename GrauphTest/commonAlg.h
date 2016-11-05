#pragma once
class CCommonAlg
{
public:
	CCommonAlg();
	~CCommonAlg();

	//输入向量终点坐标，输出[0,2pi)
	static double calcVectAngel(double x, double y);

	//将向量（x,y）旋转deta_angel
	static void rotateVect(double& x, double& y,double deta_angel);

	//将向量（x,y）旋转deta_angel
	static void rotateVect(float& x, float& y, float deta_angel);

	//输入两个取值范围[0,2pi)的角度值，输出两个矢量的夹角的绝对值
	static double calcAngelDiffAbs(double angel1,double angel2);

	//输入两个取值范围[0,2pi)的角度值，输出两个矢量的夹角
	static double calcAngelDiff(double angel1, double angel2);
private:

};


