#pragma once
#include<vector>
#include <iostream>
//#include <iomanip>
using namespace std;

#define  INFINITY  65535

template<class NameType, class DistType>
class CGraph
{
private:
	vector<NameType> Vertices;
	DistType **Edges;
	int nVer, nEdges;

public:
	CGraph()
		: Edges(NULL)
		, nEdges(0)
		, nVer(0)
	{}
	~CGraph()
	{}

public:
	int GetVerCount() const
	{
		return nVer;
	}

	// �Ͻ�˹�����㷨ʵ��
	void ShortestPath_Dijkstra(int v0, int* final, int*p, int *D)
	{
		int v, w, k, min;
		// ��ʼ������
		for (v = 0; v < nVer; ++v)
		{
			final[v] = 0;    // ȫ�������ʼ��Ϊδ֪�Զ�·��״̬
			D[v] = Edges[v0][v]; //����V0�������ߵĶ������Ȩֵ
			p[v] = 0;    // ��ʼ��·������pΪ0
		}
		D[v0] = 0;    // V0��V0·��Ϊ0
		final[v0] = 1;    // final[W]=1��ʾV0��V0����Ҫ��·��
						  // ��ʼ��ѭ����ÿ�����V0��ĳ��V��������·��
		for (v = 1; v < nVer; ++v)
		{
			min = INFINITY;    // ��ǰ��֪��V0�����������
			for (w = 0; w < nVer; ++w) // Ѱ����V0����Ķ���
			{
				if (!final[w] && D[w] < min)
				{
					min = D[w]; // w������V0�������
					k = w;
				}
			}

			final[k] = 1; // ��Ŀǰ�ҵ�������Ķ�����Ϊ1
			for (w = 0; w < nVer; ++w) // ������ǰ���·������
			{
				// �������V�����·������������·���ĳ��ȶ̵Ļ�
				if (!final[w] && (min + Edges[k][w] < D[w]))
				{
					// ˵���ҵ�����̵�·�����޸�D[w] �� p[w]
					D[w] = min + Edges[k][w]; // �޸ĵ�ǰ·������
					p[w] = k;
				}
			}
		}
	}
};