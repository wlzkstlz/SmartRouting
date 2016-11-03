#include"Graph.h"

template<class NameType, class DistType>
void CGraph::ShortestPath_Dijkstra(int v0, int* final, int*p, int *D)
{
	int v, w, k, min;
	// 初始化数据
	for (v = 0; v < nVer; ++v)
	{
		final[v] = 0;    // 全部顶点初始化为未知对短路径状态
		D[v] = Edges[v0][v]; //将与V0点有连线的顶点加上权值
		p[v] = 0;    // 初始化路径数组p为0
	}
	D[v0] = 0;    // V0至V0路径为0
	final[v0] = 1;    // final[W]=1表示V0至V0不需要求路径
					  // 开始主循环，每次求得V0到某个V顶点的最短路径
	for (v = 1; v < nVer; ++v)
	{
		min = INFINITY;    // 当前所知离V0顶点最近距离
		for (w = 0; w < nVer; ++w) // 寻找离V0最近的顶点
		{
			if (!final[w] && D[w] < min)
			{
				min = D[w]; // w顶点离V0顶点更近
				k = w;
			}
		}

		final[k] = 1; // 将目前找到的最近的顶点置为1
		for (w = 0; w < nVer; ++w) // 修正当前最短路径距离
		{
			// 如果经过V顶点的路径比现在这条路径的长度短的话
			if (!final[w] && (min + Edges[k][w] < D[w]))
			{
				// 说明找到了最短的路径，修改D[w] 和 p[w]
				D[w] = min + Edges[k][w]; // 修改当前路径长度
				p[w] = k;
			}
		}
	}
}