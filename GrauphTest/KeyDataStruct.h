#pragma once
#include <opencv2/opencv.hpp>
using namespace cv;

enum KeyNodeType
{
	KNT_AC,
	KNT_CORNER,
	KNT_MAIN_TUBE_END
};
struct TKeyNode
{
	int node_id;
	Point location;//is always on room contours
	int room_id;//belong to which room
	int contour_id;//belong to which contour point
	int main_tube_discrete_id;//for KNT_MAIN_TUBE_END
	KeyNodeType type;
	int close_to_maintube_id;//for KNT_AC
	int closest_maintube_end_id;//for KNT_AC

	int *path_list;//���ڴ洢���·����ǰ���±�����
	int *dist_list;//���ڴ洢���������·���ľ����

	TKeyNode()
	{
		contour_id=room_id=node_id=main_tube_discrete_id= close_to_maintube_id= closest_maintube_end_id = -1;
		location.x = location.y = -1;
	}
};