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
	int main_tube_discrete_id;
	KeyNodeType type;
	bool is_done;

	TKeyNode() {
		is_done = false; 
		main_tube_discrete_id= -1;
	}
};