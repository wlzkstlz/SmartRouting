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
	Point location;
	int room_id;//belong to which room
	KeyNodeType type;
	bool is_done;
};