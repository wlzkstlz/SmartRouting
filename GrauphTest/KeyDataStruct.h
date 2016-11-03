#pragma once
#include <opencv2/opencv.hpp>
using namespace cv;

enum KeyNodeType
{
	KNT_AC,
	KNT_CORNER,
	KNT_MAINTUBE
};
struct TKeyNode
{
	int node_id;
	Point location;
	int room_id;//belong to which room
	KeyNodeType type;
};