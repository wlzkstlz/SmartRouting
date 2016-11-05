#pragma once
#include <opencv2/opencv.hpp>
#include<vector>

#include"KeyDataStruct.h"
using namespace cv;

#define ROOM_BORDER_PANT_WIDTH	2//墙描边画笔宽度
class CSmartRouting
{
public:
	CSmartRouting();
	~CSmartRouting();

	void ReadSourceImg(const char*file_name);

	void SetBorderUIPts(const vector<vector<Point>>&borderUIPts)
	{
		mBorderUIPts = borderUIPts;
	}

	void SetMainTubeUIPts(const vector<Point>& mainTubeUIPts)
	{
		mMainTubeUIPts = mainTubeUIPts;
	}

	bool SetACLocations(int room_id, Point location)
	{
		if (room_id >= 0 && room_id < mACLocationsFromUI.size())
		{
			mACLocationsFromUI[room_id] = location;
			return true;
		}
		else
			return false;
	}

	void RecognizeRooms();

	void RecognizeCorners();

	void RecognizeMainTubeEnds();

	void RecognizeACLocation();

	void ShowResult();
public:
	Mat mSourceMat;
	Mat mMat4Draw;
	vector<vector<Point>>mBorderUIPts;
	vector<Point>mMainTubeUIPts;
	vector<Point>mMainTubeDiscretePts;
	
	

	vector<vector<Point>>mRoomContours;//【关键数据】所有房间最大内墙轮廓
	vector<Mat>mRoomRegions;
	//Mat mOutOfAllRoomRegion;
	//Mat mAllInOneRegion;

	vector<Point>mACLocationsFromUI;//(-1,-1)stands for undifined!



	vector<TKeyNode>mKeyNodes;//【关键数据】


//public:
//	vector<bool>mIsSolved;//是否已完成设计

	//图数据结构
public:
	vector<Point>***mConectiveEdges;// [mNodeSize][mNodeSize];//邻接矩阵

};

