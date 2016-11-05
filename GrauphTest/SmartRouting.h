#pragma once
#include <opencv2/opencv.hpp>
#include<vector>

#include"KeyDataStruct.h"
using namespace cv;

#define ROOM_BORDER_PANT_WIDTH	2//ǽ��߻��ʿ��
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
	
	

	vector<vector<Point>>mRoomContours;//���ؼ����ݡ����з��������ǽ����
	vector<Mat>mRoomRegions;
	//Mat mOutOfAllRoomRegion;
	//Mat mAllInOneRegion;

	vector<Point>mACLocationsFromUI;//(-1,-1)stands for undifined!



	vector<TKeyNode>mKeyNodes;//���ؼ����ݡ�


//public:
//	vector<bool>mIsSolved;//�Ƿ���������

	//ͼ���ݽṹ
public:
	vector<Point>***mConectiveEdges;// [mNodeSize][mNodeSize];//�ڽӾ���

};

