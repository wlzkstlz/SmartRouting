#include"SmartRouting.h"


#define MAX_ROOM_NUM		25//支持最大房间数目
#define REGION_ROOM_VALUE_STEP	(255/MAX_ROOM_NUM)

#define		CORNER_SEARCH_RANGE		(4*ROOM_BORDER_PANT_WIDTH)
#define		CORNER_CLUSTER_RANGE		(4*ROOM_BORDER_PANT_WIDTH)

CSmartRouting::CSmartRouting()
{
}

CSmartRouting::~CSmartRouting()
{
}

void CSmartRouting::ReadSourceImg(const char*file_name)
{
	mSourceMat = imread(file_name);
	mMat4Draw = Mat::zeros(mSourceMat.size(), CV_8UC1);
}

void CSmartRouting::RecognizeRooms()
{
	/*4SHOW*/
	{
		namedWindow("RecognizeRooms", CV_WINDOW_FREERATIO);
	}
	/*4SHOW END*/

	//【1】draw mBorderUIPts
	mMat4Draw.setTo(Scalar::all(255));
	for (size_t i = 0; i < mBorderUIPts.size(); i++)
		for (size_t j=1;j<mBorderUIPts[i].size();j++)
			line(mMat4Draw, mBorderUIPts[i][j-1], mBorderUIPts[i][j], Scalar::all(0), ROOM_BORDER_PANT_WIDTH);

	/*4SHOW*/
	{
		imshow("RecognizeRooms", mMat4Draw);
		waitKey(-1);
	}
	/*4SHOW END*/

	//【2】findContour
	vector<vector<cv::Point>>contours;
	vector<cv::Vec4i>h;
	findContours(mMat4Draw.clone(), contours, h, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	for (size_t i = 0; i < contours.size(); i++)
	{
		if (h[i][3]<0)//区域外轮廓
		{
			mMat4Draw.setTo(Scalar::all(0));
			drawContours(mMat4Draw, contours, i, Scalar::all(255), -1);
			int child_id = h[i][2];
			while (child_id >= 0)
			{
				drawContours(mMat4Draw, contours, child_id, Scalar::all(0), -1);
				child_id = h[child_id][0];
			}

			/*4SHOW*/
			{
				imshow("RecognizeRooms", mMat4Draw);
				waitKey(-1);
			}
			/*4SHOW END*/

			mRoomContours.push_back(contours[i]);
			Mat region = mMat4Draw.clone();
			
			mRoomRegions.push_back(region);

			mACLocationsFromUI.push_back(Point(-1, -1));//initialize into undefined!
		}
	}

	int max_id = -1;
	double max_area = 0;
	for (size_t i=0;i<mRoomContours.size();i++)
	{
		double area = contourArea(mRoomContours[i]);
		if (area>max_area)
		{
			max_area = area;
			max_id = i;
		}
	}

	/*vector<vector<Point>>::iterator it_contour = mRoomContours.begin() + max_id;
	vector<Mat>::iterator it_region = mRoomRegions.begin() + max_id;*/

	//将最外轮廓放到最后
	vector<Point> temp_contour = mRoomContours[max_id];
	mRoomContours[max_id] = mRoomContours.back();
	mRoomContours.back() = temp_contour;

	Mat temp_region = mRoomRegions[max_id];
	mRoomRegions[max_id] = mRoomRegions.back();
	mRoomRegions.back() = temp_region;
}

void CSmartRouting::RecognizeCorners()
{
	/*4SHOW*/
	{
 		namedWindow("RecognizeCorners", CV_WINDOW_FREERATIO);
	}
	/*4SHOW END*/

	for (size_t i = 0; i < mRoomContours.size()-1; i++)//【遍历每个房间】
	{
		bool*is_corner_like = new bool[mRoomContours[i].size()];//可能的墙角点数组
		for (size_t j = 0; j < mRoomContours[i].size(); j++)//【遍历房间的每一个外轮廓点，生成可能的墙角点数组】
		{
			is_corner_like[j] = false;//init

			vector<int>region_types;
			for (int m = -CORNER_SEARCH_RANGE; m <= CORNER_SEARCH_RANGE; m++)
			{
				for (int n = -CORNER_SEARCH_RANGE; n <= CORNER_SEARCH_RANGE; n++)
				{
					int xx = mRoomContours[i][j].x +n;
					int yy = mRoomContours[i][j].y+ m;
					for (int k=0;k<mRoomRegions.size();k++)
					{
						uchar*data = (mRoomRegions[k]).ptr<uchar>(yy);
						if (data[xx]>127)
						{
							bool is_new = true;
							for (int q=0;q<region_types.size();q++)
								if (region_types[q] == k)
									is_new = false;
							if (is_new)
								region_types.push_back(int(k));
						}
					}

					/*char*data = mOutOfAllRoomRegion.ptr<char>(yy);
					if (data[xx] > 127)
					{
						bool is_new = true;
						for (int q = 0; q < region_types.size(); q++)
							if (region_types[q] == -1)
								is_new = false;
						if (is_new)
							region_types.push_back(int(-1));
					}*/
				}
			}

			if (region_types.size()>=3)
				is_corner_like[j] = true;
		}

		vector<Point3f>corner_centres;
		for (size_t j=0;j<mRoomContours[i].size();j++)
		{
			if (is_corner_like[j])
			{
				int belong_to = -1;
				for (size_t k = 0; k < corner_centres.size(); k++)
				{
					if (sqrt(pow(corner_centres[k].x - mRoomContours[i][j].x, 2) + pow(corner_centres[k].y - mRoomContours[i][j].y, 2)) < CORNER_CLUSTER_RANGE)
					{
						belong_to = k;
						break;
					}
				}

				if (belong_to == -1)
				{
					corner_centres.push_back(Point3f(mRoomContours[i][j].x, mRoomContours[i][j].y, 1.0));
					continue;
				}

				double w1 = ((double)corner_centres[belong_to].z / (double)(corner_centres[belong_to].z + 1.0));
				double w2 = 1.0 - w1;
				double zz = corner_centres[belong_to].z + 1.0;
				corner_centres[belong_to] = corner_centres[belong_to] * w1 + Point3f(mRoomContours[i][j].x, mRoomContours[i][j].y, 1.0)*w2;
				corner_centres[belong_to].z = zz;
			}
		}

		

		for (size_t s=0;s<corner_centres.size();s++)//【生成角点】
		{
			TKeyNode key_node;

			int min_id = -1;
			double min_dist = 100000.0;
			for (size_t j=0;j< mRoomContours[i].size(); j++)
			{
				double distance = sqrt(pow(corner_centres[s].x- mRoomContours[i][j].x,2) + pow(corner_centres[s].y - mRoomContours[i][j].y, 2));
				if (distance<min_dist)
				{
					min_dist = distance;
					min_id = j;
				}
			}

			key_node.location = mRoomContours[i][min_id];
			key_node.room_id = i;
			key_node.type = KNT_CORNER;
			mKeyNodes.push_back(key_node);
		}

		

		delete is_corner_like;
	}

	/*4SHOW*/
		{
		Mat mat4show = mSourceMat.clone();
		mat4show.setTo(Scalar::all(0));
		drawContours(mat4show, mRoomContours, -1, Scalar(0, 255, 0));

		for (size_t k = 0; k < mKeyNodes.size(); k++)
		{
			circle(mat4show, mKeyNodes[k].location, 4, Scalar(0, 0, 255), 1);
		}

		imshow("RecognizeCorners", mat4show);
		imwrite("RecognizeCorners.bmp", mat4show);
		waitKey(-1);
		}
		/*4SHOW END*/
}