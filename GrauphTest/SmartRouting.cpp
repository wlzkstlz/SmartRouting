#include"SmartRouting.h"
#include"commonAlg.h"

#include"iostream"
using namespace std;

#define MAX_ROOM_NUM		25//支持最大房间数目
#define REGION_ROOM_VALUE_STEP	(255/MAX_ROOM_NUM)

#define		CORNER_SEARCH_RANGE		(4*ROOM_BORDER_PANT_WIDTH)
#define		CORNER_CLUSTER_RANGE		(4*ROOM_BORDER_PANT_WIDTH)

#define		MAIN_TUBE_DISCRETE_STEP		2.0

#define		INT_INFINITY	1000000

#define		DIST_THRESHOLD_BETWEEN_ROOMS_KN	(ROOM_BORDER_PANT_WIDTH*8.0)//不同房间节点被认为可以直接穿墙连接的距离阈值

#define		DIST_THRESHOLD_CLOSE_TO_MAIN_TUBE	(ROOM_BORDER_PANT_WIDTH*8.0)//认为空调与主管同贴一堵墙的距离阈值

CSmartRouting::CSmartRouting()
{
}

CSmartRouting::~CSmartRouting()
{
	if (mConnectiveMatrix)
	{
		for (size_t i = 0; i < mKeyNodes.size(); i++)
			if (mConnectiveMatrix[i])
				delete mConnectiveMatrix[i];
		delete mConnectiveMatrix;
	}
}

void CSmartRouting::ReadSourceImg(const char*file_name)
{
	mSourceMat = imread(file_name);
	mMat4Draw = Mat::zeros(mSourceMat.size(), CV_8UC1);
}

void CSmartRouting::RecognizeRooms()
{
	//【1】draw mBorderUIPts
	mMat4Draw.setTo(Scalar::all(255));
	for (size_t i = 0; i < mBorderUIPts.size(); i++)
		for (size_t j=1;j<mBorderUIPts[i].size();j++)
			line(mMat4Draw, mBorderUIPts[i][j-1], mBorderUIPts[i][j], Scalar::all(0), ROOM_BORDER_PANT_WIDTH);

	/*4SHOW*/
	{
		namedWindow("RecognizeRooms", CV_WINDOW_FREERATIO);
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

	//【3】将最外轮廓放到最后
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
			key_node.contour_id = min_id;
			key_node.type = KNT_CORNER;
			key_node.node_id = mKeyNodes.size();
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

		cvtColor(mat4show, mMat4Draw, CV_BGR2GRAY);
		//mMat4Draw = mat4show;
		}
		/*4SHOW END*/
}

void CSmartRouting::RecognizeMainTubeEnds()
{
	//【1】离散主管道点列	
	for (size_t i = 1; i < mMainTubeUIPts.size(); i++)
	{
		mMainTubeDiscretePts.push_back(mMainTubeUIPts[i - 1]);
		double angel = CCommonAlg::calcVectAngel((double)(mMainTubeUIPts[i].x - mMainTubeUIPts[i - 1].x), (double)(mMainTubeUIPts[i].y - mMainTubeUIPts[i - 1].y));
		double distance = sqrt(pow(mMainTubeUIPts[i].x - mMainTubeUIPts[i - 1].x, 2) + pow(mMainTubeUIPts[i].y - mMainTubeUIPts[i - 1].y, 2));
		double temp = MAIN_TUBE_DISCRETE_STEP;
		for (; temp < distance; temp += MAIN_TUBE_DISCRETE_STEP)
		{
			Point pt = mMainTubeUIPts[i - 1];
			pt.x += temp*cos(angel);
			pt.y += temp*sin(angel);
			mMainTubeDiscretePts.push_back(pt);
		}
	}

	//【2】判断离散点是否处于墙上
	Mat mat4draw = mMat4Draw.clone();
	mat4draw.setTo(Scalar(0));
	for (size_t i = 0; i < mBorderUIPts.size(); i++)
		for (size_t j = 1; j < mBorderUIPts[i].size(); j++)
			line(mat4draw, mBorderUIPts[i][j - 1], mBorderUIPts[i][j], Scalar(255), ROOM_BORDER_PANT_WIDTH);

	vector<bool>is_wall_pts;
	is_wall_pts.reserve(mMainTubeDiscretePts.size());
	for (size_t i = 0; i < mMainTubeDiscretePts.size(); i++)
	{
		uchar*data = mat4draw.ptr<uchar>(mMainTubeDiscretePts[i].y);
		is_wall_pts.push_back((bool)(data[mMainTubeDiscretePts[i].x]>127));
	}

	//【3】找出端点
	bool pre_on_wall = false;
	vector<bool>is_end_pts;
	is_end_pts.reserve(mMainTubeDiscretePts.size());
	for (size_t i = 0; i<is_wall_pts.size(); i++)
	{
		bool is_end_pt = false;
		if (is_wall_pts[i]!= pre_on_wall)
		{
			pre_on_wall = is_wall_pts[i];
			is_end_pt = true;
		}

		is_end_pts.push_back(is_end_pt);
	}

	//【4】生成TKeyNode
	for (size_t i = 0; i < is_end_pts.size(); i++)
	{
		if (is_end_pts[i])
		{
			int room_id = -1;
			int contour_id = -1;
			double min_dist = 100000.0;
			for (size_t j = 0; j < mRoomContours.size()-1; j++)
			{
				for (size_t k = 0; k < mRoomContours[j].size(); k++)
				{
					double dist = sqrt(pow(mRoomContours[j][k].x- mMainTubeDiscretePts[i].x,2) + pow(mRoomContours[j][k].y - mMainTubeDiscretePts[i].y, 2));
					if (dist<min_dist)
					{
						min_dist = dist;
						room_id = j;
						contour_id = k;
					}
				}
			}

			if (room_id>=0)
			{
				TKeyNode key_node;
				key_node.location = mRoomContours[room_id][contour_id];
				key_node.room_id = room_id;
				key_node.contour_id = contour_id;
				key_node.main_tube_discrete_id = i;
				key_node.type = KeyNodeType::KNT_MAIN_TUBE_END;
				key_node.node_id = mKeyNodes.size();
				mKeyNodes.push_back(key_node);
			}
		}
	}

	/*4SHOW*/
	{
		namedWindow("RecognizeMainTubeEnds", CV_WINDOW_FREERATIO);
		Mat mat4show_color = Mat::zeros(mat4draw.size(), CV_8UC3);
		cvtColor(mat4draw, mat4show_color, CV_GRAY2BGR);

		for (size_t i = 0; i < mMainTubeDiscretePts.size(); i++)
			circle(mat4show_color, mMainTubeDiscretePts[i], 1, Scalar(0, 255, 0));

		for (size_t i = 0; i < mKeyNodes.size(); i++)
			if (mKeyNodes[i].type== KeyNodeType::KNT_MAIN_TUBE_END)
				circle(mat4show_color, mKeyNodes[i].location, 2, Scalar(0, 0, 255));

		imshow("RecognizeMainTubeEnds", mat4show_color);
		imwrite("RecognizeMainTubeEnds.bmp", mat4show_color);
		waitKey(-1);
	}
	/*4SHOW END*/
}

void CSmartRouting::RecognizeACLocation()
{
	for (size_t i = 0; i < mACLocationsFromUI.size()-1; i++)
	{
		if (mACLocationsFromUI[i].x >= 0 && mACLocationsFromUI[i].y >= 0)
		{
			int contour_id = -1;
			double min_dist = 100000.0;

			for (size_t j = 0; j < mRoomContours[i].size(); j++)
			{
			
					double dist = sqrt(pow(mRoomContours[i][j].x- mACLocationsFromUI[i].x,2)+pow(mRoomContours[i][j].y - mACLocationsFromUI[i].y, 2));
					if (dist<min_dist)
					{
						min_dist = dist;
						contour_id = j;
					}
			}

			if (contour_id>=0)
			{
				TKeyNode key_node;
				key_node.location = mRoomContours[i][contour_id];
				key_node.room_id = i;
				key_node.contour_id = contour_id;
				key_node.type = KeyNodeType::KNT_AC;
				key_node.node_id = mKeyNodes.size();
				mKeyNodes.push_back(key_node);
			}
		}
	}
}

void CSmartRouting::CheckIsCloseToMainTube()
{
	for (size_t i = 0; i < mKeyNodes.size(); i++)
	{
		if (mKeyNodes[i].type== KeyNodeType::KNT_AC)
		{
			int min_id = -1;
			double min_dist = 1000000.0;
			for (size_t j = 0; j < mMainTubeDiscretePts.size(); j++)
			{
				double distance = sqrt(pow(mMainTubeDiscretePts[j].x-mKeyNodes[i].location.x,2)+
					pow(mMainTubeDiscretePts[j].y - mKeyNodes[i].location.y, 2));
				if (distance<min_dist)
				{
					min_dist = distance;
					min_id = j;
				}
			}

			if (min_dist<DIST_THRESHOLD_CLOSE_TO_MAIN_TUBE)
				mKeyNodes[i].close_to_maintube_id = min_id;
		}
	}
}

void CSmartRouting::ShowResult()
{
	namedWindow("ShowResult", CV_WINDOW_FREERATIO);
	Mat mat4show_color = Mat::zeros(mMat4Draw.size(), CV_8UC3);
	
	//1.画房间区域
	for (size_t i = 0; i < mRoomRegions.size(); i++)
		mat4show_color.setTo(Scalar::all(127), mRoomRegions[i]);

	//2.画描边线条
	for (size_t i = 0; i < mBorderUIPts.size(); i++)
		for (size_t j = 1; j < mBorderUIPts[i].size(); j++)
			line(mat4show_color, mBorderUIPts[i][j - 1], mBorderUIPts[i][j], Scalar(0, 255, 0), ROOM_BORDER_PANT_WIDTH);

	//3.画主管道线条
	for (size_t i = 1; i < mMainTubeUIPts.size(); i++)
		line(mat4show_color, mMainTubeUIPts[i- 1], mMainTubeUIPts[i], Scalar(0, 0, 255), ROOM_BORDER_PANT_WIDTH);

	//4.画空调标记位置
	for (size_t i = 0; i < mACLocationsFromUI.size()-1; i++)
		if (mACLocationsFromUI[i].x >= 0 && mACLocationsFromUI[i].y >= 0)
			circle(mat4show_color, mACLocationsFromUI[i], 5, Scalar(0x98, 0xfb, 0x98), 1);

	//5.画关键点位置
	for (size_t i = 0; i < mKeyNodes.size(); i++)
	{
		cout << "mKeyNodes[" << mKeyNodes[i].node_id << "]:type=" << mKeyNodes[i].type << ";	 room_id = " << mKeyNodes[i].room_id << ";	contour_id = " << mKeyNodes[i].contour_id << ";	location = " << mKeyNodes[i].location << ";	main_tube_discrete_id=" << mKeyNodes[i].main_tube_discrete_id << endl;
		switch (mKeyNodes[i].type)
		{
		case KeyNodeType::KNT_CORNER:
			circle(mat4show_color, mKeyNodes[i].location, 3, Scalar(0xcd, 0x00, 0xcd), 1);
			break;
		case KeyNodeType::KNT_AC :
			circle(mat4show_color, mKeyNodes[i].location, 3, Scalar(0xd4, 0xff, 0x7f), 1);
			break;
		case KeyNodeType::KNT_MAIN_TUBE_END :
			circle(mat4show_color, mKeyNodes[i].location, 3, Scalar(0x8e, 0x38, 0x8e), 1);
			break;
		default:
			break;
		}
	}

	//6显示与存储图片
	imshow("ShowResult", mat4show_color);
	imwrite("ShowResult.bmp", mat4show_color);
	waitKey(-1);
}

void CSmartRouting::CreateConnectiveMatrix()
{
	//【1】申请内存并初始化邻接矩阵
	mConnectiveMatrix = new int*[mKeyNodes.size()];
	for (size_t i = 0; i < mKeyNodes.size(); i++)
	{
		mConnectiveMatrix[i] = new int[mKeyNodes.size()];
		for (size_t j = 0; j < mKeyNodes.size(); j++)
			mConnectiveMatrix[i][j] = INT_INFINITY;
	}

	//【2】生成邻接矩阵值
	//4 test
	int total = 0;

	for (size_t i = 0; i < mKeyNodes.size(); i++)
	{
		cout << "【" << i<<"】";
		for (size_t j = 0; j < mKeyNodes.size(); j++)
		{
			if (i==j)
				mConnectiveMatrix[i][j] = 0;
			else if (mKeyNodes[i].room_id==mKeyNodes[j].room_id)//同一个房间内部节点
			{
				int id_s = mKeyNodes[j].contour_id < mKeyNodes[i].contour_id ? mKeyNodes[j].contour_id : mKeyNodes[i].contour_id;
				int id_l = mKeyNodes[j].contour_id > mKeyNodes[i].contour_id ? mKeyNodes[j].contour_id : mKeyNodes[i].contour_id;
				int increase_dist = id_l - id_s;
				int decrease_dist = mRoomContours[mKeyNodes[i].room_id].size()- increase_dist;
				mConnectiveMatrix[i][j] = increase_dist < decrease_dist ? increase_dist : -decrease_dist;//负数代表ID递减距离更近
			}
			else
			{
				double distance = sqrt(pow(mKeyNodes[i].location.x- mKeyNodes[j].location.x,2)+
					pow(mKeyNodes[i].location.y - mKeyNodes[j].location.y, 2));
				if (distance < DIST_THRESHOLD_BETWEEN_ROOMS_KN)//不同房间非常靠近的两个点可以穿墙连接
				{
					mConnectiveMatrix[i][j] = 1;
					total++;
				}
			}

			cout << mConnectiveMatrix[i][j] << ",	";
			
		}

		cout << ";" << endl;
	}

	cout << "total=" << total << endl;

}

void CSmartRouting::SolveShortestPathProblem()
{
	for (size_t i = 0; i < mKeyNodes.size(); i++)
	{
		if (mKeyNodes[i].type== KeyNodeType::KNT_AC&&mKeyNodes[i].close_to_maintube_id==-1)
		{
			mKeyNodes[i].path_list = new int[mKeyNodes.size()];
			mKeyNodes[i].dist_list = new int[mKeyNodes.size()];
			ShortestPath_Dijkstra(mKeyNodes.size(),i,mConnectiveMatrix, mKeyNodes[i].path_list, mKeyNodes[i].dist_list);

			int min_id = -1;
			int min_dist = INT_INFINITY;
			for (size_t j = 0; j < mKeyNodes.size(); j++)
			{
				if (mKeyNodes[j].type == KeyNodeType::KNT_MAIN_TUBE_END)
				{
					if (mKeyNodes[i].dist_list[j]<min_dist)
					{
						min_dist = mKeyNodes[i].dist_list[j];
						min_id = j;
					}
				}
			}

			mKeyNodes[i].closest_maintube_end_id = min_id;
		}
	}
}

void CSmartRouting::ShowShortestPaths()
{
	namedWindow("ShowShortestPaths", CV_WINDOW_FREERATIO);
	Mat mat4show_color = Mat::zeros(mMat4Draw.size(), CV_8UC3);

	//1.画房间区域
	for (size_t i = 0; i < mRoomRegions.size(); i++)
		mat4show_color.setTo(Scalar::all(127), mRoomRegions[i]);

	//2.画描边线条
	for (size_t i = 0; i < mBorderUIPts.size(); i++)
		for (size_t j = 1; j < mBorderUIPts[i].size(); j++)
			line(mat4show_color, mBorderUIPts[i][j - 1], mBorderUIPts[i][j], Scalar(0, 255, 0), ROOM_BORDER_PANT_WIDTH);

	//3.画主管道线条
	for (size_t i = 1; i < mMainTubeUIPts.size(); i++)
		line(mat4show_color, mMainTubeUIPts[i - 1], mMainTubeUIPts[i], Scalar(0, 0, 255), ROOM_BORDER_PANT_WIDTH);

	//4.画空调标记位置
	for (size_t i = 0; i < mACLocationsFromUI.size() - 1; i++)
		if (mACLocationsFromUI[i].x >= 0 && mACLocationsFromUI[i].y >= 0)
			circle(mat4show_color, mACLocationsFromUI[i], 5, Scalar(0x98, 0xfb, 0x98), 1);

	//5.画管道路径
	for (size_t i = 0; i < mKeyNodes.size(); i++)
	{
		if (mKeyNodes[i].type== KeyNodeType::KNT_AC)//对每个室内机
		{
			//与主管端点连接
			if (mKeyNodes[i].closest_maintube_end_id>=0)
			{
				int middle_node_id = mKeyNodes[i].closest_maintube_end_id;
				while (middle_node_id!=i)
				{
					int pre_node_id = mKeyNodes[i].path_list[middle_node_id];
					if (pre_node_id < 0)
						break;

					if (mConnectiveMatrix[middle_node_id][pre_node_id]==1)
						line(mat4show_color, mKeyNodes[middle_node_id].location, mKeyNodes[pre_node_id].location,Scalar(255,0,0),2);
					else
					{
						int contour_id_s = mKeyNodes[middle_node_id].contour_id < mKeyNodes[pre_node_id].contour_id ? 
							mKeyNodes[middle_node_id].contour_id : mKeyNodes[pre_node_id].contour_id;
						int contour_id_l = mKeyNodes[middle_node_id].contour_id > mKeyNodes[pre_node_id].contour_id ? 
							mKeyNodes[middle_node_id].contour_id : mKeyNodes[pre_node_id].contour_id;
						if (mConnectiveMatrix[middle_node_id][pre_node_id]>0)
							for (size_t j = contour_id_s+1; j <contour_id_l; j++)
								line(mat4show_color, mRoomContours[mKeyNodes[middle_node_id].room_id][j-1],
									mRoomContours[mKeyNodes[middle_node_id].room_id][j ], Scalar(255, 0, 0), 2);
						else
						{
							int cur_id = contour_id_l;
							int next_id = (cur_id + 1) % mRoomContours[mKeyNodes[middle_node_id].room_id].size();
							while (cur_id != contour_id_s)
							{
								line(mat4show_color, mRoomContours[mKeyNodes[middle_node_id].room_id][cur_id],
									mRoomContours[mKeyNodes[middle_node_id].room_id][next_id], Scalar(255, 0, 0), 2);
								cur_id = next_id;
								next_id= (cur_id + 1) % mRoomContours[mKeyNodes[middle_node_id].room_id].size();
							}
						}
					}
					middle_node_id = pre_node_id;
				}
			}
			//与主管端穿墙连接
			else if (mKeyNodes[i].close_to_maintube_id>=0)
				line(mat4show_color, mKeyNodes[i].location,mMainTubeDiscretePts[mKeyNodes[i].close_to_maintube_id], Scalar(255, 0, 0), 2);

			imshow("ShowShortestPaths", mat4show_color);
			waitKey(-1);
		}
	}

	//6显示与存储图片
	imshow("ShowShortestPaths", mat4show_color);
	imwrite("ShowShortestPaths.bmp", mat4show_color);
	waitKey(-1);
}


void CSmartRouting::ShortestPath_Dijkstra(int size, int v0, int** matrix, int*P, int *D)
{
	int v, w, k, min;
	bool *final=new bool[size];
	for (size_t v = 0; v < size; v++)
	{
		final[v] = false;
		D[v] = abs(matrix[v0][v]);
		P[v] =v0;
	}

	D[v0] = 0;
	final[v0] = true;

	for (v=0; v < size; v++)
	{
		min = INT_INFINITY;
		for (size_t w = 0; w < size; w++)
		{
			if (!final[w]&&D[w]<INT_INFINITY)
			{
				k = w;
				min = D[w];
			}
		}
		final[k] = true;

		for (w = 0; w < size; w++)
		{
			if (!final[w]&&(min+abs(matrix[k][w])<=D[w]))
			{
				D[w] = min + abs(matrix[k][w]);
				P[w] = k;
			}
		}
	}
}