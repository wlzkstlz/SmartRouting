#include"Graph.h"
#include "KeyDataStruct.h"
#include "SmartRouting.h"
using namespace cv;

enum UI_Mode
{
	UI_MODE_BORDER_AB,
	UI_MODE_MAINTUBE_AB,
	UI_MODE_AC_LOCATION,
	UI_MODE_COMMIT
};

int g_ui_mode=UI_MODE_COMMIT;
vector<vector<Point>>g_borderUIPts;
vector<Point>g_borderABs;
vector<Point>g_mainTubeUIPts;

bool is_ui_data_ready = false;

void on_mouse(int event, int x, int y, int flags, void *ustc);

int g_region_selected_id = -1;

void main()
{
	CSmartRouting smartRouting;
	smartRouting.ReadSourceImg("test.bmp");

	/*UI*/
	{
		namedWindow("UI", CV_WINDOW_FREERATIO);
		setMouseCallback("UI", on_mouse, &smartRouting);//调用回调函数  

		imshow("UI", smartRouting.mSourceMat);
		waitKey(-1);
	}
	/*UI END*/


	while (1)
	{
		/*UI*/
		{
			Mat mat4show = smartRouting.mSourceMat.clone();
			for (size_t i = 0; i < g_borderUIPts.size(); i++)
				for (size_t j = 1; j < g_borderUIPts[i].size(); j++)
					line(mat4show, g_borderUIPts[i][j - 1], g_borderUIPts[i][j], Scalar(255, 0, 0), ROOM_BORDER_PANT_WIDTH);

			for (size_t i = 1; i < g_borderABs.size(); i++)
				line(mat4show, g_borderABs[i - 1], g_borderABs[i], Scalar(0, 255, 0), ROOM_BORDER_PANT_WIDTH);

			for (size_t i = 1; i < g_mainTubeUIPts.size(); i++)
				line(mat4show, g_mainTubeUIPts[i - 1], g_mainTubeUIPts[i], Scalar(0, 0, 255), ROOM_BORDER_PANT_WIDTH);

			imshow("UI", mat4show);

			char a = waitKey(5);
			switch (a)
			{
			case 'A':
				g_ui_mode = UI_MODE_BORDER_AB;
				break;

			case 'B':
				g_ui_mode = UI_MODE_MAINTUBE_AB;
				break;

			case ' ':
				g_ui_mode = UI_MODE_COMMIT;
				if (g_borderUIPts.empty() == false)
					smartRouting.SetBorderUIPts(g_borderUIPts);
				if (g_mainTubeUIPts.empty() == false)
					smartRouting.SetMainTubeUIPts(g_mainTubeUIPts);
				is_ui_data_ready = true;
				break;

			case 'C':
				g_ui_mode = UI_MODE_AC_LOCATION;
				break;
			default:
				break;
			}
		}
		/*UI END*/

		if (is_ui_data_ready)
			break;
	}

	smartRouting.RecognizeRooms();
	smartRouting.RecognizeCorners();

	/*UI*/
	Mat mat4show_base = Mat::zeros(smartRouting.mMat4Draw.size(), CV_8UC3);
	cvtColor(smartRouting.mMat4Draw, mat4show_base, CV_GRAY2BGR);
	/*UI END*/

	while (1)
	{
		/*UI*/
		{
			Mat mat4show = mat4show_base.clone();
			if (g_region_selected_id>=0)
			{
				mat4show.setTo(Scalar(0, 255, 0), smartRouting.mRoomRegions[g_region_selected_id]);
			}

			for (size_t i=0;i<smartRouting.mACLocationsFromUI.size()-1;i++)
			{
				if (smartRouting.mACLocationsFromUI[i].x>=0&& smartRouting.mACLocationsFromUI[i].y >= 0)
				{
					circle(mat4show, smartRouting.mACLocationsFromUI[i], 8, Scalar(0, 255, 255), 2);
				}
			}

			imshow("UI", mat4show);

			char a = waitKey(5);
		}


		/*UI END*/
	}
}

void on_mouse(int event, int x, int y, int flags, void *ustc)
{
	CSmartRouting*smartRouting = (CSmartRouting*)ustc;
	switch (g_ui_mode)
	{
	case UI_MODE_BORDER_AB:
		if (event==CV_EVENT_RBUTTONDOWN)
		{
			g_borderABs.push_back(Point(x, y));

			if (g_borderABs.size()>=2)
			{
				g_borderUIPts.push_back(g_borderABs);
				g_borderABs.clear();
			}
		}
		else if (event==CV_EVENT_LBUTTONDOWN)
		{
			g_borderABs.push_back(Point(x,y));
		}
		break;

	case UI_MODE_MAINTUBE_AB:
		if (event == CV_EVENT_RBUTTONDOWN)
		{
			g_mainTubeUIPts.push_back(Point(x, y));
			if (g_mainTubeUIPts.size() >= 2)
			{
				smartRouting->SetMainTubeUIPts(g_mainTubeUIPts);
				g_mainTubeUIPts.clear();
			}
		}
		else if (event == CV_EVENT_LBUTTONDOWN)
			g_mainTubeUIPts.push_back(Point(x, y));
		break;

	case UI_MODE_AC_LOCATION:
		if (event == CV_EVENT_RBUTTONDOWN)
		{
			for (size_t i=0;i<smartRouting->mRoomRegions.size()-1;i++)
			{
				uchar*data = smartRouting->mRoomRegions[i].ptr<uchar>(y);
				if (data[x]>127)
				{
					g_region_selected_id = i;
					break;
				}
			}
		}
		else if (event == CV_EVENT_LBUTTONDOWN)
		{
			if (g_region_selected_id>=0)
			{
				smartRouting->mACLocationsFromUI[g_region_selected_id] = Point(x, y);
			}
		}
		break;
	case UI_MODE_COMMIT:
		break;
	default:
		break;
	}
}