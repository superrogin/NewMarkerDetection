#define VIDWIDTH 960
#define VIDHEIGHT 540

// camera calibration parameters
#define CX 1945.633150
#define CY 1443.237510
#define FX 1796.580704
#define FY 1795.000832
#define K1 -0.222282
#define K2 0.043809
#define K3 0
#define P1 0.000126
#define P2 0.001525
#define SKEWC 0

// Including headers
#include <opencv2\opencv.hpp>
//#include <stdio.h>
//#include <Windows.h>
//#include <tchar.h>

void main()
{
	int iii = 1;
}


//using namespace std;
//using namespace cv;
//
//void main()
//{
//	TCHAR szFilePath[MAX_PATH] = { 0, };
//	OPENFILENAME ofn;
//	ZeroMemory(&ofn, sizeof(OPENFILENAME));
//	ofn.lStructSize = sizeof(OPENFILENAME);
//	ofn.hwndOwner = NULL;
//	ofn.lpstrFile = szFilePath;
//	ofn.nMaxFile = sizeof(szFilePath);
//	ofn.lpstrFilter = _T("video files(*.avi, *.mp4)\0*.avi;*.mp4\0All Files (*.*)\0*.*\0");
//	ofn.nFilterIndex = 1;
//	ofn.lpstrFileTitle = NULL;
//	ofn.nMaxFileTitle = 0;
//	ofn.lpstrInitialDir = NULL;
//	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
//	if (::GetOpenFileName(&ofn) == false) return;
//	TCHAR *return_path = ofn.lpstrFile;
//
//	VideoCapture vidCap(return_path);
//
//	if (!vidCap.isOpened())
//	{
//		cout << "파일을 열수 없습니다." << endl;
//		return;
//	}
//
//	Mat intrinsic = Mat(3, 3, CV_32FC1);
//	Mat distCoeffs = Mat(4, 1, CV_32FC1);
//
//	intrinsic.ptr<float>(0)[0] = FX;
//	intrinsic.ptr<float>(0)[1] = SKEWC;
//	intrinsic.ptr<float>(0)[2] = CX;
//	intrinsic.ptr<float>(1)[1] = FY;
//	intrinsic.ptr<float>(1)[2] = CY;
//	intrinsic.ptr<float>(2)[2] = 1;
//
//	distCoeffs.ptr<float>(0)[0] = K1;
//	distCoeffs.ptr<float>(0)[1] = K2;
//	distCoeffs.ptr<float>(0)[2] = P1;
//	distCoeffs.ptr<float>(0)[3] = P2;
//
//	Mat vidFrame, resizedFrame, undistortedImage;
//	namedWindow("video", WINDOW_KEEPRATIO);
//	resizeWindow("video", VIDWIDTH, VIDHEIGHT);
//
//	while (1)
//	{
//		vidCap >> vidFrame;
//
//		if (!vidFrame.empty())
//		{
//			undistortedImage = vidFrame.clone();
//			//undistort(vidFrame, undistortedImage, intrinsic, distCoeffs);
//
//			resize(undistortedImage, resizedFrame, Size(VIDWIDTH, VIDHEIGHT));
//
//			imshow("video", resizedFrame);
//		}
//		else
//		{
//			for (int i = 0; i < 100; i++)
//			{
//				vidCap >> vidFrame;
//				if (!vidFrame.empty()) break;
//			}
//			if (vidFrame.empty())
//			{
//				vidCap = VideoCapture(return_path);
//				if (!vidCap.isOpened())
//				{
//					cout << "파일을 열수 없습니다." << endl;
//					return;
//				}
//
//				vidCap >> vidFrame;
//			}
//		}
//
//		if (waitKey(1) == 27) break;
//	}
//}
