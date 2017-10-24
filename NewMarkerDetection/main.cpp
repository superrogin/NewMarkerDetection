// macro definition
#define _CRT_SECURE_NO_WARNINGS
#define MARKER_SIZE 90
#define MARKER_NUM 6
#define MARKER_UNIT MARKER_SIZE/MARKER_NUM
#define MARKING(x) x>127?1:0
#define CNTR_NUM 100
#define ADJUSTVAL(x,y,z) x<y?y:(x>z?z:x)
#define VIDWIDTH 960
#define VIDHEIGHT 540
#define ROTMARK(x,y) (x-y<0?x-y+4:x-y)+1			// x :: index to rotate, y::rotating number
#define MARKER_CM_LENGTH 6.2

// camera calibration parameters
#define camCX 2017.47490//1948.80688//1937.28741//1945.633150
#define camCY 1486.08297//1431.67602//1437.15993//1443.237510
#define camFX 1770.64430//1775.15273//1794.56938//1796.580704
#define camFY 1770.49741//1772.15075//1790.10075//1795.000832
#define camK1 -0.21876//-0.21624//-0.21791//-0.222282
#define camK2 0.04044//0.04145//0.04187//0.043809
#define camK3 0
#define camP1 -0.00044//0.00089//-0.00026//0.000126
#define camP2 -0.00041//0.00198//0.00207//0.001525
#define camSKEWC 0

// Including headers
#include <stdio.h>
#include <opencv2\opencv.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <Windows.h>
#include <tchar.h>
#include <iostream>
#include <fstream>

// namespace declaration
using namespace cv;
using namespace std;


// random seed
RNG rng(12345);

// image brightness and contrast conversion
void convertImage(Mat src, Mat dst, int valBright, int valContrast)
{
	int varBright = ADJUSTVAL(valBright, -100, 100);
	int varContrast = ADJUSTVAL(valContrast, -100, 100);

	double alpha, beta;

	if (varContrast > 0)
	{
		double delta = 127.0*varContrast / 100;
		alpha = 255.0 / (255.0 - delta * 2);
		beta = alpha * (varBright - delta);
	}
	else
	{
		double delta = -128.0*varContrast / 100;
		alpha = (256.0 - delta * 2) / 255.0;
		beta = alpha * varBright + delta;
	}

	src.convertTo(dst, CV_8U, alpha, beta);
}

double pntDist(Point2f pnt1, Point2f pnt2)
{
	return sqrtf((pnt1.x - pnt2.x)*(pnt1.x - pnt2.x) + (pnt1.y - pnt2.y)*(pnt1.y - pnt2.y));
}

void main()
{
	// open a video file with win32
	TCHAR szFilePath[MAX_PATH] = { 0, };
	OPENFILENAME ofn;
	ZeroMemory(&ofn, sizeof(OPENFILENAME));
	ofn.lStructSize = sizeof(OPENFILENAME);
	ofn.hwndOwner = NULL;
	ofn.lpstrFile = szFilePath;
	ofn.nMaxFile = sizeof(szFilePath);
	ofn.lpstrFilter = _T("video files(*.avi, *.mp4)\0*.avi;*.mp4\0All Files (*.*)\0*.*\0");
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = NULL;
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = NULL;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
	if (::GetOpenFileName(&ofn) == false) return;
	TCHAR *return_path = ofn.lpstrFile;

	// camera calibration parameters configuration
	Mat intrinsic = Mat(3, 3, CV_32FC1);
	Mat distCoeffs = Mat(4, 1, CV_32FC1);

	intrinsic.ptr<float>(0)[0] = camFX;
	intrinsic.ptr<float>(0)[1] = camSKEWC;
	intrinsic.ptr<float>(0)[2] = camCX;
	intrinsic.ptr<float>(1)[0] = 0;
	intrinsic.ptr<float>(1)[1] = camFY;
	intrinsic.ptr<float>(1)[2] = camCY;
	intrinsic.ptr<float>(2)[0] = 0;
	intrinsic.ptr<float>(2)[1] = 0;
	intrinsic.ptr<float>(2)[2] = 1;

	distCoeffs.ptr<float>(0)[0] = camK1;
	distCoeffs.ptr<float>(0)[1] = camK2;
	distCoeffs.ptr<float>(0)[2] = camP1;
	distCoeffs.ptr<float>(0)[3] = camP2;

	Mat distortedFrame, resizedFrame, frame, grayImg, binImg, cntrImg, boxImg, markerImg, vFrame;
	namedWindow("video", WINDOW_KEEPRATIO);
	resizeWindow("video", VIDWIDTH, VIDHEIGHT);

	// video recording configuration
	VideoCapture vidCap(return_path);
	if (!vidCap.isOpened())
	{
		cout << "can't open video file." << endl;
		return;
	}

	string tmpFileName = return_path;
	tmpFileName = tmpFileName.substr(int(tmpFileName.rfind('\\')) + 1, int(tmpFileName.rfind('.') - tmpFileName.rfind('\\')) - 1);

	double fps = 15;
	int fourcc = VideoWriter::fourcc('X', 'V', 'I', 'D'); // opencv3.0이상

	bool isColor = true;

	string strOutputFile = "result_";
	strOutputFile.append(tmpFileName);
	strOutputFile.append(".avi");

	VideoWriter *videoWrite = new VideoWriter;
	if (!videoWrite->open(strOutputFile, fourcc, fps, Size(VIDWIDTH, VIDHEIGHT), isColor)) {
		delete videoWrite;
		return;
	}

	strOutputFile = "output_";
	strOutputFile.append(tmpFileName);
	strOutputFile.append(".txt");
	ofstream outFile(strOutputFile);
	outFile << "cntX" << "\t" << "cntY" << "\t" << "tx" << "\t" << "ty" << "\t" << "tz" << "\t" << "rx" << "\t" << "ry" << "\t" << "rz" << "\t" << "txP" << "\t" << "tyP" << "\t" << "tzP" << "\t" << "rxP" << "\t" << "ryP" << "\t" << "rzP" << endl;

	bool bUndistort = true;

	KalmanFilter KF(12, 6); // 12 states, 6 measurements
	Mat_<float> state(12, 1); //	State vectors (x,y,a, dx,dy,da)
	Mat processNoise(12, 1, CV_32F); // process noise
	
	Mat_<float> measurement(6, 1); // 6 measurements
	measurement.setTo(Scalar(0)); // Initialize measurement

	bool bFoundBefore = false;

	double tx, ty, tz, rx, ry, rz;
	double txP, tyP, tzP, rxP, ryP, rzP;


	// main loop
	while (1)
	{
		vidCap >> distortedFrame;				// video frame read

		if (!distortedFrame.empty())			// successful video frame grab
		{
			// Main image processing
			//resize(distortedFrame, distortedFrame, Size(VIDWIDTH, VIDHEIGHT));

			frame = distortedFrame.clone();
			if (bUndistort)	undistort(distortedFrame, frame, intrinsic, distCoeffs);

			if (bUndistort)
			{
				cvtColor(frame, grayImg, CV_BGR2GRAY);

				convertImage(grayImg, grayImg, 55, 35);
				convertImage(grayImg, grayImg, -85, 55);

				//adaptiveThreshold(grayImg, binImg, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 201, 7);

				//threshold(grayImg, binImg, 128, 255, THRESH_BINARY);
				threshold(grayImg, binImg, 125, 255, THRESH_BINARY | THRESH_OTSU);

				binImg = 255 - binImg;

				medianBlur(binImg, binImg, 7);
			}
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			if(bUndistort) findContours(binImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

			cntrImg = frame.clone();
			boxImg = frame.clone();

			Point2f srcBox[4], dstBox[4];

			int marking[MARKER_NUM][MARKER_NUM];

			Rect rtROI;
			Mat meanMat = Mat(MARKER_UNIT, MARKER_UNIT, CV_8UC1);
			Mat checkerImg = Mat(MARKER_SIZE, MARKER_SIZE, CV_8UC1);
			Mat binChkImg = Mat(MARKER_SIZE, MARKER_SIZE, CV_8UC1);
			Mat rotBinImg = Mat(MARKER_SIZE, MARKER_SIZE, CV_8UC1);
			Mat psptv, markerCenter, rvec, tvec;

			bool bMarker = true;
			int nParity = 0;
			int nShell = 0;
			int whiteBright = 255;

			int rotNum = 0;
			int rotMark[MARKER_NUM][MARKER_NUM];

			if (bUndistort)
			{
				for (size_t i = 0; i < contours.size(); i++)
				{
					vector<Point> approxedPoly;
					approxPolyDP(contours[i], approxedPoly, 20.0, true);

					if (approxedPoly.size() == 4)
					{
						/*vector<vector<Point>> tmpContour;
						tmpContour[0] = approxedPoly;
						drawContours(boxImg, approxedPoly, 0, Scalar(0, 0, 255), 3, 8, hierarchy, 0, Point());*/
						line(boxImg, approxedPoly[0], approxedPoly[1], Scalar(0, 0, 255), 3);
						line(boxImg, approxedPoly[1], approxedPoly[2], Scalar(0, 0, 255), 3);
						line(boxImg, approxedPoly[2], approxedPoly[3], Scalar(0, 0, 255), 3);
						line(boxImg, approxedPoly[3], approxedPoly[0], Scalar(0, 0, 255), 3);

						dstBox[0] = Point2f(0, 0);
						dstBox[1] = Point2f(MARKER_SIZE, 0);
						dstBox[2] = Point2f(MARKER_SIZE, MARKER_SIZE);
						dstBox[3] = Point2f(0, MARKER_SIZE);
						srcBox[0] = (Point2f)approxedPoly[0];
						srcBox[1] = (Point2f)approxedPoly[1];
						srcBox[2] = (Point2f)approxedPoly[2];
						srcBox[3] = (Point2f)approxedPoly[3];

						psptv = getPerspectiveTransform(srcBox, dstBox);
						warpPerspective(frame, markerImg, psptv, Size(MARKER_SIZE, MARKER_SIZE));

						convertImage(markerImg, markerImg, -15, 95);

						bMarker = true;
						nParity = 0;
						nShell = 0;

						for (int i = 0; i < MARKER_NUM; i++)
						{
							for (int j = 0; j < MARKER_NUM; j++)
							{
								if (bMarker)
								{
									rtROI = Rect(i*MARKER_UNIT, j*MARKER_UNIT, MARKER_UNIT, MARKER_UNIT);

									meanMat = mean(markerImg(rtROI));
									checkerImg(rtROI) = mean(markerImg(rtROI));

									int binVal = MARKING(meanMat.at<uchar>(0, 0));
									marking[i][j] = binVal;

									binChkImg(rtROI) = 255 * binVal * Mat::ones(MARKER_UNIT, MARKER_UNIT, CV_8UC1);

									if ((i == 0 || i == MARKER_NUM - 1) || (j == 0 || j == MARKER_NUM - 1))
									{
										//bMarker = bMarker && binVal == 0;
										//printf("binVal :: %d , marking :: %d \n", binVal, marking[i][j]); 
										if (binVal == 0)
											nShell++;
									}
									else if ((i == 1 || i == MARKER_NUM - 2) && (j == 1 || j == MARKER_NUM - 2))
									{
										if (binVal == 0) nParity++;
										bMarker = bMarker && nParity < 2;
									}
								}
							}
						}
						bMarker = bMarker && nParity == 1;
						bMarker = bMarker && nShell == (MARKER_NUM - 1) * 4;
						if (bMarker) break;
						/*string tmpStr = to_string(nParity);
						putText(boxImg, tmpStr, approxedPoly[0]-Point(10,0), 2, 2, Scalar::all(255), 3);
						tmpStr = to_string(nShell);
						putText(boxImg, tmpStr, approxedPoly[3] - Point(10, 0), 2, 2, Scalar(0,0,255), 3);
						tmpStr = bMarker ? "fuck" : "damn";
						putText(boxImg, tmpStr, approxedPoly[1] - Point(10, 0), 2, 2, Scalar(255, 0, 0), 3);*/
					}
				}

				if (bMarker)
				{
					vector<Point2f> tmpPoints(srcBox, srcBox + sizeof(srcBox) / sizeof(srcBox)[0]);

					cornerSubPix(grayImg, tmpPoints, Size(5, 5), Size(-1, -1), TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.01));

					double cntX = (srcBox[0].x + srcBox[1].x + srcBox[2].x + srcBox[3].x)*0.25;
					double cntY = (srcBox[0].y + srcBox[1].y + srcBox[2].y + srcBox[3].y)*0.25;

					for (int k = 0; k < 3; k++)
					{
						if (marking[1][1])
						{
							rotNum++;
							for (int i = 0; i < MARKER_NUM; i++)
							{
								for (int j = 0; j < MARKER_NUM; j++)
								{
									rotMark[j][MARKER_NUM - i - 1] = marking[i][j];
								}
							}

							for (int i = 0; i < MARKER_NUM; i++)
							{
								for (int j = 0; j < MARKER_NUM; j++)
								{
									marking[i][j] = rotMark[i][j];

									rtROI = Rect(i*MARKER_UNIT, j*MARKER_UNIT, MARKER_UNIT, MARKER_UNIT);
									rotBinImg(rtROI) = 255 * marking[i][j] * Mat::ones(MARKER_UNIT, MARKER_UNIT, CV_8UC1);
								}
							}
						}
					}

					string tmpStr;
					Point2f tmpPnt;
					vector<Point2f> markerPnts(srcBox, srcBox + sizeof(srcBox) / sizeof(srcBox)[0]);

					for (int i = 0; i < 4; i++)
					{
						srcBox[i] = tmpPoints[i];
						circle(boxImg, srcBox[i], 5, Scalar(255, 0, 0), 8);

						int pntIdx = ROTMARK(i, rotNum);
						pntIdx = 2 - pntIdx > 0 ? 2 - pntIdx : 6 - pntIdx;
						tmpPnt = (srcBox[i] - Point2f(cntX, cntY))*0.5;
						tmpStr = to_string(pntIdx);
						putText(boxImg, tmpStr, srcBox[i] + tmpPnt - Point2f(20,-20), 2, 2, Scalar(0, 0, 255), 3);

						markerPnts[pntIdx - 1] = srcBox[i];
						//printf("%d\n", pntIdx);
					}
					//printf("\n");

					vector<cv::Point3f> markerCorners3d;
					markerCorners3d.push_back(Point3f(-3.05f, 3.05f, 0));
					markerCorners3d.push_back(Point3f(3.05f, 3.05f, 0));
					markerCorners3d.push_back(Point3f(3.05f, -3.05f, 0));
					markerCorners3d.push_back(Point3f(-3.05f, -3.05f, 0));

					solvePnP(markerCorners3d, markerPnts, intrinsic, distCoeffs, rvec, tvec);

					/*Mat R;
					Rodrigues(rvec, R);
					Mat R_inv = R.inv();

					Mat P = -R_inv*tvec;
					double* p = (double *)P.data;*/

					cout << tvec << "\n\n";

					tx = tvec.at<Vec3f>(0, 0)[0];
					ty = tvec.at<Vec3f>(1, 0)[0];
					tz = tvec.at<Vec3f>(2, 0)[0];

					rx = rvec.at<Vec3f>(0, 0)[0];
					ry = rvec.at<Vec3f>(1, 0)[0];
					rz = rvec.at<Vec3f>(2, 0)[0];


					// Kalman Filter~~!

					if (!bFoundBefore) // Initialization
					{
						KF.statePre.at<float>(0) = tx;
						KF.statePre.at<float>(1) = ty;
						KF.statePre.at<float>(2) = tz;
						KF.statePre.at<float>(3) = rx;
						KF.statePre.at<float>(4) = ry;
						KF.statePre.at<float>(5) = rz;

						KF.statePre.at<float>(6) = 0;
						KF.statePre.at<float>(7) = 0;
						KF.statePre.at<float>(8) = 0;
						KF.statePre.at<float>(9) = 0;
						KF.statePre.at<float>(10) = 0;
						KF.statePre.at<float>(11) = 0;

						setIdentity(KF.transitionMatrix);
						setIdentity(KF.measurementMatrix);
						setIdentity(KF.processNoiseCov, Scalar::all(1e-3));
						setIdentity(KF.measurementNoiseCov, Scalar::all(1e-2));
						setIdentity(KF.errorCovPost, Scalar::all(0.1));
					}

					Mat prediction = KF.predict();
					
					measurement(0) = tx;
					measurement(1) = ty;
					measurement(2) = tz;
					measurement(3) = rx;
					measurement(4) = ry;
					measurement(5) = rz;

					Mat estimated = KF.correct(measurement);

					txP = estimated.at<float>(0);
					tyP = estimated.at<float>(1);
					tzP = estimated.at<float>(2);
					rxP = estimated.at<float>(3);
					ryP = estimated.at<float>(4);
					rzP = estimated.at<float>(5);

					// Kalman Filter End



					Mat psptv = getPerspectiveTransform(srcBox, dstBox);
					warpPerspective(frame, markerImg, psptv, Size(MARKER_SIZE, MARKER_SIZE));
					convertImage(markerImg, markerImg, -15, 95);
					convertImage(markerImg, markerImg, 35, 55);

					//circle(boxImg, Point(cntX, cntY), 5, Scalar(255, 0, 0), 3);
					line(boxImg, srcBox[0], srcBox[2], Scalar(0, 255, 0), 3);
					line(boxImg, srcBox[1], srcBox[3], Scalar(0, 255, 0), 3);

					circle(boxImg, Point2f(cntX, cntY), 8, Scalar(255, 255, 255), 8);
					circle(boxImg, Point2f(cntX, cntY), 4, Scalar(255, 0, 0), 4);

					warpPerspective(boxImg, markerCenter, psptv, Size(MARKER_SIZE, MARKER_SIZE));
					line(markerCenter, Point(MARKER_SIZE*0.5, 0), Point(MARKER_SIZE*0.5, MARKER_SIZE), Scalar(0, 0, 255), 1);
					line(markerCenter, Point(0, MARKER_SIZE*0.5), Point(MARKER_SIZE, MARKER_SIZE*0.5), Scalar(0, 0, 255), 1);

					double pxLength = (pntDist(srcBox[0], srcBox[1]) + pntDist(srcBox[1], srcBox[2]) + pntDist(srcBox[2], srcBox[3]) + pntDist(srcBox[3], srcBox[0]))*0.25;

					cntX = cntX / pxLength * MARKER_CM_LENGTH;
					cntY = cntY / pxLength * MARKER_CM_LENGTH;

					string myText = "Marker Center Position";

					/// Font Face
					int myFontFace = 2;

					/// Font Scale
					double myFontScale = 5.2;

					putText(boxImg, myText, Point(10, 150), myFontFace, myFontScale, Scalar::all(255), 3);
					myText = to_string(cntX);
					putText(boxImg, myText, Point(10, 300), myFontFace, myFontScale, Scalar::all(255), 3);
					myText = to_string(cntY);
					putText(boxImg, myText, Point(10, 450), myFontFace, myFontScale, Scalar::all(255), 3);
					myText = to_string(tx);
					putText(boxImg, myText, Point(10, 600), myFontFace, myFontScale, Scalar::all(255), 3);
					myText = to_string(ty);
					putText(boxImg, myText, Point(10, 750), myFontFace, myFontScale, Scalar::all(255), 3);
					myText = to_string(tz);
					putText(boxImg, myText, Point(10, 900), myFontFace, myFontScale, Scalar::all(255), 3);

					outFile << cntX << "\t" << cntY << "\t" << tx << "\t" << ty << "\t" << tz << "\t" << rx << "\t" << ry << "\t" << rz << "\t" << txP << "\t" << tyP << "\t" << tzP << "\t" << rxP << "\t" << ryP << "\t" << rzP << endl;


					bFoundBefore = true;
				}
				else
				{
					bFoundBefore = false;
				}

				for (int i = 1; i < MARKER_NUM; i++)
				{
					line(markerImg, Point(MARKER_UNIT*i, 0), Point(MARKER_UNIT*i, MARKER_SIZE), Scalar(0, 0, 255));
					line(markerImg, Point(0, MARKER_UNIT*i), Point(MARKER_SIZE, MARKER_UNIT*i), Scalar(0, 0, 255));
				}

				namedWindow("rotatedBinary", WINDOW_KEEPRATIO);
				//namedWindow("binMarker", WINDOW_KEEPRATIO);
				//namedWindow("markerMosaic", WINDOW_KEEPRATIO);
				namedWindow("marker", WINDOW_KEEPRATIO);
				namedWindow("markerCenter", WINDOW_KEEPRATIO);
			}
			if (bUndistort) {
				resize(grayImg, grayImg, Size(VIDWIDTH, VIDHEIGHT));
				resize(binImg, binImg, Size(VIDWIDTH, VIDHEIGHT));
				//resize(cntrImg, cntrImg, Size(VIDWIDTH, VIDHEIGHT));
			}
			resize(boxImg, boxImg, Size(VIDWIDTH, VIDHEIGHT));

			if (bUndistort) {
				imshow("gray", grayImg);
				imshow("binary", binImg);
				//imshow("contours", cntrImg);
			}
			imshow("boxed", boxImg);

			if (bMarker && bUndistort)
			{
				for (int i = 0; i < rotNum; i++)
				{
					Point2f tmpPoint = srcBox[0];
					for (int j = 0; j < 3; j++)
					{
						srcBox[j] = srcBox[j + 1];
					}
					srcBox[3] = tmpPoint;
				}

				for (int i = 0; i < MARKER_NUM; i++)
				{
					for (int j = 0; j < MARKER_NUM; j++)
					{
						marking[i][j] = rotMark[i][j];

						rtROI = Rect(i*MARKER_UNIT, j*MARKER_UNIT, MARKER_UNIT, MARKER_UNIT);
						rotBinImg(rtROI) = 255 * marking[i][j] * Mat::ones(MARKER_UNIT, MARKER_UNIT, CV_8UC1);
					}
				}

				imshow("marker", markerImg);
				imshow("markerCenter", markerCenter);
				//imshow("markerMosaic", checkerImg);
				//imshow("binMarker", binChkImg);
				imshow("rotatedBinary", rotBinImg);
			}

			/*resize(frame, resizedFrame, Size(VIDWIDTH, VIDHEIGHT));

			imshow("video", resizedFrame);*/

			*videoWrite << boxImg;// resizedFrame;
		}
		else									// failed to grab a video frame
		{
			for (int i = 0; i < 100; i++)		// some tries to grab a frame if it failed
			{
				vidCap >> distortedFrame;
				if (!distortedFrame.empty()) break;
			}
			if (distortedFrame.empty())			// after the tries
			{
				delete videoWrite;
				outFile.close();
				return;							// video end. if you want to replay without finishing, remove this and the above line.
				vidCap = VideoCapture(return_path);
				if (!vidCap.isOpened())
				{
					cout << "can't open video file." << endl;
					return;
				}

				vidCap >> distortedFrame;
			}
		}

		char ch = waitKey(1);
		if (ch == 27) break;
		else if (ch == 32)
		{
			bUndistort = !bUndistort;
		}
	}

	delete videoWrite;
	outFile.close();
}
