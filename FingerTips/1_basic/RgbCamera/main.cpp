#define _USE_MATH_DEFINES
#include <cmath>

#include <iostream>
#include <sstream>

// NuiApi.hの前にWindows.hをインクルードする
#include <Windows.h>
#include <NuiApi.h>

#include <opencv2/opencv.hpp>

#define ERROR_CHECK( ret )  \
  if ( ret != S_OK ) {    \
    std::stringstream ss;	\
    ss << "failed " #ret " " << std::hex << ret << std::endl;			\
    throw std::runtime_error( ss.str().c_str() );			\
  }

const NUI_IMAGE_RESOLUTION CAMERA_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;

const int		U_A = 75;
const int		L_A = 55;
const int		K = 25;
const int		U_LIMIT = 190;
const int		L_LIMIT = 170;
const int		U_RANK = 5;
const int		SKIP = 10;
const double	REC_AB = (1.0 / (K * K));
const double	REC_PI = (1.0 / M_PI);
const double	SCOPE = 0.8;

typedef struct
{
	std::vector< cv::Point >::iterator ptr_c;
	std::vector< cv::Point >::iterator ptr_r;
	std::vector< cv::Point >::iterator ptr_l;
	int param = NULL;
} wireframe;

typedef struct
{
	std::vector< cv::Point >::iterator head;
	std::vector< cv::Point >::iterator foot;
	std::vector< cv::Point >::iterator itr;
} itrs;

std::vector< cv::Point >::iterator getDoppelItr(std::vector< std::vector< cv::Point > >::iterator it, std::vector< cv::Point >::iterator itr)
{
	std::vector< cv::Point >::iterator doppel = it->begin();
	std::vector< cv::Point >::iterator lower = it->end();

	while (true)
	{
		if (doppel == itr || doppel == lower)
			break;
		else
			++doppel;
	}// end while

	return doppel;
}// end getDoppelItr

int getVectorLength(cv::Point *point)
{
	return static_cast<int>(pow(point->x * point->x + point->y * point->y, 0.5));
}// end getVectorLength

int getAngle(std::vector< cv::Point >::iterator a, std::vector< cv::Point >::iterator b, std::vector< cv::Point >::iterator c)
{

	double cos_sita = ((a->x - c->x) * (b->x - c->x) + (a->y - c->y) * (b->y - c->y)) * REC_AB;
	double sita = acos(cos_sita);

	sita *= 180 * REC_PI;

	return static_cast<int>(sita);
}// end getAngle

std::vector< cv::Point >::iterator getPoint(itrs *points, std::vector< cv::Point >::iterator local_itr, const char *v)
{
	int v_length = 0;
	cv::Point t_point;
	local_itr = (v == "R" ? (points->itr == points->foot ? points->head : ++local_itr) : (points->itr == points->head ? points->foot : --local_itr));
	int couter = 0;

	while (true)
	{
		++couter;
		t_point.x = local_itr->x - points->itr->x;
		t_point.y = local_itr->y - points->itr->y;

		v_length = getVectorLength(&t_point);

		if (v_length == K)
			break;
		else if (points->itr == local_itr || K < v_length)
			break;

		local_itr = (v == "R" ? (local_itr == points->foot ? points->head : ++local_itr) : (local_itr == points->head ? points->foot : --local_itr));
	}// end while

	return local_itr;
}// end getPoint

void topdownSort(wireframe * array)
{
	wireframe temp;

	for (int i = 0; i < 4; ++i)
		for (int j = i + 1; j < 5; ++j)
			if (array[i].param > array[j].param)
			{
				temp = array[i];
				array[i] = array[j];
				array[j] = temp;
			}// end if
}// end topdownSort

void drawFingerTips(const cv::Point *palm, std::vector< std::vector< cv::Point > >::iterator it, cv::Mat *img)
{
	std::vector< cv::Point >::iterator r_it;
	std::vector< cv::Point >::iterator l_it;
	cv::Point len_mat;
	wireframe minimum;
	wireframe best_5[U_RANK];
	itrs c_points;
	int temp_length;
	int angle;
	int counter = 0;

	minimum.param = NULL;
	c_points.head = it->begin();
	c_points.foot = --it->end();
	c_points.itr = it->begin();

	for (std::vector< cv::Point >::iterator bottom = c_points.foot - (2 + 1); c_points.itr < bottom; c_points.itr += 2)
	{
		r_it = getPoint(&c_points, getDoppelItr(it, c_points.itr), "R");
		l_it = getPoint(&c_points, getDoppelItr(it, c_points.itr), "L");

		if (r_it == c_points.itr || l_it == c_points.itr)
			continue;

		angle = getAngle(r_it, l_it, c_points.itr);

		if (angle >= L_LIMIT && angle <= U_LIMIT)
		{
			if (minimum.param != NULL)
			{
				len_mat.x = minimum.ptr_c->x - palm->x;
				len_mat.y = minimum.ptr_c->y - palm->y;

				temp_length = getVectorLength(&len_mat);

				if (best_5[0].param < temp_length || best_5[0].param == NULL)
				{
					best_5[0] = minimum;
					best_5[0].param = temp_length;

					if (counter < 5)
						++counter;

					topdownSort(best_5);
				}// end if

				cv::line(*img, cv::Point(minimum.ptr_c->x, minimum.ptr_c->y), cv::Point(minimum.ptr_r->x, minimum.ptr_r->y), cv::Scalar(255, 00, 00), 2);
				cv::line(*img, cv::Point(minimum.ptr_c->x, minimum.ptr_c->y), cv::Point(minimum.ptr_l->x, minimum.ptr_l->y), cv::Scalar(255, 00, 00), 2);

				minimum.param = NULL;
			}// end if
		}
		else if (angle <= U_A && angle > L_A)
		{
			if (angle < minimum.param || minimum.param == NULL)
			{
				minimum.ptr_c = c_points.itr;
				minimum.ptr_r = r_it;
				minimum.ptr_l = l_it;
				minimum.param = angle;
			}// end if
		}// end if
	}// end for

	for (int i = 4, size = 5 - counter; i >= size; --i)
	{
		cv::circle(*img, cv::Point(best_5[i].ptr_c->x, best_5[i].ptr_c->y), 10, cv::Scalar(00, 100, 255), 1);
	}// end for

}// end drawFingerTips

cv::Point getPalm(std::vector< std::vector< cv::Point > >::iterator it, unsigned char *ptr, int img_c, int img_r)
{
	int max = 0;
	int min = 9999;
	int len;
	double rec_img_c = 1.0 / img_c;
	cv::Point palm;
	cv::Point t_palm;
	cv::Point temp;
	itrs c_points;

	c_points.head = it->begin();
	c_points.foot = it->end();
	c_points.itr = it->begin();

	for (int i = 0, size = img_r * img_c, x, y; i < size; i += SKIP, ptr += SKIP)
	{
		min = 9999;

		if (*ptr)
		{
			while (c_points.itr < c_points.foot - SKIP * 2)
			{
				temp.x = c_points.itr->x - (x = i % img_c);
				temp.y = c_points.itr->y - (y = static_cast<int>(i * rec_img_c));

				len = getVectorLength(&temp);

				if (min > len)
				{
					min = len;
					t_palm.x = x;
					t_palm.y = y;
				}// end if

				c_points.itr += SKIP;
			}// end while

			if (min > max)
			{
				max = min;
				palm.x = t_palm.x;
				palm.y = t_palm.y;
			}// end if
		}// end if

		if (c_points.itr != c_points.head)
			c_points.itr = it->begin();
	}// end for

	return palm;
}// end getPalm

void fill_Internal(std::vector< std::vector< cv::Point > >::iterator it, cv::Mat *binImage)
{
	int x = 0, y = 0, v = 0, ptr = 0;
	int bin_col = binImage->cols;
	int bin_row = binImage->rows - 1;
	std::vector< int > move;
	std::vector< cv::Point >::iterator itr = it->begin();
	
	while (itr != it->end())
	{
		if (binImage->data[itr->y * binImage->step + itr->x * binImage->elemSize() + 1] != 0)
		{
			x = itr->x;
			y = itr->y;

			break;
		}// end if

		++itr;
	}// end while

	while (true) {
		ptr = static_cast<int>(y * binImage->step + x * binImage->elemSize() + 1);

		if ((x < -1 || y < 0 || x > bin_col || y > bin_row) || binImage->data[ptr] == 0)
		{
			switch (v)
			{
			case 0:
				++v;
				++x;
				--y;
				break;
			case 1:
				++v;
				++x;
				++y;
				break;
			case 2:
				++v;
				--x;
				++y;
				break;
			case 3:
				v = move.back();
				move.pop_back();
				--y;
			default:
				break;
			}// end switch
		}
		else
		{
			binImage->data[ptr] = 0;
			move.push_back(v);

			if (v == 2)
			{
				v = 1;
				--y;
			}
			else
			{
				v = 0;
				--x;
			}// end if
		}// end if

		if (move.size() == 0)
			break;
	}// end while
}// end fill_Internal

std::vector< std::vector< cv::Point > > getContours(cv::Mat *img, cv::Mat *bin)
{
	cv::Mat tempImage;
	std::vector< std::vector< cv::Point > > tempContours;
	int counter_min = 0;
	//int counter_max = 0;

	cv::threshold(*img, tempImage, 0.0, 255.0, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);	// 2値化
	cv::findContours(tempImage, tempContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);	// 輪郭の抽出

	for (int i = 0, size_i = static_cast<int>(tempContours.size()); i < size_i; ++i)
		for (int k = 0, size_k = static_cast<int>(tempContours[i].size()); k < size_k; ++k)
			if (tempContours[i][k].x == 1)
				++counter_min;
			//else if (tempContours[i][k].x == 639)
			//	++counter_max;

	if (counter_min > tempImage.rows * SCOPE) /*|| counter_max > tempImage.rows * SCOPE)*/
	{
		cv::threshold(*img, tempImage, 0.0, 255.0, CV_THRESH_BINARY | CV_THRESH_OTSU);// 反転
		cv::threshold(*img, *bin, 0.0, 255.0, CV_THRESH_BINARY | CV_THRESH_OTSU);
		cv::findContours(tempImage, tempContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	}
	else
	{
		cv::threshold(*img, *bin, 0.0, 255.0, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
	}// end if

	return tempContours;
}// end getContours

class KinectSample
{
private:

	INuiSensor* kinect;
	HANDLE imageStreamHandle;
	HANDLE depthStreamHandle;
	HANDLE streamEvent;

	DWORD width;
	DWORD height;

public:

	KinectSample()
	{
	}// end KinectSample

	~KinectSample()
	{
		// 終了処理
		if ( kinect != 0 ) {
			kinect->NuiShutdown();
			kinect->Release();
		}// end if
	}// end ~KinectSample

	void initialize()
	{
		createInstance();

		// Kinectの設定を初期化する
		ERROR_CHECK( kinect->NuiInitialize( NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH) );

		// RGBカメラを初期化する
		ERROR_CHECK( kinect->NuiImageStreamOpen( NUI_IMAGE_TYPE_COLOR, CAMERA_RESOLUTION, 0, 2, 0, &imageStreamHandle ) );

		// 距離カメラ
		ERROR_CHECK(kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, CAMERA_RESOLUTION, 0, 2, 0, &depthStreamHandle));

		// フレーム更新イベントのハンドル作成(Win32ApiのCreateEventでハンドラーを作成)
		streamEvent = ::CreateEvent(0, TRUE, FALSE, 0);

		// kinectオブジェクトにフレーム更新イベントを設定する
		ERROR_CHECK(kinect->NuiSetFrameEndEvent(streamEvent, 0));

		// 指定した解像度の、画面サイズを取得する
		::NuiImageResolutionToSize(CAMERA_RESOLUTION, width, height );
	  }// end initialize

	void run()
	{
		cv::Mat image;

		// メインループ
		while ( 1 ) {
			drawRgbImage(image);

			// 画像を表示する
			cv::imshow( "Kinect Sample", image );

			// 終了のためのキー入力チェック兼、表示のためのウェイト
			int key = cv::waitKey( 10 );
			if ( key == 'q' ) {
				break;
			}// end if
		}// end while
	}// end run

private:

	void createInstance()
	{
		// 接続されているKinectの数を取得する
		int count = 0;
		ERROR_CHECK( ::NuiGetSensorCount( &count ) );
		if ( count == 0 ) {
			throw std::runtime_error( "Kinect を接続してください" );
		}// end if

		// 最初のKinectのインスタンスを作成する
		ERROR_CHECK( ::NuiCreateSensorByIndex( 0, &kinect ) );

		// Kinectの状態を取得する
		HRESULT status = kinect->NuiStatus();
		if ( status != S_OK ) {
				throw std::runtime_error( "Kinect が利用可能ではありません" );
		}// end if
	}// end if

	void drawRgbImage( cv::Mat& image )
	{
		// RGBカメラのフレームデータを取得する
		NUI_IMAGE_FRAME imageFrame = { 0 };
		ERROR_CHECK( kinect->NuiImageStreamGetNextFrame( imageStreamHandle, INFINITE, &imageFrame ) );

		// 画像データを取得する
		NUI_LOCKED_RECT colorData;
		imageFrame.pFrameTexture->LockRect( 0, &colorData, 0, 0 );

		// 画像データをコピーする
		image = cv::Mat( height, width, CV_8UC4, colorData.pBits );
//////
		// ぼかし
		cv::Mat filtered;
		cv::GaussianBlur(image, filtered, cv::Size(39, 39), 8, 0);

		// グレースケール
		cv::Mat grayImage, binImage;
		cv::cvtColor(filtered, grayImage, CV_BGR2GRAY);

		// 輪郭の座標リスト
		std::vector< std::vector< cv::Point > > contours = getContours(&grayImage, &binImage);

		// 輪郭と内部の選別
		for (std::vector< std::vector< cv::Point > >::iterator max = contours.begin(); contours.size() != 1;)
		{
			if ((max + 1)->size() > max->size())
			{
				fill_Internal(max, &binImage);
				contours.erase(std::remove(contours.begin(), contours.end(), *max), contours.end());
			}
			else
			{
				fill_Internal(max + 1, &binImage);
				contours.erase(std::remove(contours.begin(), contours.end(), *(max + 1)), contours.end());
			}// end if
		}// end for

		 // 検出された輪郭線を緑で描画
		for (auto contour = contours.begin(); contour != contours.end(); ++contour) {
			cv::polylines(image, *contour, true, cv::Scalar(0, 255, 0), 2);
		}// end for

		// 手のひらの検出
		cv::Point palm = getPalm(contours.begin(), binImage.ptr<unsigned char>(0), binImage.cols, binImage.rows);
		cv::circle(image, cv::Point(palm.x, palm.y), 10, cv::Scalar(00, 00, 255), -1);

		// 指先の検出と描画
		drawFingerTips(&palm, contours.begin(), &image);
//////
		//cv::namedWindow("result", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
		//cv::imshow("result", binImage);
		//cv::waitKey(0);

		// フレームデータを解放する
		ERROR_CHECK( kinect->NuiImageStreamReleaseFrame( imageStreamHandle, &imageFrame ) );
	}// end drawRgbImage
};// end KinectSample

void main()
{
	try
	{
		KinectSample kinect;
		kinect.initialize();
		kinect.run();
	}
	catch ( std::exception& ex )
	{
		std::cout << ex.what() << std::endl;
	}// end try
}// end main