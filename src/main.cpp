#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif


#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "utils.h"
#include "ReadFromXmlAndRectify.h"

#define SHOW true
//#define SWAP(a, b) {typeof(a) _t=a; a=b; b=_t;} 

using namespace cv;
using namespace std;


void addObjPts(const Size& board_size, const int square_size, vector<Point3f>& obj_pts);

// ����ȫ�ֱ���
bool image_capture = false;  // ����ȫ�ֱ�������������ɼ��궨ͼ��
bool show_image = true;      // �Ƿ���ʾ�м��ע����
char win_name_1[100];        // ����1����
char win_name_2[100];        // ����2����


//����¼���Ӧ��������������ɼ��궨ͼ��˫��������һ�βɼ�һ��ͼ��
void on_mouse(int event, int x, int y, int flags, void* a)
{
	if (event == EVENT_LBUTTONDBLCLK)
	{
		image_capture = true;
	}
}

int runCalibrateAndRectify();
int readParamsFromXmlAndRectify_old();
int readParamsFromXmlAndRectify();

// to show rectifying
int showStereoAlignment(const string& root, const Size& img_size, const string& ext);

const int getDirs(const string& path, vector<string>& dirs);
const int getFilesFormat(const string& path, const string& format, vector<string>& files);


// ------------------------------------------
int runCalibrateAndRectify()  // 
{
	// **********
	//const char ckboard_name[50] = "CalibStereoMeasure";  // Calibration_Image_Camera1

	//Size board_size = Size(10, 8);   // �궨���̸���ڽǵ�ߴ�(��7x7): cols, rows

	const float square_size = 20.0;  // �궨���Ϻڰ׸��ӵ�ʵ�ʱ߳���mm��
	int nFrames = 27;                // ���ڱ궨��ͼ����Ŀ: 20, 16, 11
	// **********

	string output_file_name;         // ����ļ�������
	bool show_undistorted = true;    // �Ƿ���ӻ��������
	vector<string> left_img_paths;   // ����ͼͼ������(·��)�б�
	vector<string> right_img_paths;  // ����ͼͼ������(·��)�б�
	Size img_size;                   // ����궨ͼ��ĳߴ�

	int MODE = 0;  // ��ʼ��
	std::cout << "����һ��˫Ŀ�Ӿ��궨��������" << endl;
	std::cout << "���ȣ���ѡ��������궨ģʽ��1��������ͷ�ɼ��궨ͼ�񣩻�2����ͼ�����л�ȡ�궨ͼ��" << endl;
	cin >> MODE;

	cout << "���������̸�Ŀ¼·��: (��Ŀ¼�ṹ: left��right)" << endl;
	string checkboard_path;
	cin >> checkboard_path;
	cout << "Checkboard path: " << checkboard_path << endl;

	cout << "����������궨�ļ�·��:" << endl;
	string res_xml_path;
	cin >> res_xml_path;
	cout << "Output xml path: " << res_xml_path << endl;

	cout << "���������̸�x�����ڽǵ����: " << endl;
	int n_corners_x;
	cin >> n_corners_x;

	cout << "���������̸�y�����ڽǵ����: " << endl;
	int n_corners_y;
	cin >> n_corners_y;

	cout << "��ѡ���Ƿ�ͼ��˳ʱ����ת90��: (1: ��, 0: ��)" << endl;
	int rotate90cw;
	cin >> rotate90cw;

	if (rotate90cw)  // SWAP n_corners_x and n_corners_y
	{
		int tmp = n_corners_x;
		n_corners_x = n_corners_y;
		n_corners_y = tmp;
		cout << "Corner numbers swapped for the x and y direction, "
			<< "now n_corners_x: " << n_corners_x << ", n_corners_y: " << n_corners_y << endl;
	}

	Size board_size = Size(n_corners_x, n_corners_y);   // �궨���̸���ڽǵ�ߴ�(��7x7): cols, rows

	/************************************************************************************/
	/*********************�����ʵʱ�ɼ��궨ͼ�񲢱��浽ָ���ļ���***********************/
	//�����롰1���������������������Ԥ�����ں��������ɼ��궨ͼ��
	//��Ҫ�ɼ�ͼ�����Ŀ�ɱ���nrFrames�������ɹ���nrFrames��ͼ��Ԥ�������Զ��ر�
	if (MODE == 1)
	{
		//VideoCapture inputCapture;
		VideoCapture input_capture_1;
		VideoCapture input_capture_2;
		input_capture_1.open(1);
		input_capture_2.open(2);

		//if(!inputCapture.isOpened()==true) return -1;
		if (!input_capture_1.isOpened() == true)
		{
			std::printf("Open camera 1 failed.\n");
			return -1;
		}
		if (!input_capture_2.isOpened() == true)
		{
			std::printf("Open camera 2 failed.\n");
			return -1;
		}

		// �������ɼ�ͼ��ķֱ��ʴ�С  
		input_capture_1.set(CAP_PROP_FRAME_WIDTH, 960);    // 640
		input_capture_1.set(CAP_PROP_FRAME_HEIGHT, 720);   // 480
		input_capture_2.set(CAP_PROP_FRAME_WIDTH, 960);
		input_capture_2.set(CAP_PROP_FRAME_HEIGHT, 720);

		if (show_image)
		{
			cv::namedWindow("�����(���1)�궨ͼ��ɼ�Ԥ������", WINDOW_AUTOSIZE);
			cv::namedWindow("�����(���2)�궨ͼ��ɼ�Ԥ������", WINDOW_AUTOSIZE);

			//��������¼���������������ɼ��궨ͼ����ָ������˫��������һ�βɼ�һ��ͼ��
			cv::setMouseCallback("����������1���궨ͼ��ɼ�Ԥ������", on_mouse, NULL);
		}

		//Mat src_image;
		Mat src_img_1;
		Mat src_img_2;
		int capture_count = 0;

		// ��ʱ�ɼ�: 10s���
		clock_t start_time;
		clock_t end_time;
		double  duration;
		start_time = clock();

		while (1)
		{
			input_capture_1 >> src_img_1;
			input_capture_2 >> src_img_2;

			if (show_image)  // ���������ʾ����
			{
				if (rotate90cw)
				{
					cv::Mat src1, src2;
					cv::rotate(src_img_1, src1, cv::ROTATE_90_CLOCKWISE);
					cv::rotate(src_img_2, src2, cv::ROTATE_90_CLOCKWISE);

					imshow("�����(���1)�궨ͼ��ɼ�Ԥ������", src1);
					imshow("�����(���2)�궨ͼ��ɼ�Ԥ������", src2);
					cv::waitKey(50);  // 35
				}
				else
				{
					imshow("�����(���1)�궨ͼ��ɼ�Ԥ������", src_img_1);
					imshow("�����(���2)�궨ͼ��ɼ�Ԥ������", src_img_2);
					cv::waitKey(50);  // 35
				}
			}
			else  // ���������ʾ����, ÿ��10��ɼ�һ��
			{
				end_time = clock();
				duration = (double)(end_time - start_time) / CLOCKS_PER_SEC;
				if (duration > 10)  // ���ʱ��10s
				{
					image_capture = true;
				}
			}

			if (image_capture == true && capture_count < nFrames)
			{
				Mat cap_1;
				Mat cap_2;

				input_capture_1 >> cap_1;
				input_capture_2 >> cap_2;

				// ���ñ궨ͼ����·��, ������
				char save_path[100];

				if (rotate90cw)
				{
					cv::Mat cap1, cap2;
					cv::rotate(cap_1, cap1, cv::ROTATE_90_CLOCKWISE);
					cv::rotate(cap_2, cap2, cv::ROTATE_90_CLOCKWISE);

					sprintf(save_path, "%s/l_%d%s",
						checkboard_path.c_str(),
						capture_count + 1,
						".jpg");
					cv::imwrite(save_path, cap1);

					sprintf(save_path, "%s/r_%d%s",
						checkboard_path.c_str(),
						capture_count + 1,
						".jpg");
					cv::imwrite(save_path, cap2);
				}
				else
				{
					sprintf(save_path, "%s/l_%d%s",
						checkboard_path.c_str(),
						capture_count + 1,
						".jpg");
					cv::imwrite(save_path, cap_1);

					sprintf(save_path, "%s/r_%d%s",
						checkboard_path.c_str(),
						capture_count + 1,
						".jpg");
					cv::imwrite(save_path, cap_2);
				}

				capture_count++;
				image_capture = false;
			}
			else if (capture_count >= nFrames)
			{
				std::cout << "�궨ͼ��ɼ���ϣ����ɼ���" << capture_count << "�ű궨ͼ��" << endl;
				if (show_image)
				{
					cv::destroyWindow("�����(���1)�궨ͼ��ɼ�Ԥ������");
					cv::destroyWindow("�����(���2)�궨ͼ��ɼ�Ԥ������");
				}

				image_capture = false;
				break;
			}
		}
	}

	/***********************************************************************************/
	/*******************����ű궨ͼ���·�����뵽imageList��1/2��������****************/
	if (MODE == 1 || MODE == 2)
	{
		string left_dir = checkboard_path + string("/left");
		string right_dir = checkboard_path + string("/right");
		getFilesFormat(left_dir, ".jpg", left_img_paths);
		getFilesFormat(right_dir, ".jpg", right_img_paths);

		assert(left_img_paths.size() == right_img_paths.size());

		nFrames = (int)left_img_paths.size();
	}

	std::cout << "Image list 1 size:" << left_img_paths.size() << endl;
	std::cout << "Image list 2 size:" << right_img_paths.size() << endl;
	if (left_img_paths.size() == 0 || right_img_paths.size() == 0)
	{
		cout << "[Err]: empty iamge pairs, please check the checkboard dir path!\n";
		return -1;
	}

	/************************************************************************************/
	/****************�궨ǰ��׼����������ȡObject Points��Image Points*********************/
	//1 ���ȸ���square size(���̸��ʵ�ʱ߳�������20mm)��board size(���̸�Ľǵ���������5x5)��
	// ����forѭ���õ��ǵ����������Object Points��Z�������Ϊ0��
	//2 ����forѭ����findChessboardCorners()�����õ���ǵ�������������objectPoints��Ӧ��ͼ��������������imagePoints

	vector<vector<Point2f>> img_pts1;     // �������ͼ����ͼ���ƽ��ǵ� 
	vector<vector<Point3f>> obj_pts1(1);  //��ʱ�ȶ���һά��obj_pts_1����ȷ����img_pts��ά��֮���ٽ�������
	vector<vector<Point2f>> img_pts2;     // �������ͼ����ͼ���ƽ��ǵ�
	vector<vector<Point3f>> obj_pts2(1);  // �������ͼ���пռ�ǵ�

	// ---------- ��������ͼ
	std::printf("\nProcessing the left checkboard frames...\n");

	// ��ȡ�ǵ����������ϵ����
	addObjPts(board_size, (int)square_size, obj_pts1[0]);

	//��ͨ���ı����displayCorners1��ֵ��ȷ���Ƿ�չʾ��ȡ�ǵ���ͼ��
	bool show_corners1 = true;

	// ��������ͼ���е�ƽ��ǵ�
	for (int i = 0; i < left_img_paths.size(); ++i)
	{
		const string& img_path = left_img_paths[i];
		cv::Mat src_1 = imread(img_path, 1);
		if (src_1.empty())
		{
			std::cout << "[Err]: Read empty image " << img_path << endl;
			exit(-1);
		}

		if (rotate90cw)
		{
			cv::Mat src1;
			cv::rotate(src_1, src1, cv::ROTATE_90_CLOCKWISE);
			src_1 = src1;
		}

		img_size = src_1.size();

		vector<Point2f> corners1;
		bool found1 = cv::findChessboardCorners(src_1, board_size, corners1);

		if (found1)
		{
			if (corners1.size() != board_size.width * board_size.height)
			{
				printf("[Warning]: Found corners number not correct! %s\n", img_path.c_str());
			}

			Mat gray_img1;
			cv::cvtColor(src_1, gray_img1, cv::COLOR_BGR2GRAY);

			cv::cornerSubPix(gray_img1,
				corners1,
				Size(11, 11),
				Size(-1, -1),
				TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));

			// ��ŵ�ǰ����ͼƽ��ǵ�
			img_pts1.push_back(corners1);

			if (show_image & show_corners1)
			{
				sprintf(win_name_1, "������ǵ��ȡ���");
				Mat src_show_1 = src_1.clone();
				cv::drawChessboardCorners(src_show_1, board_size, Mat(corners1), found1);
				cv::imshow(win_name_1, src_show_1);
				cv::waitKey(500);
				cv::destroyWindow(win_name_1);
			}
		}
		else
		{
			printf("Found corners failed for %s\n", img_path.c_str());
			continue;
		}

		printf("[Left frame %d] found %d corners for %s.\n",
			i + 1, (int)corners1.size(), img_path.c_str());
	}

	// ��object points������������
	obj_pts1.resize(img_pts1.size(), obj_pts1[0]);  // ����(����ͼͼ�����)��

	// ---------- ��������ͼ
	std::printf("\nProcessing the right checkboard frames...\n");

	// ��ȡ�ǵ����������ϵ����
	addObjPts(board_size, (int)square_size, obj_pts2[0]);

	bool show_corners2 = true;
	for (int i = 0; i < right_img_paths.size(); i++)
	{
		const string& img_path = right_img_paths[i];
		cv::Mat src_2 = imread(img_path, 1);
		if (src_2.empty())
		{
			std::cout << "[Err]: Read empty image " << img_path << endl;
			exit(-1);
		}

		if (rotate90cw)
		{
			cv::Mat src2;
			cv::rotate(src_2, src2, cv::ROTATE_90_CLOCKWISE);
			src_2 = src2;
		}

		vector<Point2f> corners2;
		bool found_2 = cv::findChessboardCorners(src_2, board_size, corners2);

		if (found_2)
		{
			if (corners2.size() != board_size.width * board_size.height)
			{
				printf("[Warning]: Found corners number not correct! %s\n", img_path.c_str());
			}

			Mat gray_img_2;
			cv::cvtColor(src_2, gray_img_2, cv::COLOR_BGR2GRAY);

			cv::cornerSubPix(gray_img_2,
				corners2,
				Size(11, 11),
				Size(-1, -1),
				TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
			img_pts2.push_back(corners2);

			if (show_image & show_corners2)
			{
				sprintf(win_name_1, "������ǵ��ȡ���");

				Mat src_show_2 = src_2.clone();
				cv::drawChessboardCorners(src_show_2, board_size, Mat(corners2), found_2);
				cv::imshow(win_name_1, src_show_2);
				cv::waitKey(500);
				cv::destroyWindow(win_name_1);
			}
		}
		else
		{
			printf("Found corners failed for %s\n", img_path.c_str());
			continue;
		}

		printf("[Right frame %d] found %d corners for %s.\n",
			i + 1, (int)corners2.size(), img_path.c_str());
	}

	// ������ͼ�Ŀռ�ǵ�������������
	obj_pts2.resize(img_pts2.size(), obj_pts2[0]);

	/**************************************************************************************/
	/********************************���е�Ŀ����궨******************************************/
	//ͨ��calibrateCamera()������������궨
	//��ҪΪ�˵õ�������ڲξ���cameraMatrix������ϵ������distCoeffs
	//�������ͨ���������ص���ͶӰ����С��������궨�ľ������
	//����õ��������ξ�����Ҫ

	//Mat camera_matrix = Mat::eye(3, 3, CV_64F);
	//Mat distCoeffs = Mat::zeros(8, 1, CV_64F);  // ����ϵ����˳����[k1,k2,p1,p2,k3,(k4,k5,k6)]
	Mat K1 = Mat::eye(3, 3, CV_64F);       // ����ͼ����ڲξ���
	Mat dist1 = Mat::zeros(5, 1, CV_64F);  // ����ͼ����ϵ�� 
	Mat K2 = Mat::eye(3, 3, CV_64F);       // ����ͼ����ڲξ���
	Mat dist2 = Mat::zeros(5, 1, CV_64F);  // ����ͼ����ϵ��

	//vector<Mat> rvecs, tvecs;
	vector<Mat> r_vecs1, t_vecs1;  // ����ͼÿ����ͼ��λ��: ��ת�����ƽ������ 
	vector<Mat> r_vecs2, t_vecs2;  // ����ͼÿ����ͼ��λ��: ��ת�����ƽ������ 

	/*
	calibrateCamera()
	������� objectPoints  �ǵ��ʵ����������
			 imagePoints   �ǵ��ͼ������
			 imageSize     ͼ��Ĵ�С
	�������
			 cameraMatrix  ������ڲξ���
			 distCoeffs    ����Ļ������
			 rvecs         ��תʸ��(�����)
			 tvecs         ƽ��ʸ��(�������
	*/

	//����ϵ����˳����[k1,k2,p1,p2,k3,(k4,k5,k6)]
	//������ǲ���ҪK3���ڳ�ʼ��K3ΪO֮�󣬿���ʹ�ñ�־CV_CALIB_FIX_K3���������궨��������ı�K3��ֵ
	//һ��أ�K3Ӧ����Ϊ0������ʹ�����۾�ͷ���ο���learning opencv����ʮһ�£�
	//���ص�distCoeffs1�����ĳ����ɱ�־λflag��������flag����ΪCV_CALIB_RATIONAL_MODELʱ�������л��������8����
	//�����ó�����flagʱ������5ά�Ļ���ϵ������[k1, k2, p1, p2, k3] or [k0, k1, p0, p1, k2]

	// ----- ��֤�ǵ������Ƿ�һ��...

	std::printf("\nStart calibrate the left camera...\n");
	double re_proj_err1 = cv::calibrateCamera(obj_pts1,
		img_pts1,
		img_size,
		K1,
		dist1,
		r_vecs1,
		t_vecs1,
		cv::CALIB_FIX_K3);

	//checkRange()����  ---���ڼ������е�ÿһ��Ԫ�ص���Ч��
	bool ok1 = cv::checkRange(K1) && checkRange(dist1);
	if (ok1)
	{
		std::printf("\n");
		std::cout << "������궨�ɹ���" << endl;
		std::cout << "������궨����ͶӰ��" << re_proj_err1 << "pixel" << endl;
		std::cout << "������ڲξ���" << endl << K1 << endl;
		std::cout << "���������ϵ������" << endl << dist1 << endl;

		if (show_undistorted == true)
		{
			for (int i = 0; i < left_img_paths.size(); i++)
			{
				cv::Mat temp = cv::imread(left_img_paths[i], 1);
				if (rotate90cw)
				{
					cv::Mat tmp;
					cv::rotate(temp, tmp, cv::ROTATE_90_CLOCKWISE);
					temp = tmp;
				}

				//����undistort()�����õ��������������ͼ��
				Mat undistort_view;
				cv::undistort(temp, undistort_view, K1, dist1);

				if (show_image)
				{
					sprintf(win_name_1, "ԭ����ͼ��");
					sprintf(win_name_2, "�������ͼ��");
					cv::imshow(win_name_1, temp);
					cv::imshow(win_name_2, undistort_view);
					cv::waitKey(500);
					cv::destroyWindow(win_name_1);
					cv::destroyWindow(win_name_2);
				}
			}
		}
	}
	std::printf("Calibrate the left camera done.\n");

	std::printf("\nStart calibrate the right camera...\n");
	double re_proj_err2 = calibrateCamera(obj_pts2,
		img_pts2,
		img_size,
		K2,
		dist2,
		r_vecs2,
		t_vecs2,
		cv::CALIB_FIX_K3);

	bool ok2 = checkRange(K2) && checkRange(dist2);
	if (ok2)
	{
		std::printf("\n");
		std::cout << "������궨�ɹ���" << endl;
		std::cout << "������궨����ͶӰ��" << re_proj_err2 << "pixel" << endl;
		std::cout << "������ڲξ���" << endl << K2 << endl;
		std::cout << "���������ϵ������" << endl << dist2 << endl;

		if (show_undistorted == true)
		{
			for (int i = 0; i < right_img_paths.size(); i++)
			{
				cv::Mat temp = cv::imread(right_img_paths[i], 1);
				if (rotate90cw)
				{
					cv::Mat tmp;
					cv::rotate(temp, tmp, cv::ROTATE_90_CLOCKWISE);
					temp = tmp;
				}

				//����undistort()�����õ��������������ͼ��
				Mat undistort_view;
				cv::undistort(temp, undistort_view, K2, dist2);

				if (show_image)
				{
					sprintf(win_name_1, "ԭ����ͼ��");
					sprintf(win_name_2, "�������ͼ��");

					cv::imshow(win_name_1, temp);
					cv::imshow(win_name_2, undistort_view);
					cv::waitKey(500);
					cv::destroyWindow(win_name_1);
					cv::destroyWindow(win_name_2);
				}
			}
		}
	}
	std::printf("Calibrate the right camera done.\n");


	/**************************************************************************************/
	/********************************����궨(˫Ŀ�궨)******************************************/
	//����stereoCalibrate()������������궨���õ�4������R ��תʸ�� Tƽ��ʸ�� E�������� F��������
	//ͬʱ���Եõ�����������ڲξ���ͻ���ϵ������cameraMatrix1��distCoeffs1��cameraMatrix2��distCoeffs2
	//Ҳ����˵��ʵ���Բ������ȵ�����ÿ��������б궨��
	//stereoCalibrate()�����ڲ�Ӧ���ǵ�����calibrateCamera()������
	//����Ҳ�����ȵ���calibrateCamera()�����ȶ�ÿ��������б궨���õ��ڲξ���ͻ���ϵ���ĳ�ʼֵ
	//Ȼ������궨ʱstereoCalibrate()��������ڲξ���ͻ���ϵ�������Ż�����ʱӦ����flag��CALIB_USE_INTRINSIC_GUESS
	//���û�����ȵ��ù�calibrateCamera()�������벻Ҫʹ��flag:CALIB_USE_INTRINSIC_GUESS����õ�����ֵĽ��
	//���֮ǰ�궨��������ڲξ���ͻ�����������⣬����������궨ʱ����һ���Ż�����ʹ��CV_CALIB_FIX_INTRINSIC
	//���ݹٷ��ĵ����飬stereoCalibrate()��������Ĳ����ռ��ά���ܸߣ�һ���Եõ��ܶ�����
	//���ܻᵼ��ĳЩ�����ɢ���������ֵ��ƫ����ȷ����������ǰʹ����calibrateCamera()������ÿ��������й��궨
	//�����ѡ��CALIB_FIX_INTRINSICӦ�õ�stereoCalibrate()�����У������ܼ��ټ���Ĳ���
	//��ֹ����ĳЩ�����ɢ���������ֵ
	//CV_CALIB_FIX_INTRINSIC��������Ƿ�ʹ�û����������һ��Ȩ��

	std::printf("\nStart stereo calibrating...\n");

	//R: ��תʸ�� T: ƽ��ʸ�� E: �������� F: ��������
	Mat R = Mat::eye(3, 3, CV_64F);
	Mat T = Mat::zeros(3, 1, CV_64F);
	Mat E = Mat::zeros(3, 3, CV_64F);
	Mat F = Mat::eye(3, 3, CV_64F);  // ����double����

	double rms = 0.0;
	try
	{
		rms = cv::stereoCalibrate(obj_pts1,   // input
			img_pts1, img_pts2,				  // input
			K1, dist1,                        // input
			K2, dist2,                        // input
			img_size,                         // input
			R, T, E, F,                       // output
			CALIB_FIX_INTRINSIC,
			TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 100, 1e-5));
	}
	catch (const std::exception&)
	{
		cout << "Stereo calibration failed, exit the program now.\n";
		return -1;
	}

	std::cout << "Stereo Calibration done with RMS error: " << rms << "pixel\n\n";
	std::cout << "������ڲξ���" << endl << K1 << endl;
	std::cout << "���������ϵ������" << endl << dist1 << endl;
	std::cout << "������ڲξ���" << endl << K2 << endl;
	std::cout << "���������ϵ������" << endl << dist2 << endl;
	std::cout << "R:" << endl << R << endl;
	std::cout << "T:" << endl << T << endl;
	std::printf("Stereo calibration succeeded!.\n");

	/**************************************************************************************/
	/********************************����궨���******************************************/
	//����.xml�ļ�ʱ��Ҫע��2�����⣺
	//1 ��Ҫ�����Mat�ͱ�������ʱ����Ҫ��ʼ������������������
	//2 ����ʱ�����ı�ʶ�������в��ܳ��֡�.����
	FileStorage fs(res_xml_path, FileStorage::WRITE);

	/********************************�������********************************/
	std::printf("Start stereo rectifying...\n");

	Mat R1, R2, P1, P2, Q;
	cv::stereoRectify(
		K1,					   // input
		dist1,                 // input
		K2,					   // input
		dist2,				   // input
		img_size,              // input
		R, T,                  // input
		R1, R2, P1, P2, Q      // output
	);
	std::cout << "Q:" << endl << Q << endl;

	if (fs.isOpened())
	{
		fs << "width" << img_size.width;
		fs << "height" << img_size.height;
		fs << "board_width" << board_size.width;
		fs << "board_height" << board_size.height;
		fs << "nFrames" << nFrames;
		fs << "cameraMatrix1" << K1;
		fs << "distCoeffs1" << dist1;
		fs << "cameraMatrix2" << K2;
		fs << "distCoeffs2" << dist2;
		fs << "R" << R;
		fs << "T" << T;
		fs << "E" << E;
		fs << "F" << F;
		fs << "R1" << R1;
		fs << "R2" << R2;
		fs << "P1" << P1;
		fs << "P2" << P2;
		fs << "Q" << Q;

		fs.release();
		std::cout << "Calibration result has been saved successfully to "
			<< res_xml_path << endl;
	}
	else
	{
		std::cout << "Error: can not save the Calibrate&Rectify result!" << endl;
	}

	/********************************����У������ӳ���********************************/
	// ��ԭͼ���У����ͼ���ϵĵ�һһӳ�䡣
	Mat map_x1 = Mat(img_size, CV_32FC1);
	Mat map_y1 = Mat(img_size, CV_32FC1);
	Mat map_x2 = Mat(img_size, CV_32FC1);
	Mat map_y2 = Mat(img_size, CV_32FC1);

	cv::initUndistortRectifyMap(K1, dist1, R1, P1,
		img_size, CV_16SC2,
		map_x1, map_y1);
	cv::initUndistortRectifyMap(K2, dist2, R2, P2,
		img_size, CV_16SC2,
		map_x2, map_y2);

	// ��ȡԭʼͼ
	Mat left_img = cv::imread(left_img_paths[0], IMREAD_COLOR);
	Mat right_img = cv::imread(right_img_paths[0], IMREAD_COLOR);
	if (rotate90cw)
	{
		cv::Mat left_tmp, right_tmp;
		cv::rotate(left_img, left_tmp, cv::ROTATE_90_CLOCKWISE);
		left_img = left_tmp;

		cv::rotate(right_img, right_tmp, cv::ROTATE_90_CLOCKWISE);
		right_img = right_tmp;
	}

	// ����������������ͼ
	Mat left_img_rect, right_img_rect;
	if (!map_x1.empty() && !map_y1.empty())  // �ݽ��н�����ӳ��
	{
		cv::remap(left_img, left_img_rect,
			map_x1, map_y1, INTER_LINEAR);  // img_1 -> img_l
	}
	if (!map_x2.empty() && !map_y2.empty())
	{
		cv::remap(right_img, right_img_rect,
			map_x2, map_y2, INTER_LINEAR);  // img_2 -> img_r
	}
	if (show_image)  // ���ӻ�
	{
		cv::imshow("imgLr", left_img_rect);
		cv::imshow("imgRr", right_img_rect);
	}
	std::printf("Stereo rectifying done.\n");

	cv::imwrite("./imgLeft.png", left_img_rect);
	cv::imwrite("./imgRight.png", right_img_rect);  // ����ͼƬ
	std::printf("%s written.\n%s written.\n", "./imgLeft.png", "./imgRight.png");

	/********************************��ʾ����Ч��******************************************/
	// ����IMG���߶�һ�������˫��
	Mat img_origin(int(img_size.height * 0.5), img_size.width, CV_8UC3);   // ����ǰ����-����ͼ
	Mat img_rectify(int(img_size.height * 0.5), img_size.width, CV_8UC3);  // ���������-����ͼ

	// ǳ����
	Mat img_rectify_part_1 = img_rectify(Rect(0, 0, int(img_size.width * 0.5), int(img_size.height * 0.5)));
	Mat img_rectify_part_2 = img_rectify(Rect(int(img_size.width * 0.5), 0, int(img_size.width * 0.5), int(img_size.height * 0.5)));

	// ���
	cv::resize(left_img_rect, img_rectify_part_1, img_rectify_part_1.size(), 0, 0, INTER_AREA);
	cv::resize(right_img_rect, img_rectify_part_2, img_rectify_part_2.size(), 0, 0, INTER_AREA);  // �ı�ͼ��ߴ磬����0,0

	// ��-����ͼ������
	for (int i = 0; i < img_rectify.rows; i += 16)
	{
		cv::line(img_rectify, Point(0, i), Point(img_rectify.cols, i), Scalar(0, 255, 0), 1, 8);
	}
	std::printf("Draw lines for rectified left-right frame done.\n");

	// ǳ����
	Mat img_part_1 = img_origin(Rect(0, 0, int(img_size.width * 0.5), int(img_size.height * 0.5)));
	Mat img_part_2 = img_origin(Rect(int(img_size.width * 0.5), 0, int(img_size.width * 0.5), int(img_size.height * 0.5)));

	// ���
	cv::resize(left_img, img_part_1, img_part_1.size(), 0, 0, INTER_AREA);
	cv::resize(right_img, img_part_2, img_part_2.size(), 0, 0, INTER_AREA);  // �ı�ͼ��ߴ磬����0,0

	// ��-����ͼ������
	for (int i = 0; i < img_rectify.rows; i += 16)
	{
		cv::line(img_origin, Point(0, i), Point(img_origin.cols, i), Scalar(0, 255, 0), 1, 8);
	}
	std::printf("Draw lines for un-rectified left-right frame done.\n");

	if (show_image)
	{
		// ���ӻ�
		cv::imshow("unrectified", img_origin);
		cv::imshow("rectified", img_rectify);
	}

	// ������ӻ����
	cv::imwrite("./unretified.png", img_origin);
	cv::imwrite("./retified.png", img_rectify);
	std::printf("%s written.\n%s written.\n", "./un-retified.png", "./retified.png");
	std::printf("Stereo calibration done.\n");
	std::cout << "��������˳�����..." << endl;

	if (show_image)
	{
		cv::waitKey();
	}

	return 0;
}


int showStereoAlignment(const string& root, const Size& img_size, const string& ext = ".png")
{
	// get directories of the paird imgs
	vector<string> dir_paths;
	getDirs(root, dir_paths);

	// get img paths of the paird imgs
	vector<string> left_img_paths, right_img_paths;
	getFilesFormat(dir_paths[0], ext, left_img_paths);
	getFilesFormat(dir_paths[1], ext, right_img_paths);
	assert(left_img_paths.size() == right_img_paths.size());

	if (left_img_paths.size() == 0)
	{
		std::cout << "[Err]: empty image dir.\n";
		return -1;
	}

	const string left_img_dir_name("image_02");
	const string right_img_dir_name("image_03");

	string right_img_path;
	int cnt = 0;
	for (const auto& left_img_path : left_img_paths)
	{
		replaceStr(left_img_path, "image_02", "image_03", right_img_path, -1);

		Mat left_img = cv::imread(left_img_path, cv::IMREAD_COLOR);
		Mat right_img = cv::imread(right_img_path, cv::IMREAD_COLOR);
		if (left_img.empty() || right_img.empty())
		{
			std::cout << "[Err]: Left or right image is empty!\n";
			exit(-1);
		}

		// ����IMG���߶�һ�������˫��
		Mat img_pair(int(img_size.height * 0.5), img_size.width, CV_8UC3);  // ���������-����ͼ

		// ǳ����
		Mat img_pair_part_1 = img_pair(Rect(0, 0, int(img_size.width * 0.5), int(img_size.height * 0.5)));
		Mat img_pair_part_2 = img_pair(Rect(int(img_size.width * 0.5), 0, int(img_size.width * 0.5), int(img_size.height * 0.5)));

		// resize���
		cv::resize(left_img, img_pair_part_1, img_pair_part_1.size(), 0, 0, INTER_AREA);
		cv::resize(right_img, img_pair_part_2, img_pair_part_2.size(), 0, 0, INTER_AREA);  // �ı�ͼ��ߴ磬����0,0

		// ��-����ͼ������
		for (int i = 0; i < img_pair.rows; i += 16)
		{
			cv::line(img_pair, Point(0, i), Point(img_pair.cols, i), Scalar(0, 255, 0), 1, 8);
		}
		std::printf("Draw lines for rectified left-right frame done for %d.\n", cnt);

		// ���ӻ�
		cv::imshow("Image-Pair", img_pair);
		cv::waitKey();

		char save_path[50];
		sprintf(save_path, "e:/tmp/%05d_kitti.jpg", cnt);
		cv::imwrite(save_path, img_pair);

		cnt += 1;
	}

	return 0;
}



int readParamsFromXmlAndRectify_old()
{
	/*
	Read parameters from xml file
	*/
	// ----- Read left, right intrinsic matrix
	const string l_K_xml_path("./xmls/IntrinsicMatrix_left.xml");
	const string r_K_xml_path("./xmls/IntrinsicMatrix_right.xml");

	vector<double> l_K, r_K;
	string elem_name("IntrinsicMatrix_left");
	readParamsFromXml(l_K_xml_path, elem_name, l_K);

	elem_name = "IntrinsicMatrix_right";
	readParamsFromXml(r_K_xml_path, elem_name, r_K);

	{
		l_K = {
			1150.96520619,  0.00000000,  566.82739051,
			0.00000000,  1148.23163638,  459.93509031,
			0.00000000,  0.00000000,  1.00000000
		};

		r_K = {
			1139.30950514,  0.00000000,  648.95382247,
			0.00000000,  1135.66052819,  485.02322134,
			0.00000000,  0.00000000,  1.00000000
		};
	}

	Mat l_K_mat(l_K, true);
	Mat r_K_mat(r_K, true);
	l_K_mat = l_K_mat.reshape(0, 3);
	r_K_mat = r_K_mat.reshape(0, 3);
	std::cout << "Left intrinsic matrix:\n" << l_K_mat << endl;
	std::cout << "Right intrinsic matrix:\n" << r_K_mat << endl;

	// ----- Read left and right rotation matrix
	const string l_rotate_xml_path("./xmls/Rl.xml");
	const string r_rotate_xml_path("./xmls/Rr.xml");

	vector<double> l_rotate, r_rotate;
	elem_name = "Rl";
	int ret = readParamsFromXml(l_rotate_xml_path, elem_name, l_rotate);
	if (ret < 0)
	{
		l_rotate = {
			0.99994399,  0.00566509, -0.00894005,
			-0.00576325, 0.99992296, -0.01099325,
			0.00887708,  0.01104416,  0.99989961
		};
	}

	elem_name = "Rr";
	ret = readParamsFromXml(r_rotate_xml_path, elem_name, r_rotate);
	if (ret < 0)
	{
		r_rotate = {
			0.99985362,  0.01028608,  0.01367277,
			-0.01043638,  0.99988540,  0.01096683,
			-0.01355840, -0.01110792,  0.99984638
		};
	}

	Mat l_R_mat(l_rotate, true);
	Mat r_R_mat(r_rotate, true);
	l_R_mat = l_R_mat.reshape(0, 3);
	r_R_mat = r_R_mat.reshape(0, 3);
	std::cout << "Left rotation Mat:\n" << l_R_mat << endl;
	std::cout << "Right rotation Mat:\n" << r_R_mat << endl;

	// ----- Read left and right 3x4 projection matrix
	const string l_P_xml_path("./xmls/Pl.xml");
	const string r_P_xml_path("./xmls/Pr.xml");

	vector<double> l_P, r_P;
	elem_name = "Pl";
	ret = readParamsFromXml(l_P_xml_path, elem_name, l_P);
	if (ret < 0)
	{
		l_P = {
		1141.94608228,  0.00000000,  606.10649043,  0.00000000,
		0.00000000,  1141.94608228,  470.76888035,  0.00000000,
		0.00000000,  0.00000000,  1.00000000,  0.00000000
		};
	}

	elem_name = "Pr";
	ret = readParamsFromXml(r_P_xml_path, elem_name, r_P);
	if (ret < 0)
	{
		r_P = {
			1141.94608228,  0.00000000,  606.10649043, -180780.77651645,
			0.00000000,  1141.94608228,  470.76888035,  0.00000000,
			0.00000000,  0.00000000,  1.00000000,  0.00000000
		};
	}

	Mat l_P_mat(l_P, true);
	Mat r_P_mat(r_P, true);
	l_P_mat = l_P_mat.reshape(0, 3);
	r_P_mat = r_P_mat.reshape(0, 3);
	std::cout << "Left projection Mat:\n" << l_P_mat << endl;
	std::cout << "Right projection Mat:\n" << r_P_mat << endl;

	// ----- Read left and right camera distortion coefficients
	const string l_dist_xml_path("./xmls/distCoeffL.xml");
	const string r_dist_xml_path("./xmls/distCoeffR.xml");

	vector<double> l_dists, r_dists;
	elem_name = "distCoeffL";
	ret = readParamsFromXml(l_dist_xml_path, elem_name, l_dists);
	if (ret < 0)
	{
		l_dists = {
			-0.38198221,  0.13206702, -0.02789491,  0.01244918,  0.00000000
		};
	}

	elem_name = "distCoeffR";
	ret = readParamsFromXml(r_dist_xml_path, elem_name, r_dists);
	if (ret < 0)
	{
		r_dists = {
			-0.38915232,  0.12921631, -0.02736358,  0.00185069,  0.00000000
		};
	}

	Mat l_dist_mat(l_dists, true);
	Mat r_dist_mat(r_dists, true);
	l_dist_mat = l_dist_mat.reshape(0, 5);
	r_dist_mat = r_dist_mat.reshape(0, 5);
	std::cout << "Left distortion coefficients:\n" << l_dist_mat << endl;
	std::cout << "Right distortion coefficients:\n" << r_dist_mat << endl;

	// Read Rotation vector and Translation vector
	const string r_xml_path("./xmls/Rotation.xml");
	const string t_xml_path("./xml/Translation.xml");

	vector<double> r_vect, t_vect;

	elem_name = "Rotation";
	ret = readParamsFromXml(r_xml_path, elem_name, r_vect);  // ��ȡ��ת����
	if (ret < 0)
	{
		r_vect = { 0.02214207, -0.02243658,  0.00467347 };
	}

	elem_name = "Translation";
	ret = readParamsFromXml(t_xml_path, elem_name, t_vect);
	if (ret < 0)
	{
		t_vect = { -158.29175859, -1.62844041, -2.16460339 };
	}

	Mat r_vect_mat(r_vect, true);
	Mat t_mat(t_vect, true);

	// �޵����˹�任: ��ת���� ����> ��ת���� 
	Mat r_mat;
	cv::Rodrigues(r_vect_mat, r_mat);

	std::cout << "Rotation matrix:\n" << r_mat << endl;
	std::cout << "Translation vector:\n" << t_mat << endl;

	// ������ͶӰ����Q: ��ȡ˫ĿУ������ڲξ���K
	const Size img_size = Size(1280, 720);
	Mat R1, R2, P1, P2, Q;  // �����֤
	try
	{
		cv::stereoRectify(
			l_K_mat,
			l_dist_mat,
			r_K_mat,
			r_dist_mat,
			img_size,
			r_mat,
			t_mat,
			R1, R2, P1, P2, Q
		);
	}
	catch (const std::exception&)
	{
		std::cout << "Rectifying failed!\n";
	}
	std::cout << "Q matrix:" << endl << Q << endl;

	const string q_f_path = "./xmls/Q.xml";
	FileStorage fs_q(q_f_path, FileStorage::WRITE);
	if (fs_q.isOpened())
	{
		fs_q << "Q" << Q;
		fs_q.release();
		std::cout << q_f_path + " saved.\n";
	}
	else
	{
		std::cout << "Error: can not save the Q matrix!" << endl;
	}

	std::cout << "R1 diff:\n" << R1 - l_R_mat << endl;
	std::cout << "R2 diff:\n" << R2 - r_R_mat << endl;
	std::cout << "P1 diff:\n" << P1 - l_P_mat << endl;
	std::cout << "P2 diff:\n" << P2 - r_P_mat << endl;

	// ʹ��ʵ�ʼ���õ��Ľ�������
	l_R_mat = R1;
	r_R_mat = R2;
	l_P_mat = P1;
	r_P_mat = P2;

	std::cout << "R1 diff:\n" << R1 - l_R_mat << endl;
	std::cout << "R2 diff:\n" << R2 - r_R_mat << endl;
	std::cout << "P1 diff:\n" << P1 - l_P_mat << endl;
	std::cout << "P2 diff:\n" << P2 - r_P_mat << endl;

	std::cout << "Rectifying parameters read done.\n";

	/*
	Read image pairs, do rectifying and save
	*/
	const string img_root("./rig_checkboard_1");
	const string img_rectified_root("E:/LeftRightFramesRectified");
	const string left_rectified_dir = img_rectified_root + "/image_02";
	const string right_rectified_dir = img_rectified_root + "/image_03";

	// get directories of the paird imgs
	vector<string> dir_paths;
	getDirs(img_root, dir_paths);

	// get img paths of the paird imgs
	vector<string> left_img_paths, right_img_paths;
	getFilesFormat(dir_paths[0], ".jpg", left_img_paths);
	getFilesFormat(dir_paths[1], ".jpg", right_img_paths);
	assert(left_img_paths.size() == right_img_paths.size());

	const string left_img_dir_name("left");
	const string right_img_dir_name("right");

	// traverse each of the paird-img
	string right_img_path;
	vector<string> tokens;
	for (int i = 0; i < left_img_paths.size(); ++i)
	{
		// get image name by splitting str
		const string& left_img_path = left_img_paths[i];
		const string& right_img_path = right_img_paths[i];

		splitStr(left_img_path, tokens, '/');
		const auto& img_name = tokens[tokens.size() - 1];

		// get corresponding right image path
		//replaceStr(left_img_path, left_img_dir_name, right_img_dir_name, right_img_path, -1);

		// read original image
		Mat left_img = cv::imread(left_img_path, cv::IMREAD_COLOR);
		cout << left_img_path << " read in.\n";
		Mat right_img = cv::imread(right_img_path, cv::IMREAD_COLOR);
		cout << right_img_path << " read in.\n";

		if (left_img.empty() || right_img.empty())
		{
			cout << "Left or right image is empty.\n";
			continue;
		}

		const Size& img_size = left_img.size();

		/*********����У������ӳ���*********/
		// ��ԭͼ���У����ͼ���ϵĵ�һһӳ�䡣
		Mat remapm_x_1 = Mat(img_size, CV_32FC1);
		Mat remapm_y_1 = Mat(img_size, CV_32FC1);
		Mat remapm_x_2 = Mat(img_size, CV_32FC1);
		Mat remapm_y_2 = Mat(img_size, CV_32FC1);

		cv::initUndistortRectifyMap(l_K_mat, l_dist_mat, R1, P1,
			img_size, CV_16SC2, remapm_x_1, remapm_y_1);
		cv::initUndistortRectifyMap(r_K_mat, r_dist_mat, R2, P2,
			img_size, CV_16SC2, remapm_x_2, remapm_y_2);

		// ����������������ͼ
		Mat left_img_rectified, right_img_rectified;
		if (!remapm_x_1.empty() && !remapm_y_1.empty())  // �ݽ��н�����ӳ��
		{
			cv::remap(left_img, left_img_rectified,
				remapm_x_1, remapm_y_1, INTER_LINEAR);  // left_img -> left_img_rectified
		}
		if (!remapm_x_2.empty() && !remapm_y_2.empty())
		{
			cv::remap(right_img, right_img_rectified,
				remapm_x_2, remapm_y_2, INTER_LINEAR);  // right_img -> right_img_rectified
		}
		//if (SHOW)
		//{
		//	cv::imshow("imgLr", img_l);
		//	cv::imshow("imgRr", img_r);
		//}
		//printf("\nStereo rectifying done.\n");

		/********************************��ʾ����Ч��******************************************/
		if (SHOW)
		{
			// ����IMG���߶�һ�������˫��
			Mat img_origin(int(img_size.height * 0.5), img_size.width, CV_8UC3);   // ����ǰ����-����ͼ
			Mat img_rectify(int(img_size.height * 0.5), img_size.width, CV_8UC3);  // ���������-����ͼ

			// ǳ����
			Mat img_rectify_part_1 = img_rectify(Rect(0, 0, int(img_size.width * 0.5), int(img_size.height * 0.5)));
			Mat img_rectify_part_2 = img_rectify(Rect(int(img_size.width * 0.5), 0, int(img_size.width * 0.5), int(img_size.height * 0.5)));

			// resize���
			cv::resize(left_img_rectified, img_rectify_part_1, img_rectify_part_1.size(), 0, 0, INTER_AREA);
			cv::resize(right_img_rectified, img_rectify_part_2, img_rectify_part_2.size(), 0, 0, INTER_AREA);  // �ı�ͼ��ߴ磬����0,0

			// ��-����ͼ������
			for (int i = 0; i < img_rectify.rows; i += 16)
			{
				cv::line(img_rectify, Point(0, i), Point(img_rectify.cols, i), Scalar(0, 255, 0), 1, 8);
			}
			std::printf("Draw lines for rectified left-right frame done.\n");

			// ǳ����
			Mat img_part_1 = img_origin(Rect(0, 0, int(img_size.width * 0.5), int(img_size.height * 0.5)));
			Mat img_part_2 = img_origin(Rect(int(img_size.width * 0.5), 0, int(img_size.width * 0.5), int(img_size.height * 0.5)));

			// resize���
			cv::resize(left_img, img_part_1, img_part_1.size(), 0, 0, INTER_AREA);
			cv::resize(right_img, img_part_2, img_part_2.size(), 0, 0, INTER_AREA);  // �ı�ͼ��ߴ磬����0,0

			// ��-����ͼ������
			for (int i = 0; i < img_rectify.rows; i += 16)
			{
				cv::line(img_origin, Point(0, i), Point(img_origin.cols, i), Scalar(0, 255, 0), 1, 8);
			}
			std::printf("Draw lines for un-rectified left-right frame done.\n");

			// ���ӻ�
			cv::imshow("unrectified", img_origin);
			cv::imshow("rectified", img_rectify);
			cv::waitKey();
		}

		// �������֮���left-rightͼ���
		const string left_rectified_path = left_rectified_dir + '/' + img_name;
		const string right_rectified_path = right_rectified_dir + '/' + img_name;

		cv::imwrite(left_rectified_path, left_img_rectified);
		cv::imwrite(right_rectified_path, right_img_rectified);

		if (i % 1000 == 0)
		{
			cout << left_rectified_path + " saved.\n";
			cout << right_rectified_path + " saved.\n";
		}

		//// update frame counting
		//cnt += 1;
	}

	return 0;
}

int readParamsFromXmlAndRectify()
{
	/*
	Read parameters from xml file
	*/
	// ----- Read left, right intrinsic matrix
	const string params_xml_path("./xmls/CalibrateRectify_CalibStereoMeasure.xml");
	Mat K1, K2, dist1, dist2, R, T, R1, R2, P1, P2, Q;
	readStereoCalibFromXml(params_xml_path, K1, dist1, K2, dist2, R, T, R1, R2, P1, P2, Q);

	// ������ͶӰ����Q: ��ȡ˫ĿУ������ڲξ���K
	Size img_size(1280, 720);
	Mat R1_, R2_, P1_, P2_, Q_;  // �����֤
	try
	{
		cv::stereoRectify(
			K1,
			dist1,
			K2,
			dist2,
			img_size,
			R,
			T,
			R1_, R2_, P1_, P2_, Q_  // output
		);
	}
	catch (const std::exception&)
	{
		std::cout << "Rectifying failed!\n";
	}
	std::cout << "Q matrix:" << endl << Q_ << endl;

	cout << "R1 diff:\n" << R1 - R1_ << endl;
	cout << "R2 diff:\n" << R2 - R2_ << endl;
	cout << "P1 diff:\n" << P1 - P1_ << endl;
	cout << "P2 diff:\n" << P2 - P2_ << endl;
	cout << "Q diff:\n" << Q - Q_ << endl;

	std::cout << "Rectifying parameters read done.\n";

	/*
	Read image pairs, do rectifying and save
	*/
	const string img_root("F:/PyScripts/LeftRightFrames");  // "F:/PyScripts/LeftRightFrames"
	const string img_rectified_root("E:/LeftRightFramesRectified");
	const string left_rectified_dir = img_rectified_root + "/image_02";
	const string right_rectified_dir = img_rectified_root + "/image_03";

	// get directories of the paird imgs
	vector<string> dir_paths;
	getDirs(img_root, dir_paths);

	// get img paths of the paird imgs
	vector<string> left_img_paths, right_img_paths;
	getFilesFormat(dir_paths[0], ".jpg", left_img_paths);
	getFilesFormat(dir_paths[1], ".jpg", right_img_paths);
	assert(left_img_paths.size() == right_img_paths.size());

	const string left_img_dir_name("image_02");
	const string right_img_dir_name("image_03");

	// traverse each of the paird-img
	string right_img_path;
	vector<string> tokens;
	//int cnt = 0;
	for (int i = 0; i < left_img_paths.size(); ++i)
	{
		// get image name by splitting str
		const string& left_img_path = left_img_paths[i];
		const string& right_img_path = right_img_paths[i];

		splitStr(left_img_path, tokens, '/');
		const auto& img_name = tokens[tokens.size() - 1];

		// get corresponding right image path
		//replaceStr(left_img_path, left_img_dir_name, right_img_dir_name, right_img_path, -1);

		// read original image
		Mat left_img = cv::imread(left_img_path, cv::IMREAD_COLOR);
		cout << left_img_path << " read in.\n";
		Mat right_img = cv::imread(right_img_path, cv::IMREAD_COLOR);
		cout << right_img_path << " read in.\n";
		if (left_img.empty() || right_img.empty())
		{
			cout << "Left or right image is empty.\n";
			continue;
		}

		const Size& img_size = left_img.size();

		/*********����У������ӳ���*********/
		// ��ԭͼ���У����ͼ���ϵĵ�һһӳ�䡣
		Mat remapm_x_1 = Mat(img_size, CV_32FC1);
		Mat remapm_y_1 = Mat(img_size, CV_32FC1);
		Mat remapm_x_2 = Mat(img_size, CV_32FC1);
		Mat remapm_y_2 = Mat(img_size, CV_32FC1);

		cv::initUndistortRectifyMap(K1, dist1, R1, P1,
			img_size, CV_16SC2, remapm_x_1, remapm_y_1);
		cv::initUndistortRectifyMap(K2, dist2, R2, P2,
			img_size, CV_16SC2, remapm_x_2, remapm_y_2);

		// ����������������ͼ
		Mat left_img_rectified, right_img_rectified;
		if (!remapm_x_1.empty() && !remapm_y_1.empty())  // �ݽ��н�����ӳ��
		{
			cv::remap(left_img, left_img_rectified,
				remapm_x_1, remapm_y_1, INTER_LINEAR);  // left_img -> left_img_rectified
		}
		if (!remapm_x_2.empty() && !remapm_y_2.empty())
		{
			cv::remap(right_img, right_img_rectified,
				remapm_x_2, remapm_y_2, INTER_LINEAR);  // right_img -> right_img_rectified
		}
		//if (SHOW)
		//{
		//	cv::imshow("imgLr", img_l);
		//	cv::imshow("imgRr", img_r);
		//}
		//printf("\nStereo rectifying done.\n");

		/********************************��ʾ����Ч��******************************************/
		if (SHOW)
		{
			// ����IMG���߶�һ�������˫��
			Mat img_origin(int(img_size.height * 0.5), img_size.width, CV_8UC3);   // ����ǰ����-����ͼ
			Mat img_rectify(int(img_size.height * 0.5), img_size.width, CV_8UC3);  // ���������-����ͼ

			// ǳ����
			Mat img_rectify_part_1 = img_rectify(Rect(0, 0, int(img_size.width * 0.5), int(img_size.height * 0.5)));
			Mat img_rectify_part_2 = img_rectify(Rect(int(img_size.width * 0.5), 0, int(img_size.width * 0.5), int(img_size.height * 0.5)));

			// resize���
			cv::resize(left_img_rectified, img_rectify_part_1, img_rectify_part_1.size(), 0, 0, INTER_AREA);
			cv::resize(right_img_rectified, img_rectify_part_2, img_rectify_part_2.size(), 0, 0, INTER_AREA);  // �ı�ͼ��ߴ磬����0,0

			// ��-����ͼ������
			for (int i = 0; i < img_rectify.rows; i += 16)
			{
				cv::line(img_rectify, Point(0, i), Point(img_rectify.cols, i), Scalar(0, 255, 0), 1, 8);
			}
			std::printf("Draw lines for rectified left-right frame done.\n");

			// ǳ����
			Mat img_part_1 = img_origin(Rect(0, 0, int(img_size.width * 0.5), int(img_size.height * 0.5)));
			Mat img_part_2 = img_origin(Rect(int(img_size.width * 0.5), 0, int(img_size.width * 0.5), int(img_size.height * 0.5)));

			// resize���
			cv::resize(left_img, img_part_1, img_part_1.size(), 0, 0, INTER_AREA);
			cv::resize(right_img, img_part_2, img_part_2.size(), 0, 0, INTER_AREA);  // �ı�ͼ��ߴ磬����0,0

			// ��-����ͼ������
			for (int i = 0; i < img_rectify.rows; i += 16)
			{
				cv::line(img_origin, Point(0, i), Point(img_origin.cols, i), Scalar(0, 255, 0), 1, 8);
			}
			std::printf("Draw lines for un-rectified left-right frame done.\n");

			// ���ӻ�
			cv::imshow("unrectified", img_origin);
			cv::imshow("rectified", img_rectify);
			cv::waitKey();
		}

		// �������֮���left-rightͼ���
		const string left_rectified_path = left_rectified_dir + '/' + img_name;
		const string right_rectified_path = right_rectified_dir + '/' + img_name;

		cv::imwrite(left_rectified_path, left_img_rectified);
		cv::imwrite(right_rectified_path, right_img_rectified);

		if (i % 1000 == 0)
		{
			cout << left_rectified_path + " saved.\n";
			cout << right_rectified_path + " saved.\n";
		}

		//// update frame counting
		//cnt += 1;
	}

	return 0;
}


/*����궨����ģ���ʵ����������*/
void addObjPts(const Size& board_size, const int square_size, vector<Point3f>& obj_pts)
{// (points_per_row, points_per_col): (cols, rows)
	for (int y = 0; y < board_size.height; ++y)
	{
		for (int x = 0; x < board_size.width; ++x)
		{
			obj_pts.push_back(Point3f((float)(x * square_size), (float)(y * square_size), 0.0f));
		}
	}
}


// the main function
int main(int argc, char* argv[])
{
	//if (argc < 5)
	//{
	//	cout << "Please specify:\n"
	//		<< "(1). checkboard's dir path | dir_path: left/right\n"
	//		<< "(2). result_path: ***.xml\n"
	//		<< "(3). n_corners_x\n"
	//		<< "(4). n_corners_y\n"
	//		<< endl;
	//	system("pause");
	//	return -1;
	//}

	// argv[1], argv[2], atoi(argv[3]), atoi(argv[4])
	runCalibrateAndRectify();

	//readParamsFromXmlAndRectify_old();

	//readParamsFromXmlAndRectify();

	/********* Image pair alignment testing **********/
	//Size img_size = Size(1280, 720);
	//showStereoAlignment("e:/xiaomi_outdoor", img_size, ".png");  // xiaomi
	//////showStereoAlignment("f:/PyScripts/LeftRightFrames", img_size, ".jpg");
	//showStereoAlignment("e:/LeftRightFramesRectified", img_size, ".jpg");  // stereo rig

	////img_size = Size(3130, 960);  // apollo
	////showStereoAlignment("e:/stereo_test", img_size, ".jpg");

	//img_size = Size(1226, 370);
	//showStereoAlignment("e:/kitti_test", img_size, ".jpg");  // kitti
}
