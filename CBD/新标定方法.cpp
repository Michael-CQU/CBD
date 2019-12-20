//#include <iostream>
//#include <vector>
//#include <fstream>
//#include <string>
//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//
//using namespace std;
//using namespace cv;
//
//int main()
//{
//	ifstream inImgPath("filename.txt");    //�궨����ͼ���ļ���·��
//	vector<string> imgList;
//	vector<string>::iterator p;
//	string temp;
//	if (!inImgPath.is_open())
//	{
//		cout << "û���ҵ��ļ�" << endl;
//	}
//	//��ȡ�ļ��б����ͼƬ�ļ�·�����������������
//	while (getline(inImgPath, temp))
//	{
//		imgList.push_back(temp);
//	}
//
//	ofstream fout("caliberation_result.txt");   //����궨������ļ�
//
//	cout << "��ʼ��ȡ�ǵ�......" << endl;
//	cv::Size image_size;//����ͼƬ��С
//	cv::Size pattern_size = cv::Size(6, 9);//�궨����ÿ�С�ÿ�еĽǵ���������ͼƬ�еı궨�����ڽǵ���Ϊ4*6
//	vector<cv::Point2f> corner_points_buf;//��һ�����黺���⵽�Ľǵ㣬ͨ������Point2f��ʽ
//	vector<cv::Point2f>::iterator corner_points_buf_ptr;
//	vector<vector<cv::Point2f>> corner_points_of_all_imgs;
//	int image_num = 0;
//	string filename;
//	while (image_num < imgList.size())
//	{
//		filename = imgList[image_num++];
//		cout << "image_num = " << image_num << endl;
//		cout << filename.c_str() << endl;
//		cv::Mat imageInput = cv::imread(filename.c_str());
//		if (image_num == 1)
//		{
//			image_size.width = imageInput.cols;
//			image_size.height = imageInput.rows;
//			cout << "image_size.width = " << image_size.width << endl;
//			cout << "image_size.height = " << image_size.height << endl;
//		}
//
//		if (findChessboardCorners(imageInput, pattern_size, corner_points_buf) == 0)
//		{
//			cout << "can not find chessboard corners!\n";   //�Ҳ����ǵ�
//			exit(1);
//		}
//		else
//		{
//			cv::Mat gray;
//			cv::cvtColor(imageInput, gray, CV_RGB2GRAY);
//			cv::find4QuadCornerSubpix(gray, corner_points_buf, cv::Size(5, 5));
//			corner_points_of_all_imgs.push_back(corner_points_buf);
//			cv::drawChessboardCorners(gray, pattern_size, corner_points_buf, true);
//			cv::imshow("camera calibration", gray);
//			cv::waitKey(100);
//		}
//	}
//
//	int total = corner_points_of_all_imgs.size();
//	cout << "total=" << total << endl;
//	int cornerNum = pattern_size.width * pattern_size.height;//ÿ��ͼƬ�ϵ��ܵĽǵ���
//	for (int i = 0; i < total; i++)
//	{
//		cout << "--> ��" << i + 1 << "��ͼƬ������ -->:" << endl;
//		for (int j = 0; j < cornerNum; j++)
//		{
//			cout << "-->" << corner_points_of_all_imgs[i][j].x;
//			cout << "-->" << corner_points_of_all_imgs[i][j].y;
//			if ((j + 1) % 3 == 0)
//			{
//				cout << endl;
//			}
//			else
//			{
//				cout.width(10);
//			}
//		}
//		cout << endl;
//	}
//
//	cout << endl << "�ǵ���ȡ���" << endl;
//
//	//������궨
//	cout << "��ʼ�궨������������" << endl;
//	cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));//����ξ���H������Ӧ�Ծ���
//	cv::Mat distCoefficients = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));//�������5������ϵ����k1,k2,p1,p2,k3
//	vector<cv::Mat> tvecsMat;//ÿ��ͼ���ƽ��������t
//	vector<cv::Mat> rvecsMat;//ÿ��ͼ�����ת�������޵������ת������
//	vector<vector<cv::Point3f>> objectPoints;//��������ͼƬ�Ľǵ����ά����
//											 //��ʼ��ÿһ��ͼƬ�б궨���Ͻǵ����ά����
//	int i, j, k;
//	for (k = 0; k < image_num; k++)//����ÿһ��ͼƬ
//	{
//		vector<cv::Point3f> tempCornerPoints;//ÿһ��ͼƬ��Ӧ�Ľǵ�����
//		//�������еĽǵ�
//		for (i = 0; i < pattern_size.height; i++)
//		{
//			for (j = 0; j < pattern_size.width; j++)
//			{
//				cv::Point3f singleRealPoint;//һ���ǵ������
//				singleRealPoint.x = i * 10;
//				singleRealPoint.y = j * 10;
//				singleRealPoint.z = 0;//����z=0
//				tempCornerPoints.push_back(singleRealPoint);
//			}
//		}
//		objectPoints.push_back(tempCornerPoints);
//	}
//
//	cv::calibrateCamera(objectPoints, corner_points_of_all_imgs, image_size, cameraMatrix, distCoefficients, rvecsMat, tvecsMat, 0);
//	cout << "�궨���" << endl;
//
//	//��ʼ����궨���
//	cout << "��ʼ����궨���" << endl;
//
//	cout << endl << "�����ز�����" << endl;
//	fout << "�����ز�����" << endl;
//	cout << "1.�����������:" << endl;
//	fout << "1.�����������:" << endl;
//	cout << "��С��" << cameraMatrix.size() << endl;
//	fout << "��С��" << cameraMatrix.size() << endl;
//	cout << cameraMatrix << endl;
//	fout << cameraMatrix << endl;
//
//	cout << "2.����ϵ����" << endl;
//	fout << "2.����ϵ����" << endl;
//	cout << "��С��" << distCoefficients.size() << endl;
//	fout << "��С��" << distCoefficients.size() << endl;
//	cout << distCoefficients << endl;
//	fout << distCoefficients << endl;
//
//	cout << endl << "ͼ����ز�����" << endl;
//	fout << endl << "ͼ����ز�����" << endl;
//	cv::Mat rotation_Matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));//��ת����
//	for (i = 0; i < image_num; i++)
//	{
//		cout << "��" << i + 1 << "��ͼ�����ת������" << endl;
//		fout << "��" << i + 1 << "��ͼ�����ת������" << endl;
//		cout << rvecsMat[i] << endl;
//		fout << rvecsMat[i] << endl;
//		cout << "��" << i + 1 << "��ͼ�����ת����" << endl;
//		fout << "��" << i + 1 << "��ͼ�����ת����" << endl;
//		cv::Rodrigues(rvecsMat[i], rotation_Matrix);//����ת����ת��Ϊ���Ӧ����ת����
//		cout << rotation_Matrix << endl;
//		fout << rotation_Matrix << endl;
//		cout << "��" << i + 1 << "��ͼ���ƽ��������" << endl;
//		fout << "��" << i + 1 << "��ͼ���ƽ��������" << endl;
//		cout << tvecsMat[i] << endl;
//		fout << tvecsMat[i] << endl;
//	}
//
//	cout << "����������" << endl;
//
//	//�Ա궨�����������
//	cout << "��ʼ���۱궨���......" << endl;
//
//	//����ÿ��ͼ���еĽǵ�����������ȫ���ǵ㶼��⵽��
//	int corner_points_counts;
//	corner_points_counts = pattern_size.width * pattern_size.height;
//
//	cout << "ÿ��ͼ��ı궨��" << endl;
//	fout << "ÿ��ͼ��ı궨��" << endl;
//	double err = 0;//����ͼ������
//	double total_err = 0;//����ͼ���ƽ�����
//	for (i = 0; i < image_num; i++)
//	{
//		vector<cv::Point2f> image_points_calculated;//����¼������ͶӰ�������
//		vector<cv::Point3f> tempPointSet = objectPoints[i];
//		cv::projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoefficients, image_points_calculated);
//
//		//�����µ�ͶӰ����ɵ�ͶӰ��֮������
//		vector<cv::Point2f> image_points_old = corner_points_of_all_imgs[i];
//		//���������ݻ���Mat��ʽ
//		cv::Mat image_points_calculated_mat = cv::Mat(1, image_points_calculated.size(), CV_32FC2);
//		cv::Mat image_points_old_mat = cv::Mat(1, image_points_old.size(), CV_32FC2);
//		for (j = 0; j < tempPointSet.size(); j++)
//		{
//			image_points_calculated_mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points_calculated[j].x, image_points_calculated[j].y);
//			image_points_old_mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points_old[j].x, image_points_old[j].y);
//		}
//		err = cv::norm(image_points_calculated_mat, image_points_old_mat, cv::NORM_L2);
//		err /= corner_points_counts;
//		total_err += err;
//		cout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
//		fout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
//	}
//	cout << "����ƽ����" << total_err / image_num << "����" << endl;
//	fout << "����ƽ����" << total_err / image_num << "����" << endl;
//	cout << "�������" << endl;
//
//	fout.close();
//
//	cv::Mat mapx = cv::Mat(image_size, CV_32FC1);
//	cv::Mat mapy = cv::Mat(image_size, CV_32FC1);
//	cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
//	cout << "�������ͼ��" << endl;
//	string imageFileName;
//	std::stringstream StrStm;
//	for (int i = 0; i < image_num; i++)
//	{
//		cout << "Frame #" << i + 1 << endl;
//		cv::initUndistortRectifyMap(cameraMatrix, distCoefficients, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
//		cv::Mat src_image = cv::imread(imgList[i].c_str(), 1);
//		cv::Mat new_image = src_image.clone();
//		cv::remap(src_image, new_image, mapx, mapy, cv::INTER_LINEAR);
//		imshow("ԭʼͼ��", src_image);
//		imshow("������ͼ��", new_image);
//
//		StrStm.clear();
//		imageFileName.clear();
//		StrStm << i + 1;
//		StrStm >> imageFileName;
//		imageFileName += "_d.jpg";
//		cv::imwrite(imageFileName, new_image);
//
//		cv::waitKey(200);
//	}
//	cout << "�������" << endl;
//
//	cv::waitKey(0);
//
//	system("pause");
//
//	return 0;
//}