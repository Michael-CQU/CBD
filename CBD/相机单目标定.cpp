//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <iostream>
//#include <fstream>
//#include <vector>
//
//using namespace cv;
//using namespace std;
//
//void main(char *args)
//{
//    //�����ļ�����
//    std::vector<std::string>  filenames;
//
//    //��Ҫ���ĵĲ���
//    //������궨��ָ�������ͼƬ·�����Լ��궨��������ļ�
//    string infilename = "sample/left/filename.txt";        //������������left��Ϊright
//    string outfilename = "sample/left/caliberation_result.txt";
//
//    //�궨����ͼƬ�ļ���·��,ÿһ�б���һ���궨ͼƬ��·��  ifstream �Ǵ�Ӳ�̶����ڴ�
//    ifstream fin(infilename);
//    //����궨�Ľ��  ofstream �Ǵ��ڴ�д��Ӳ��
//    ofstream fout(outfilename);
//
//    /*
//    1.��ȡ��һ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ������ȡÿ���ǵ�����������ϵ�е�����
//    ��������ϵ��ԭ��λ��ͼ������Ͻ�
//    */
//    std::cout << "��ʼ��ȡ�ǵ�......" << std::endl;;
//    //ͼ������
//    int imageCount = 0;
//    //ͼ��ߴ�
//    cv::Size imageSize;
//    //�궨����ÿ��ÿ�еĽǵ���
//    cv::Size boardSize = cv::Size(9, 6);
//    //����ÿ��ͼ���ϼ�⵽�Ľǵ�
//    std::vector<Point2f>  imagePointsBuf;
//    //�����⵽�����нǵ�
//    std::vector<std::vector<Point2f>> imagePointsSeq;
//    char filename[100];
//    if (fin.is_open())
//    {
//        //��ȡ��ϣ�
//        while (!fin.eof())
//        {
//            //һ�ζ�ȡһ��
//            fin.getline(filename, sizeof(filename) / sizeof(char));
//            //�����ļ���
//            filenames.push_back(filename);
//            //��ȡͼƬ
//            Mat imageInput = cv::imread(filename);
//            //�����һ��ͼƬʱ��ȡͼ�����Ϣ
//            if (imageCount == 0)
//            {
//                imageSize.width = imageInput.cols;
//                imageSize.height = imageInput.rows;
//                std::cout << "imageSize.width = " << imageSize.width << std::endl;
//                std::cout << "imageSize.height = " << imageSize.height << std::endl;
//            }
//
//            std::cout << "imageCount = " << imageCount << std::endl;
//            imageCount++;
//
//            //��ȡÿһ��ͼƬ�Ľǵ�
//            if (cv::findChessboardCorners(imageInput, boardSize, imagePointsBuf) == 0)
//            {
//                //�Ҳ����ǵ�
//                std::cout << "Can not find chessboard corners!" << std::endl;
//                exit(1);
//            }
//            else
//            {
//                Mat viewGray;
//                //ת��Ϊ�Ҷ�ͼƬ
//                cv::cvtColor(imageInput, viewGray, cv::COLOR_BGR2GRAY);
//                //�����ؾ�ȷ��   �Դ���ȡ�Ľǵ���о�ȷ��
//                cv::find4QuadCornerSubpix(viewGray, imagePointsBuf, cv::Size(5, 5));
//                //���������ص�
//                imagePointsSeq.push_back(imagePointsBuf);
//                //��ͼ������ʾ�ǵ�λ��
//                cv::drawChessboardCorners(viewGray, boardSize, imagePointsBuf, true);
//                //��ʾͼƬ
//                //cv::imshow("Camera Calibration", viewGray);
//                cv::imwrite("test.jpg", viewGray);
//                //�ȴ�0.5s
//                // waitKey(500);
//            }
//        }        
//        
//        //����ÿ��ͼƬ�ϵĽǵ��� 54
//        int cornerNum = boardSize.width * boardSize.height;
//
//        //�ǵ�����
//        int total = imagePointsSeq.size()*cornerNum;
//        std::cout << "total = " << total << std::endl;
//
//        for (int i = 0; i < total; i++)
//        {
//            int num = i / cornerNum;
//            int p = i%cornerNum;
//            //cornerNum��ÿ��ͼƬ�Ľǵ���������ж������Ϊ����������ڵ���
//            if (p == 0)
//            {                                        
//                std::cout << "\n�� " << num+1 << "��ͼƬ������ -->: " << std::endl;
//            }
//            //������еĽǵ�
//            std::cout<<p+1<<":("<< imagePointsSeq[num][p].x;
//            std::cout << imagePointsSeq[num][p].y<<")\t";
//            if ((p+1) % 3 == 0)
//            {
//                std::cout << std::endl;
//            }
//        }
//
//        std::cout << "�ǵ���ȡ���!" << std::endl;
//
//        /*
//        2.������궨 ��������ϵԭ��λ�ڱ궨�����Ͻ�(��һ����������Ͻ�)
//        */
//        std::cout << "��ʼ�궨" << std::endl;
//        //������ά��Ϣ��������������������ϵ������
//        //ʵ�ʲ����õ��궨����ÿ�����̸�Ĵ�С
//        cv::Size squareSize = cv::Size(26, 26);
//        //����ͼƬ�ǵ�����
//        std::vector<int> pointCounts;
//        //����궨���Ͻǵ����ά����
//        std::vector<std::vector<cv::Point3f>> objectPoints;
//        //������ڲ������� M=[fx �� u0,0 fy v0,0 0 1]
//        cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, Scalar::all(0));
//        //�������5������ϵ��k1,k2,p1,p2,k3
//        cv::Mat distCoeffs = cv::Mat(1, 5, CV_64F, Scalar::all(0));
//        //ÿ��ͼƬ����ת����
//        std::vector<cv::Mat> tvecsMat;
//        //ÿ��ͼƬ��ƽ������
//        std::vector<cv::Mat> rvecsMat;
//
//        //��ʼ���궨���Ͻǵ����ά����
//        int i, j, t;
//        for (t = 0; t < imageCount; t++)
//        {
//            std::vector<cv::Point3f> tempPointSet;
//            //����
//            for (i = 0; i < boardSize.height; i++)
//            {
//                //����
//                for (j = 0; j < boardSize.width; j++)
//                {
//                    cv::Point3f realPoint;
//                    //����궨�������������ϵ��z=0��ƽ���ϡ�
//                    realPoint.x = i*squareSize.width;
//                    realPoint.y = j*squareSize.height;
//                    realPoint.z = 0;
//                    tempPointSet.push_back(realPoint);
//                }
//            }
//            objectPoints.push_back(tempPointSet);
//        }
//
//        //��ʼ��ÿ��ͼ���еĽǵ��������ٶ�ÿ��ͼ���ж����Կ��������ı궨��
//        for (i = 0; i < imageCount; i++)
//        {
//            pointCounts.push_back(boardSize.width*boardSize.height);
//        }
//        //��ʼ�궨
//        cv::calibrateCamera(objectPoints, imagePointsSeq, imageSize, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);
//        std::cout << "�궨���" << std::endl;
//        //�Ա궨�����������
//        std::cout << "��ʼ���۱궨���......" << std::endl;
//        //����ͼ���ƽ�������ܺ�
//        double totalErr = 0.0;
//        //ÿ��ͼ���ƽ�����
//        double err = 0.0;
//        //�������¼���õ���ͶӰ��
//        std::vector<cv::Point2f> imagePoints2;
//        std::cout << "ÿ��ͼ��ı궨���:" << std::endl;
//        fout << "ÿ��ͼ��ı궨���:" << std::endl;
//        for (i = 0; i < imageCount; i++)
//        {
//            std::vector<cv::Point3f> tempPointSet = objectPoints[i];
//            //ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ��imagePoints2������������ϵ�µĵ����꣩
//            cv::projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, imagePoints2);
//            //�����µ�ͶӰ��;ɵ�ͶӰ��֮������
//            std::vector<cv::Point2f> tempImagePoint = imagePointsSeq[i];
//            cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
//            cv::Mat imagePoints2Mat = cv::Mat(1, imagePoints2.size(), CV_32FC2);
//            for (int j = 0; j < tempImagePoint.size(); j++)
//            {
//                imagePoints2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(imagePoints2[j].x, imagePoints2[j].y);
//                tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
//            }
//            //Calculates an absolute difference norm or a relative difference norm.
//            err = cv::norm(imagePoints2Mat, tempImagePointMat, NORM_L2);
//            totalErr += err /= pointCounts[i];
//            std::cout << "  ��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
//            fout<<  "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
//
//        }
//        //ÿ��ͼ���ƽ�������
//        std::cout << "  ����ƽ�����:" << totalErr / imageCount << "����" << std::endl;
//        fout << "����ƽ�����:" << totalErr / imageCount << "����" << std::endl;
//        std::cout << "�������!" << std::endl;
//        //����궨���
//        std::cout << "��ʼ����궨���....." << std::endl;
//        //����ÿ��ͼ�����ת����
//        cv::Mat rotationMatrix = cv::Mat(3, 3, CV_32FC1, Scalar::all(0));
//        fout << "����ڲ�������:" << std::endl;
//        fout << cameraMatrix << std::endl << std::endl;
//        fout << "����ϵ��:" << std::endl;
//        fout << distCoeffs << std::endl << std::endl;
//
//        for (int i = 0; i < imageCount; i++)
//        {
//            fout << "��" << i + 1 << "��ͼ�����ת����:" << std::endl;
//            fout << tvecsMat[i] << std::endl;
//            //����ת����ת��Ϊ���Ӧ����ת����
//            cv::Rodrigues(tvecsMat[i], rotationMatrix);
//            fout << "��" << i + 1 << "��ͼ�����ת����:" << std::endl;
//            fout << rotationMatrix << std::endl;
//            fout << "��" << i + 1 << "��ͼ���ƽ������:" << std::endl;
//            fout << rvecsMat[i] << std::endl;
//        }
//        std::cout << "�������" << std::endl;
//
//        /************************************************************************
//        ��ʾ������
//        *************************************************************************/
//        cv::Mat mapx = cv::Mat(imageSize, CV_32FC1);
//        cv::Mat mapy = cv::Mat(imageSize, CV_32FC1);
//        cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
//        std::cout << "��ʾ����ͼ��" << endl;
//        for (int i = 0; i != imageCount; i++)
//        {
//            std::cout << "Frame #" << i + 1 << "..." << endl;
//            //����ͼƬ���������ӳ�����mapx��mapy(����������У��������У����Ҫʹ��˫��)
//            initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, imageSize, CV_32FC1, mapx, mapy);
//            //��ȡһ��ͼƬ
//            Mat imageSource = imread(filenames[i]);
//            Mat newimage = imageSource.clone();
//            //��һ�ֲ���Ҫת������ķ�ʽ
//            //undistort(imageSource,newimage,cameraMatrix,distCoeffs);
//            //����У��
//            remap(imageSource, newimage, mapx, mapy, INTER_LINEAR);
//            imshow("ԭʼͼ��", imageSource);
//            imshow("������ͼ��", newimage);
//            waitKey();
//        }
//
//        //�ͷ���Դ
//        fin.close();
//        fout.close();
//        system("pause");        
//    }
//}