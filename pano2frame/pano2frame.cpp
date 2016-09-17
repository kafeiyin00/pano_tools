// pano2frame.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include "pano_info.h"
#include "LatLong-UTMconversion.h"
#include "opencv3_dev.h"

namespace bf = boost::filesystem;


void readGPSData(std::string& gpsfile, std::vector<gpsdata> &datas)
{
	std::ifstream ifs;
	ifs.open(gpsfile, std::ios::in);
	if (ifs.fail())
	{
		return;
	}

	std::string line;
	while (ifs.good())
	{
		std::getline(ifs, line);
		std::vector<std::string> values;
		boost::split(values, line, boost::is_any_of(" ;\t"));
		double latitude, longitude, altitude, heading(0), pitch(0),roll(0),x(0),y(0),z(0);
		latitude = std::atof(values[13].c_str());
		longitude = std::atof(values[14].c_str());
		altitude = std::atof(values[15].c_str());
		heading = std::atof(values[18].c_str());
		pitch = std::atof(values[17].c_str());
		roll = std::atof(values[16].c_str());
		// 23 --> wgs84
		char zone[256];
		LLtoUTM(23, latitude, longitude, y, x, zone);
		z = altitude;
		//12张图片
		for (int i = 0; i < 12; i++)
		{
			datas.push_back(gpsdata(x, y, z, heading, pitch, roll));
		}
	}
}


//pitch -> roll -> heading
void pano2frame2(const std::string& panopath, const std::string& framepath,
	double heading, double pitch, double roll,
	cv::Mat& rotation0)
{
	const double myPI = 3.1415926;

	//ccd 相机相面大小 单位mm
	double ccd_M_Width = 36;
	double ccd_M_Height = 24;

	//ccd 相机阵列大小 单位无
	int ccd_P_Width = 690 * 3;
	int ccd_P_Height = 460 * 3;

	//ccd 相机焦距 单位mm
	double F_M = 20;

	double Xc, Yc, Zc;
	double Xc_W, Yc_W, Zc_W;

	double R = 200;
	double xwp, ywp, zwp, xyzwp2, xsp, ysp, zsp;
	double theta, omiga;

	int  w = 4096;
	int  h = 2048;

	double y, x;
	int u, v;

	cv::Mat originPano = cv::imread(panopath.c_str(), cv::IMREAD_COLOR);


	cv::Mat recoverFrame(ccd_P_Height, ccd_P_Width, CV_8UC3);
	recoverFrame.setTo(cv::Scalar(0, 0, 0));

	heading = (heading / 180.0)*myPI;
	pitch = (pitch / 180.0)*myPI;
	roll = (roll / 180.0)*myPI;
	double r_heading[3][3] = {
		{cos(heading),-sin(heading),0},
		{sin(heading),cos(heading),0},
		{0,0,1}
	};
	double r_pitch[3][3] = {
		{cos(pitch),0,sin(pitch)},
		{0,1,0},
		{-sin(pitch),0,cos(pitch)} 
	};
	double r_roll[3][3] = {
		{1,0,0},
		{0,cos(roll),-sin(roll)},
		{0,sin(roll),cos(roll)}
	};
	double r_rotation[3][3];
	cv::Mat
		R_heading(3, 3, CV_64FC1, r_heading),
		R_pitch(3, 3, CV_64FC1, r_pitch),
		R_roll(3, 3, CV_64FC1, r_roll),
		R_rotation(3, 3, CV_64FC1, r_rotation);

	//--计算旋转矩阵
	//pitch -> roll ->heading
	R_rotation = R_heading*R_roll*R_pitch;//此时是相机坐标到世界的转换
	R_rotation.copyTo(rotation0);
	for (int i = 0; i<ccd_P_Height; i++)
	{
		for (int j = 0; j<ccd_P_Width; j++)
		{
			//将像素坐标转为公制坐标
			Xc = (j - ccd_P_Width / 2.0) * (ccd_M_Width / (double)ccd_P_Width);
			Yc = (-i + ccd_P_Height / 2.0) * (ccd_M_Height / (double)ccd_P_Height);
			Zc = F_M;//共心坐标系

			Xc_W = r_rotation[0][0] * Xc + r_rotation[0][1] * Yc + r_rotation[0][2] * Zc;
			Yc_W = r_rotation[1][0] * Xc + r_rotation[1][1] * Yc + r_rotation[1][2] * Zc;
			Zc_W = r_rotation[2][0] * Xc + r_rotation[2][1] * Yc + r_rotation[2][2] * Zc;

			//计算共线的球面坐标

			xwp = Xc_W;
			ywp = Yc_W;
			zwp = Zc_W;

			xyzwp2 = xwp*xwp + ywp*ywp + zwp*zwp;

			xsp = sqrt(R*R*xwp*xwp / xyzwp2);
			ysp = sqrt(R*R*ywp*ywp / xyzwp2);
			zsp = sqrt(R*R*zwp*zwp / xyzwp2);

			//
			//判断象限：
			if (xwp>0 && ywp>0 && zwp>0)
			{
				//1象限
				theta = asin(zsp / R);
				omiga = asin(ysp / (R*cos(theta)));
			}
			if (xwp<0 && ywp>0 && zwp>0)
			{
				//2象限
				theta = asin(zsp / R);
				omiga = myPI - asin(ysp / (R*cos(theta)));
			}
			if (xwp<0 && ywp<0 && zwp>0)
			{
				//3象限
				theta = asin(zsp / R);
				omiga = myPI + asin(ysp / (R*cos(theta)));
			}
			if (xwp>0 && ywp<0 && zwp>0)
			{
				//4象限
				theta = asin(zsp / R);
				omiga = 2*myPI - asin(ysp / (R*cos(theta)));
			}
// 			//----------------------------------------
			if (xwp>0 && ywp>0 && zwp<0)
			{
				//5象限
				theta = -asin(zsp / R);
				omiga = asin(ysp / (R*cos(theta)));
			}
			if (xwp<0 && ywp>0 && zwp<0)
			{
				//6象限
				theta = -asin(zsp / R);
				omiga = myPI - asin(ysp / (R*cos(theta)));
			}
			if (xwp<0 && ywp<0 && zwp<0)
			{
				//7象限
				theta = -asin(zsp / R);
				omiga = myPI + asin(ysp / (R*cos(theta)));
			}
			if (xwp>0 && ywp<0 && zwp<0)
			{
				//8象限
				theta = -asin(zsp / R);
				omiga = 2 * myPI - asin(ysp / (R*cos(theta)));
			}
			//结算对应像素位置：
			y = -(theta*h) / myPI+h/2;
			x = (omiga*w) / (2 * myPI)+w/2;
 			if (x > w)
			{
				x -= w;
			}

			u = (int)(x + 0.5);
			v = (int)(y + 0.5);

			if (u<w && u>0 && v<h && v>0)
			{
				recoverFrame.at<cv::Vec3b>(i, j)[0] = originPano.at<cv::Vec3b>(v, u)[0];
				recoverFrame.at<cv::Vec3b>(i, j)[1] = originPano.at<cv::Vec3b>(v, u)[1];
				recoverFrame.at<cv::Vec3b>(i, j)[2] = originPano.at<cv::Vec3b>(v, u)[2];
			}
		}//结束列
	}//结束行

	// 消除黑线
	for (int i = 0; i<ccd_P_Height; i++)
	{
		for (int j = 1; j<ccd_P_Width - 1; j++)
		{
			if (recoverFrame.at<cv::Vec3b>(i, j)[0] == 0 && recoverFrame.at<cv::Vec3b>(i, j)[1] == 0 && recoverFrame.at<cv::Vec3b>(i, j)[2] == 0)
			{
				recoverFrame.at<cv::Vec3b>(i, j)[0] = recoverFrame.at<cv::Vec3b>(i, j - 1)[0];
				recoverFrame.at<cv::Vec3b>(i, j)[1] = recoverFrame.at<cv::Vec3b>(i, j - 1)[1];
				recoverFrame.at<cv::Vec3b>(i, j)[2] = recoverFrame.at<cv::Vec3b>(i, j - 1)[2];
			}
		}

	}
	cv::imwrite(framepath, recoverFrame);
}

int _tmain(int argc, _TCHAR* argv[])
{
	std::string panofilefolderpath, framefolder, gpsfilename, cameraposfile, pairfilename, jsonfilename;
	std::cout << "输入全景目录\n";
	std::cin >> panofilefolderpath;
	std::cout << "输入恢复目录\n";
	std::cin >> framefolder;
	std::cout << "输入gps地址\n";
	std::cin >> gpsfilename;
	//std::cout << "输入恢复相机pos地址\n";
	//std::cin >> cameraposfile;
	//std::cout << "输入pair地址\n";
	//std::cin >> pairfilename;
	bf::create_directory(framefolder + "//result");
	cameraposfile = framefolder + "//result//regps.txt";
	pairfilename = framefolder + "//pair.txt";
	jsonfilename = framefolder + "//result//data.json";

	std::vector<frame_data> frameDatas;
	//转换相片
    	{
    		bf::path panofolder(panofilefolderpath);
    		bf::directory_iterator end_iter;
    		for (bf::directory_iterator beg_iter(panofolder); beg_iter != end_iter; beg_iter++)
    		{
				if (bf::is_directory(*beg_iter))
				{
					std::cout << "已忽略一个子目录" << std::endl;
					continue;
				}
    			for (int i = 0; i < 12; i++)
    			{
    				std::stringstream recoverframefile;
    				std::string temp_name = beg_iter->path().filename().string();
    				temp_name.erase(temp_name.begin() + temp_name.size()- 4, temp_name.begin() + temp_name.size());
    				recoverframefile << (framefolder + "//" + temp_name);
    				recoverframefile <<"_"<<std::hex<< i << ".jpg";
    				std::cout << recoverframefile.str() << std::endl;
 				frame_data fd;
 				cv::Mat R0;
 				pano2frame2((beg_iter->path().string().c_str()), recoverframefile.str(), i*30, 90, 90, R0);
 				cv::cv2eigen(R0,fd._R0);
 				frameDatas.push_back(fd);
    			}
    		}
    	}

// 	记录gps到exif
//  	{
//  			std::vector<gpsdata> datas;
//  			readdata(gpsfilename, datas);
//  			std::cout << "共" << datas.size() << "次曝光" << std::endl;
//  			bf::path frmfolder(framefolder);
//  			bf::directory_iterator end_iter;
//  			std::vector<std::string> files;
//  			for (bf::directory_iterator beg_iter(frmfolder); beg_iter != end_iter; beg_iter++)
//  			{
//  				files.push_back(beg_iter->path().string());
//  			}
//  			int i = 0;
//  			for (auto beg = files.begin(), end = files.end(); beg != end; beg++)
//  			{
//  				std::stringstream ss;
//  				//ss.precision()
//  				ss << "exiftool"
//  					//<< " -exif:focallength=" << 10
//  					<< " -exif:gpslatitude=" << std::fixed << datas[i]._latitude
//  					<< " -exif:gpslongitude=" << std::fixed << datas[i]._longitude
//  					<< " -exif:gpsaltitude=" << std::fixed << datas[i]._altitude << " " << *beg;
//  				system(ss.str().c_str());
//  				i++;
//  			}
//  		}
	
	//输出到txt
	//序列化
	
	std::vector<gpsdata> datas;
	{
		std::ofstream frame_pos_ofs;
		frame_pos_ofs.open(cameraposfile);
		if (frame_pos_ofs.fail())
		{
			return 0;
		}

		readGPSData(gpsfilename, datas);
		std::cout << "共" << datas.size() << "次曝光" << std::endl;
		bf::path frmfolder(framefolder);
		bf::directory_iterator end_iter;
		std::vector<bf::path> files;
		for (bf::directory_iterator beg_iter(frmfolder); beg_iter != end_iter; beg_iter++)
		{
			if (bf::is_directory(*beg_iter))
			{
				std::cout << "已忽略一个子目录" << std::endl;
				continue;
			}
			files.push_back(beg_iter->path());
		}
		int i = 0;
		for (auto beg = files.begin(), end = files.end(); beg != end; beg++)
		{
			if (bf::is_directory(*beg))
			{
				std::cout << "已忽略一个子目录" << std::endl;
				continue;
			}
			frame_pos_ofs << beg->filename().string() << " "
				<< std::fixed
				<< datas[i]._x << " "
				<< datas[i]._y << " "
				<< datas[i]._z << " "
				<< "\n";
			
			frameDatas[i]._frameName = beg->filename().string();
			frameDatas[i]._gpsData = datas[i];
			i++;
		}
		frame_pos_ofs.close();
	}

		
	
	//添加pair
	{
		std::ofstream ofs_pair(pairfilename);
		std::vector<std::vector<int> > list_pair(datas.size());
		for (int i = 0; i < datas.size(); i++)
		{
			for (int j = i+1; j < datas.size(); j++)
			{
				gpsdata datai, dataj;
				datai = datas[i];
				dataj = datas[j];
				//ofs_pair << i <<" ";
	
				//按照距离
				//double dx, dy;
				//dx = fabs(datai._x - dataj._x);
				//dy = fabs(datai._y - dataj._y);
				//if (dx < 500 && dy < 500
				//	&& (fabs(datai._x - dataj._x) > 1 || fabs(datai._y - dataj._y) > 1))
				//{
				//	//ofs_pair << j << " ";
				//	list_pair[i].push_back(j);
				//}
				//ofs_pair << "\n";
				//按照顺序
				int frame_id = i / 12;
				int temp_frame_id = j / 12;
				if (std::abs(frame_id - temp_frame_id) == 1 || std::abs(frame_id - temp_frame_id) == 2)
				{
					list_pair[i].push_back(j);
				}
					
			}
		}
		for (int i = 0; i < list_pair.size(); i++)
		{
			if (list_pair[i].size()>0)
			{
				ofs_pair << i << " ";
				for (int j = 0; j < list_pair[i].size(); j++)
				{
					ofs_pair << list_pair[i][j] << " ";
				}
				ofs_pair << "\n";
			}
				
		}
	}
	
	//
	std::ofstream file(jsonfilename);
	cereal::JSONOutputArchive archive(file);
	archive(cereal::make_nvp("mydata", frameDatas));
	file.close();

	return 0;
}

