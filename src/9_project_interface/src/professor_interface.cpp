#include "professor_image_elab_interface.hpp"
#include "professor_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

#include <vector>
#include <atomic>
#include <unistd.h>
#include <stdlib.h>

#include <experimental/filesystem>
#include <sstream>

namespace professor {
	//-------------------------------------------------------------------------
	//          GENERIC IMAGE LISTENER IMPLEMENTATION
	//-------------------------------------------------------------------------
	void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
		static size_t id = 0;
		static bool init = false;
		static std::string folder_path;

		if(!init){
			bool exist = true;
			int i = 0;
			while(exist && i < 1000){
				std::stringstream ss;
				ss << config_folder << "/camera_image" << std::setw(3) << std::setfill('0') << i << "/";
 				folder_path = ss.str();
				exist = std::experimental::filesystem::exists(folder_path);
				i++;
			}
      
			if(i > 999 || !std::experimental::filesystem::create_directories(folder_path)){
				throw std::logic_error( "NO EMTY FOLDER" );
			}

			init = true;
		}

		cv::imshow(topic, img_in);
		char c;
		c = cv::waitKey(30);
		std::stringstream img_file;
		switch(c){
			case 's':
				img_file << folder_path << std::setfill('0') << std::setw(3)  << (id++) << ".jpg";
				cv::imwrite( img_file.str(), img_in );

				std::cout << "Saved image " << img_file.str() << std::endl;
				break;
			default:
				break;
		}
	}


	//-------------------------------------------------------------------------
	//          EXTRINSIC CALIB IMPLEMENTATION
	//-------------------------------------------------------------------------

	// Defintion of the function pickNPoints and the callback mouseCallback.
	// The function pickNPoints is used to display a window with a background
	// image, and to prompt the user to select n points on this image.
	static cv::Mat bg_img;
	static std::vector<cv::Point2f> result;
	static std::string name;
	static std::atomic<bool> done;
	static int n;
	static double show_scale = 1.0;

	void mouseCallback(int event, int x, int y, int, void* p){
		if(event != cv::EVENT_LBUTTONDOWN || done.load()) return;
    
		result.emplace_back(x*show_scale, y*show_scale);
		cv::circle(bg_img, cv::Point(x,y), 20/show_scale, cv::Scalar(0,0,255), -1);
		cv::imshow(name.c_str(), bg_img);

		if(result.size() >= n){
			usleep(500*1000);
			done.store(true);
		}
	}

	std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat& img){
		result.clear();
		cv::Size small_size(img.cols/show_scale, img.rows/show_scale);
		cv::resize(img, bg_img, small_size);
		// bg_img = img.clone();
		name = "Pick " + std::to_string(n0) + " points";
		cv::imshow(name.c_str(), bg_img);
		cv::namedWindow(name.c_str());
		n = n0;

		done.store(false);

		cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
		while(!done.load()){
			cv::waitKey(500);
		}

		cv::destroyWindow(name.c_str());
		return result;
	}

	bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
		std::string file_path = config_folder + "/extrinsicCalib.csv";
		std::vector<cv::Point2f> image_points;
		if(!std::experimental::filesystem::exists(file_path)){
			std::experimental::filesystem::create_directories(config_folder);
			image_points = pickNPoints(4, img_in);
			// SAVE POINT TO FILE
			// std::cout << "IMAGE POINTS: " << std::endl;
 			// for (const auto pt: image_points) {
			//   std::cout << pt << std::endl;
			// }
			std::ofstream output(file_path);
			if(!output.is_open()){
				throw std::runtime_error("Cannot write file: " + file_path);
			}
			for(const auto pt: image_points){
				output << pt.x << " " << pt.y << std::endl;
			}
			output.close();
		}else{
			// LOAD POINT FROM FILE
			std::ifstream input(file_path);
			if (!input.is_open()){
				throw std::runtime_error("Cannot read file: " + file_path);
			}
			while(!input.eof()){
				double x, y;
				if(!(input >> x >> y)){
					if (input.eof()) break;
					else{
						throw std::runtime_error("Malformed file: " + file_path);
					}
				}
				image_points.emplace_back(x, y);
			}
			input.close();
		}

		cv::Mat dist_coeffs;
		dist_coeffs = (cv::Mat1d(1,4) << 0, 0, 0, 0, 0);
		bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

		// cv::Mat Rt;
		// cv::Rodrigues(rvec_, Rt);
		// auto R = Rt.t();
		// auto pos = -R * tvec_;

		if(!ok){
			std::cerr << "FAILED SOLVE_PNP" << std::endl;
		}

		return ok;
	}

	//-------------------------------------------------------------------------
	//          UNDISTORT IMAGE IMPLEMENTATION
	//-------------------------------------------------------------------------
	void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
		// SLOW VERSION
		// undistort(img_in, img_out, cam_matrix, dist_coeffs);

		// OPTIMIZED VERSION
		static bool maps_initialized = false;
		static cv::Mat full_map1, full_map2;

		if(!maps_initialized){
			// Note: m1type=CV_16SC2 to use fast fixed-point maps (see cv::remap)
			cv::Mat R;
			cv::initUndistortRectifyMap(cam_matrix, dist_coeffs, R, cam_matrix, img_in.size(), CV_16SC2, full_map1, full_map2);

			maps_initialized = true;
		}

		// Initialize output image
		cv::remap(img_in, img_out, full_map1, full_map2, cv::INTER_LINEAR);
	}

	//-------------------------------------------------------------------------
	//          FIND PLANE TRANSFORM
	//-------------------------------------------------------------------------
	void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, const std::vector<cv::Point2f>& dest_image_points_plane, cv::Mat& plane_transf, const std::string& config_folder){
		cv::Mat image_points;

		// project points
		cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

		plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
	}

	//-------------------------------------------------------------------------
	//          UNWARP TRANSFORM
	//-------------------------------------------------------------------------
	void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, const std::string& config_folder){
		cv::warpPerspective(img_in, img_out, transf, img_in.size());
	}

	//-------------------------------------------------------------------------
	//          PROCESS MAP
	//-------------------------------------------------------------------------
	bool processObstacles(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& obstacle_list){
		// Find red regions: h values around 0 (positive and negative angle: [0,15] U [160,179])
		cv::Mat red_mask_low, red_mask_high, red_mask;
		cv::inRange(hsv_img, cv::Scalar(0, 40, 40), cv::Scalar(15, 255, 255), red_mask_low);
		cv::inRange(hsv_img, cv::Scalar(163, 40, 40), cv::Scalar(180, 255, 255), red_mask_high);
		cv::add(red_mask_low, red_mask_high, red_mask);

		// cv::Mat img_small;
		// cv::resize(red_mask, img_small, cv::Size(640, 512));
		// cv::imshow("obstacles", red_mask);
		// cv::waitKey(1);

		std::vector<std::vector<cv::Point>> contours, contours_approx;
		std::vector<cv::Point> approx_curve;
		cv::Mat contours_img;

		// Process red mask
		// contours_img = img_in.clone();
		cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		// drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
		// std::cout << "N. contours: " << contours.size() << std::endl;
		for(int i=0; i<contours.size(); ++i){
			//std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
			approxPolyDP(contours[i], approx_curve, 3, true);

			Polygon scaled_contour;
			for (const auto& pt: approx_curve) {
				scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
			}
			obstacle_list.push_back(scaled_contour);
			// contours_approx = {approx_curve};
			// drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
			// std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
		}
		// std::cout << std::endl;
		// cv::imshow("Original", contours_img);
		// cv::waitKey(1);

		return true;
	}

	bool processGate(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& gate_list){
		// Find purple regions
		cv::Mat purple_mask;
		cv::inRange(hsv_img, cv::Scalar(130, 10, 10), cv::Scalar(165, 255, 255), purple_mask);

		std::vector<std::vector<cv::Point>> contours, contours_approx;
		std::vector<cv::Point> approx_curve;
		// cv::Mat contours_img;

		// Process purple mask
		// contours_img = hsv_img.clone();
		cv::findContours(purple_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		// drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 4, cv::LINE_AA);
		// std::cout << "N. contours: " << contours.size() << std::endl;

		bool res = false;
		for(auto& contour: contours){
			const double area = cv::contourArea(contour);
			// std::cout << "AREA " << area << std::endl;
			// std::cout << "SIZE: " << contours.size() << std::endl;
			if(area > 500){
				approxPolyDP(contour, approx_curve, 30, true);

				if(approx_curve.size()!=4) continue;

				// contours_approx = {approx_curve};
				// drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

                Polygon gate;
				for(const auto& pt: approx_curve){
					gate.emplace_back(pt.x/scale, pt.y/scale);
				}
				gate_list.push_back(gate);
				res = true;
			}
		}

		// cv::imshow("Original", contours_img);
		// cv::waitKey(1);

		return res;
	}

	bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<Polygon>& gate_list, const std::string& config_folder){
		// Convert color space from BGR to HSV
		cv::Mat hsv_img;
		cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

		const bool res1 = processObstacles(hsv_img, scale, obstacle_list);
		if(!res1) std::cout << "processObstacles return false" << std::endl;

		const bool res2 = processGate(hsv_img, scale, gate_list);
		if(!res2) std::cout << "processGate return false" << std::endl;

		return res1 && res2;
	}

	//-------------------------------------------------------------------------
	//          FIND ROBOT
	//-------------------------------------------------------------------------
	bool processRobot(const cv::Mat& hsv_img, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string ns){
		cv::Mat robot_mask;
		if (ns == "/my_robot_0") {
			// Robot 0 blue
			cv::inRange(hsv_img, cv::Scalar(90, 50, 50), cv::Scalar(140, 255, 255), robot_mask);
		}else if (ns == "/my_robot_1"){
			// Robot 1 yellow
			cv::inRange(hsv_img, cv::Scalar(15, 0, 0), cv::Scalar(36, 255, 255), robot_mask);
		}else if (ns == "/my_robot_2"){
			// Robot 2 green
			cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), robot_mask);
		}

		// Process masks
		std::vector<std::vector<cv::Point>> contours, contours_approx;
		std::vector<cv::Point> approx_curve;
		cv::findContours(robot_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		// cv::imshow("filterrrr"+ns, robot_mask);
		// cv::waitKey(1);
		// cv::Mat contours_img;
		// contours_img = hsv_img.clone();
		// drawContours(contours_img, contours, -1, cv::Scalar(0,0,0), 4, cv::LINE_AA);
		// std::cout << "N. contours: " << contours.size() << std::endl;

		bool found = false;
		for(int i=0; i<contours.size(); ++i){
			// std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;

			cv::approxPolyDP(contours[i], approx_curve, 10, true);
			contours_approx = {approx_curve};

			// cv::drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

			double area = cv::contourArea(approx_curve);

			if(approx_curve.size() != 3) continue;

			if(area < 300 || area>3000) continue;

			found = true;
			break;
		}

		if(found){
			for(const auto& pt: approx_curve){
				triangle.emplace_back(pt.x/scale, pt.y/scale);
			}

			double cx = 0, cy = 0;
			for(auto item: triangle){
				cx += item.x;
				cy += item.y;
			}
			cx /= triangle.size();
			cy /= triangle.size();

			double dst = 0;
			Point vertex;
			for(auto& item: triangle){
				double dx = item.x-cx;
				double dy = item.y-cy;
				double curr_d = dx*dx + dy*dy;
				if(curr_d > dst){
					dst = curr_d;
					vertex = item;
				}
			}

			// cv::Moments m = cv::moments(approx_curve, false);
			// cv::Point center(m.m10/m.m00, m.m01/m.m00);
			// cv::Vec4f line;
			// cv::fitLine(approx_curve, line, cv::DIST_L2, 0, 0.01, 0.01);
			// cv::line(warpedFrame, cv::Point(line[2], line[3]), cv::Point(line[2]+line(0)*80, line(3)+line(1)*80), (0,255,0), 2);


			// cv::line(contours_img, center*scale, vertex*scale, (0,255,0), 2);
			// cv::circle(contours_img, center*scale, 20, cv::Scalar(0,0,0), -1);

			double dx = cx-vertex.x;
			double dy = cy-vertex.y;

			x = cx;
			y = cy;
			theta = std::atan2(dy, dx);

			// covariance = {};

			// std::cout << xc_m << " " << yc_m << " " << theta*180/M_PI << std::endl;
		}

		// cv::imshow("Original", contours_img);
		// cv::waitKey(1);

		return found;
	}

	bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string ns, const std::string& config_folder){
		// Convert color space from BGR to HSV
		cv::Mat hsv_img;
		cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);
		return processRobot(hsv_img, scale, triangle, x, y, theta, ns);
	}

	//-------------------------------------------------------------------------
	//          PLAN PATH
	//-------------------------------------------------------------------------
	bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<Polygon>& gate_list, const std::vector<float> x, const std::vector<float> y, const std::vector<float> theta, std::vector<Path>& path, const std::string& config_folder){
		float ds = 0.1;
		float dx0 = ( ( (gate_list[0][0].x + gate_list[0][1].x + gate_list[0][2].x + gate_list[0][3].x) / 4) - x[0]) / 50;
		float dx1 = ( ( (gate_list[0][0].x + gate_list[0][1].x + gate_list[0][2].x + gate_list[0][3].x) / 4) - x[1]) / 50;
		float dx2 = ( ( (gate_list[0][0].x + gate_list[0][1].x + gate_list[0][2].x + gate_list[0][3].x) / 4) - x[2]) / 50;
		float dy0 = ( ( (gate_list[0][0].y + gate_list[0][1].y + gate_list[0][2].y + gate_list[0][3].y) / 4) - y[0]) / 50;
		float dy1 = ( ( (gate_list[0][0].y + gate_list[0][1].y + gate_list[0][2].y + gate_list[0][3].y) / 4) - y[1]) / 50;
		float dy2 = ( ( (gate_list[0][0].y + gate_list[0][1].y + gate_list[0][2].y + gate_list[0][3].y) / 4) - y[2]) / 50;

		// shortest dummy paths for the 3 robots
		for(float l=0, s=0; l<50; l++, s+=ds){
		    path[0].points.emplace_back(s, x[0]+dx0*l, y[0]+dy0*l, theta[0], 0.0);
		}
		for(float l=0, s=0; l<50; l++, s+=ds){
		    path[1].points.emplace_back(s, x[1]+dx1*l, y[1]+dy1*l, theta[1], 0.0);
		}
		for(float l=0, s=0; l<50; l++, s+=ds){
		    path[2].points.emplace_back(s, x[2]+dx2*l, y[2]+dy2*l, theta[2], 0.0);
		}

		return true;
	}
}
