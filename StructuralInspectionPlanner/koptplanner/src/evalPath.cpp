/*!
 * \file plan.cpp
 *
 * More elaborate description
 */
#include "ros/ros.h"
#include <sys/time.h>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Path.h"
#include <std_msgs/Int32.h>
#include <shape_msgs/Plane.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <unordered_set>
#include <map>
#include "std_msgs/String.h"

ros::Publisher obstacle_pub;
ros::Subscriber avoidObstacle_result_sub;
ros::Subscriber marker_sub;
ros::Subscriber stl_sub;

double g_security_distance;
std::string pkgPath;

int obsBoxSize;

std::vector<nav_msgs::Path> mesh;
std::vector<geometry_msgs::PoseStamped> path;

double xmin_obs;
double xmax_obs;
double ymin_obs;
double ymax_obs;
double zmin_obs;
double zmax_obs;

int results[8] = {0,0,0,0,0,0,0,0};
int cptExec;
bool nextObstacle;

long time_EXEC;
long avg_time_EXEC;
bool RVIZ_ON;

void getResult(const std_msgs::Int32::ConstPtr& res);
void getSTL(const nav_msgs::Path::ConstPtr& res);
void getPath(const nav_msgs::Path::ConstPtr& res);
bool isVisibilityBoxOK(std::vector<double> xs, std::vector<double> ys, std::vector<double> zs);
void checkPath();
void publishObstacle();

int main(int argc, char **argv)
{
	ros::init(argc, argv, "evalPath");
	ros::NodeHandle n;

	obstacle_pub = n.advertise<geometry_msgs::PointStamped>("clicked_point", 1);
	avoidObstacle_result_sub = n.subscribe("aO_result", 10000, getResult);
	marker_sub = n.subscribe("visualization_marker", 1, getPath);
	stl_sub = n.subscribe("stl_mesh", 10000, getSTL);	

	obsBoxSize = 5;
	pkgPath = ros::package::getPath("koptplanner");
	g_security_distance = 5.0;
	time_EXEC = 0;
	avg_time_EXEC = 0;
	cptExec = 0;
	nextObstacle = false;
	
	int numExecutions = std::stof(argv[1]);
	ROS_INFO("Number of executions: %d", numExecutions);
	
	if (argc == 3) {
		RVIZ_ON = (bool) std::stoi(argv[2]);
	}
	else {
		RVIZ_ON = false;
	}
	ROS_INFO("RVIZ used: %d", RVIZ_ON);
	
	// Receive the mesh from the stl_mesh topic
	boost::shared_ptr<nav_msgs::Path const> sharedMesh;
	ROS_INFO("Ready to receive the mesh");
	do {
		sharedMesh = ros::topic::waitForMessage<nav_msgs::Path>("stl_mesh", n);
	  	if(sharedMesh != NULL) {
			mesh.push_back(*sharedMesh);
	  	}
	//} while(sharedMesh != NULL);
	} while(!ros::isShuttingDown() && mesh.size() < 396);
	ROS_INFO("Mesh size:%d", (int) mesh.size());
	
	// Receive the path from the visualization_mrker topic
	boost::shared_ptr<nav_msgs::Path const> sharedPath;
	ROS_INFO("Ready to receive the path");
	sharedPath = ros::topic::waitForMessage<nav_msgs::Path>("visualization_marker", n, ros::Duration(0));
	 if(sharedPath != NULL) {
		for(auto it = sharedPath->poses.begin(); it != sharedPath->poses.end(); it++) {
			if(it->pose.position.x != (it+1)->pose.position.x && it->pose.position.y != (it+1)->pose.position.y && it->pose.position.z != (it+1)->pose.position.z) {
				path.push_back(*it);
			}
		}
	}
	ROS_INFO("Path size:%d", (int) path.size());
			
	ros::Rate loop_rate(500);	
		
	ROS_INFO("Waiting for the subscriber for the obstacle pose!");
	while(!ros::isShuttingDown() && obstacle_pub.getNumSubscribers() < 1);
	ROS_INFO("Subscriber found!");
	nextObstacle = true;
			
	while(ros::ok() && cptExec<=numExecutions) {
		if(nextObstacle)
			publishObstacle();
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	ROS_INFO("Results in koptplanner/data/results.txt");

	std::vector<nav_msgs::Path>().swap(mesh);
	std::vector<geometry_msgs::PoseStamped>().swap(path);
	
	return 0;
}

void publishObstacle() {
	ROS_INFO("Iter: %d", cptExec);

	int idVPKO = rand() % (mesh.size()-1) + 1;
	
	geometry_msgs::PointStamped poseObs;
	poseObs.header.seq = cptExec;
	poseObs.header.frame_id = "/kopt_frame";
	
	poseObs.point.x = path[idVPKO].pose.position.x;
	poseObs.point.y = path[idVPKO].pose.position.y;
	poseObs.point.z = path[idVPKO].pose.position.z;
	
	obstacle_pub.publish(poseObs);
	
	xmin_obs = poseObs.point.x-0.5*obsBoxSize-g_security_distance;
	xmax_obs = poseObs.point.x+0.5*obsBoxSize+g_security_distance;	
	ymin_obs = poseObs.point.y-0.5*obsBoxSize-g_security_distance;
	ymax_obs = poseObs.point.y+0.5*obsBoxSize+g_security_distance;	
	zmin_obs = poseObs.point.z-0.5*obsBoxSize-g_security_distance;
	zmax_obs = poseObs.point.z+0.5*obsBoxSize+g_security_distance;
	
	nextObstacle = false;
	timeval time;
	gettimeofday(&time, NULL);
	time_EXEC -= time.tv_sec * 1000000 + time.tv_usec;
}

void getResult(const std_msgs::Int32::ConstPtr& res) {
	std::fstream file;

	// res->data ==0 => No problems during avoidObstacle
	if(res->data == 0) {
		timeval time;
		gettimeofday(&time, NULL);
		time_EXEC += time.tv_sec * 1000000 + time.tv_usec;
		avg_time_EXEC += time_EXEC/1000.0;
		checkPath();
	}
	
	time_EXEC = 0;
	results[res->data]++;
	
	file.open(pkgPath+"/data/results.txt", std::ios::out | std::ios::trunc);
	file << "Executions: " << cptExec << "\n";
	if(results[0] > 0) // Avoid division by 0
		file << "Average execution time: " << avg_time_EXEC/results[0] << "\n";
	else
		file << "Average execution time: " << -1.0 << "\n";
	file << "OK: " << results[0] << "\n";
	file << "ERROR #1 No collision free configuration: " << results[1] << "\n";
	file << "ERROR #2 Path KO: " << results[2] << "\n";
	file << "ERROR #3 Start point KO: " << results[3] << "\n";
	file << "ERROR #4 Waypoint KO: " << results[4] << "\n";
	file << "ERROR #5 Miss VP: " << results[5] << "\n";
	file << "ERROR #6 Corrupted VP: " << results[6] << "\n";
	file << "ERROR #7 Multiples configurations with the same ID: " << results[7] << "\n";

	file.close();
	nextObstacle = true;
	cptExec++;
	
	if(RVIZ_ON)
		ros::Duration(5).sleep();
}
void getSTL(const nav_msgs::Path::ConstPtr& res) {}

void getPath(const nav_msgs::Path::ConstPtr& res) {	
	if(res->poses.size() > 2*mesh.size()) {	
		path.clear();
		path.push_back(*(res->poses.begin()));
		for(auto it = res->poses.begin()+1; it != res->poses.end(); it++) {
			if(it->header.seq != (it-1)->header.seq) {
				path.push_back(*it);
			}
		}
	}
}

void checkPath() {
	std::vector<double> xs;
	std::vector<double> ys;
	std::vector<double> zs;

	std::vector<int> allVPsID(mesh.size()+1); // vector with all the IDs between 0 and path.size()
	std::iota (std::begin(allVPsID), std::end(allVPsID), 0);
	
	std::fstream fileVP;
	std::string lineVP;
	double threshold = 0.0001;
	bool OK = false;
	
	for(std::vector<geometry_msgs::PoseStamped>::iterator it_path = std::begin(path); it_path != std::end(path); it_path++) {
		
		// Check if all the triangles ID are in the path
		auto idVP = std::find(allVPsID.begin(), allVPsID.end(), it_path->header.seq);
		if(idVP != allVPsID.end())
			allVPsID.erase(idVP);
		else if(it_path->header.seq <= mesh.size()) {
			ROS_ERROR("Multiples configurations with the same ID: %d!", it_path->header.seq);
			results[7]++;
			results[0]--;
			ros::shutdown();
			return;
		}
		
		// Check if the configuration is the corresponding database
		if(it_path->header.seq <= mesh.size()) {
			fileVP.open((pkgPath+"/viewpoints/viewpoint_"+std::to_string(it_path->header.seq)+".txt").c_str(), std::ios::in);
				if(fileVP.is_open()) {
					while(getline(fileVP,lineVP)) {
						std::vector<std::string> poseStrVP;
						boost::split(poseStrVP, lineVP, boost::is_any_of("\t"));
					
						if( std::fabs(it_path->pose.position.x - std::atof(poseStrVP[0].c_str())) < threshold &&
							std::fabs(it_path->pose.position.y - std::atof(poseStrVP[1].c_str())) < threshold &&
							std::fabs(it_path->pose.position.z - std::atof(poseStrVP[2].c_str())) < threshold) {
							OK = true;
							break;
						}
					}
				}
				else {
					ROS_ERROR("Viewpoint %d not found", it_path->header.seq);
					ros::shutdown();
				}
				fileVP.close();
			
				if(!OK) {
					ROS_ERROR("Viewpoint %d corrupted", it_path->header.seq);
					results[6]++;
					results[0]--;
				}
				OK = false;
		}
					
		// Check if the configuration is OK
		if(it_path->header.seq > 0 && it_path->header.seq <= mesh.size()) {
			xs = {	mesh[it_path->header.seq-1].poses[0].pose.position.x,
					mesh[it_path->header.seq-1].poses[1].pose.position.x,
					mesh[it_path->header.seq-1].poses[2].pose.position.x,
					it_path->pose.position.x};

			ys = {	mesh[it_path->header.seq-1].poses[0].pose.position.y,
					mesh[it_path->header.seq-1].poses[1].pose.position.y,
					mesh[it_path->header.seq-1].poses[2].pose.position.y,
					it_path->pose.position.y};
		
			zs = {	mesh[it_path->header.seq-1].poses[0].pose.position.z,
					mesh[it_path->header.seq-1].poses[1].pose.position.z,
					mesh[it_path->header.seq-1].poses[2].pose.position.z,
					it_path->pose.position.z};
		}
		else {
			xs = {it_path->pose.position.x};				
			ys = {it_path->pose.position.y};						
			zs = {it_path->pose.position.z};
		}
		
		if(!isVisibilityBoxOK(xs, ys, zs)) {
			ROS_ERROR("VP %d KO!", it_path->header.seq);
			results[2]++;
			results[0]--;
			// TODO Comment
			//ros::shutdown();
			return;
		}
		
	}
	
	if(!allVPsID.empty()) {
		for(int i=0; i<allVPsID.size(); i++) {
			if(allVPsID[i] > mesh.size())
				ROS_ERROR("Missing VP: %d", allVPsID[i]);
		}
		results[5]++;
		results[0]--;
	}
}

bool isVisibilityBoxOK(std::vector<double> xs, std::vector<double> ys, std::vector<double> zs) {
	auto minmaxX = std::minmax_element(xs.begin(), xs.end()); //*minmaxX.first = minX ; *minmaxX.second = maxX
 	auto minmaxY = std::minmax_element(ys.begin(), ys.end()); //*minmaxY.first = minY ; *minmaxY.second = maxY
	auto minmaxZ = std::minmax_element(zs.begin(), zs.end()); //*minmaxZ.first = minZ ; *minmaxZ.second = maxZ
	
	return (xmin_obs < *minmaxX.first && xmax_obs < *minmaxX.first || xmin_obs > *minmaxX.second && xmax_obs > *minmaxX.second ||
			ymin_obs < *minmaxY.first && ymax_obs < *minmaxY.first || ymin_obs > *minmaxY.second && ymax_obs > *minmaxY.second ||
			zmin_obs < *minmaxZ.first && zmax_obs < *minmaxZ.first || zmin_obs > *minmaxZ.second && zmax_obs > *minmaxZ.second);
}
