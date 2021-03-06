/*!
 * \file plan.cpp
 *
 * More elaborate description
 */
#include "ros/ros.h"
#include <sys/time.h>
#include <stdlib.h>
#include <fstream>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Path.h"
#include <std_msgs/Int32.h>
#include "koptplanner/plan.hpp"
#include "planner_rrts/system_holonomic.h"
#include "koptplanner/TriangleObject.h"
#include "koptplanner/Rotorcraft.h"
#include "koptplanner/FixedWing.h"
#include "koptplanner/TriangleObject.hpp"
#include "planner_rrts/rrts.hpp"
#include "planner_rrts/system_holonomic.hpp"
#include "koptplanner/Rotorcraft.hpp"
#include "koptplanner/FixedWing.hpp"
#include "koptplanner/ptpPlanner.hpp"
#include "LKH.h"
#include "koptplanner/inspection.h"
#include "planner_rrts/system_holonomic.h"
#include "optec/qpOASES.hpp"
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <shape_msgs/Plane.h>
#include <ros/package.h>

#include <boost/algorithm/string.hpp>
#include "octree.h"
#include <unordered_set>
#include <map>

#include "std_msgs/String.h"
#ifdef __TIMING_INFO__
	long time_DBS;
	long time_RRTS;
	long time_RRTS_req;
	long time_LKH;
	long time_LKH_ALL;
	long time_READ;
	long time_CHANGE_PATH;
#endif
long time_start;
long millisecStart;

USE_MODEL_NAMESPACE

// Static variables in the functions have to be reset, use bools (int) to trigger:
int Gain23Static;
int GreedyTourmark;
int SFCTourRank;
int ReadPenaltiesStatic;

extern bool plannerArrayBool;
extern PTPPlanner** plannerArray;
int * reinitRRTs;
int maxID;
reg_t problemBoundary;
StateVector * VP;

ros::Publisher marker_pub;
ros::Publisher viewpoint_pub;
koptplanner::inspection::Response* res_g;
double ** lookupTable;
double g_speed;
double g_maxAngularSpeed;
double g_camAngleHorizontal;
double g_camAngleVertical;
double g_camPitch;
double g_maxClimbSinkRate;

double g_scale;
double g_rrt_scope;
int g_rrt_it;
int g_rrt_it_init;
int g_max_obstacle_depth;
double g_discretization_step;
double g_angular_discretization_step;
double g_const_A;
double g_const_B;
double g_const_C;
double g_const_D;
bool g_lazy_obstacle_check;
double g_security_distance;
double vp_tol;

bool g_closed_tour;
double g_cost;
string g_tourlength;
std::fstream file;
koptError_t koptError;
double g_max_obs_dim;

std::string pkgPath;

int obsBoxSize;
ros::Publisher obstacle_pub;
ros::Publisher stl_pub;
double sleep_time;
ros::Subscriber clickedPoint_sub;

std::vector<nav_msgs::Path> mesh;

bool endPoint;
std::vector<double> spaceSize = {1000.0, 1000.0, 1000.0};
std::vector<double> spaceCenter = {0.0, 0.0, 0.0};

Octree<std::vector<int>>* octree = NULL;

double threshold = 0.0001;
double xmin_obs;
double xmax_obs;
double ymin_obs;
double ymax_obs;
double zmin_obs;
double zmax_obs;

std::vector<geometry_msgs::PoseStamped> path;
std::vector<int> vectorChangedVP;
int octreeSize;

ros::Publisher results_pub;
int pathMaxSize;
bool RVIZ_ON;
bool USE_evalPath;

void readSTLfile(std::string name);
void publishStl();
void publishViewpoint(geometry_msgs::PoseStamped vp);
void publishPath();

void poseObstacle(const geometry_msgs::PointStamped& clicked_pose);
int planForHiddenTrangles();
void setupSystem();
void readPath();
void fillOctree(geometry_msgs::PoseStamped previousVP, geometry_msgs::PoseStamped vp);
double octreeScaling(double value);
geometry_msgs::Pose selectViewpointFromFile(std::vector<double> xs, std::vector<double> ys, std::vector<double> zs, int pathID);
bool isVisibilityBoxOK(std::vector<double> xs, std::vector<double> ys, std::vector<double> zs);
void changeInspectionPath(int firstIDPathKO, std::vector<geometry_msgs::PoseStamped> &tmpPath);
void cleanVariables();

void savePathToFile();
void checkNewTourFile();
void generateNewTourFile();

int main(int argc, char **argv)
{
	ros::init(argc, argv, "avoidObstacle");
	ros::NodeHandle n;

	marker_pub = n.advertise<nav_msgs::Path>("visualization_marker", 1);
	viewpoint_pub = n.advertise<visualization_msgs::Marker>("viewpoint_marker", 1);  
	obstacle_pub = n.advertise<visualization_msgs::Marker>("scenario", 1);
	stl_pub = n.advertise<nav_msgs::Path>("stl_mesh", 10000);
	
	ros::Publisher marker_pubINIT = n.advertise<nav_msgs::Path>("initPATH", 1);	
	results_pub = n.advertise<std_msgs::Int32>("aO_result", 1);
	
	clickedPoint_sub = n.subscribe("clicked_point", 1, poseObstacle);

	
	// Problem setup with the last calculated path from tour.txt file
	sleep_time = 0.01;
	obsBoxSize = 5;
	pkgPath = ros::package::getPath("koptplanner");
	g_security_distance = 5.0;
	octreeSize = 64;
	octree = new Octree<std::vector<int>>(octreeSize);
	
	if (argc == 3) {
		RVIZ_ON = (bool) std::stoi(argv[1]);
		USE_evalPath = (bool) std::stoi(argv[2]);
	}
	else {
		RVIZ_ON = true;
		USE_evalPath = false;
	}
	
	ROS_INFO("RVIZ used: %d, evalPath used: %d", RVIZ_ON, USE_evalPath);
	
	generateNewTourFile();
	
	ROS_INFO("Waiting for the subscriber for the STL!");
	while(!ros::isShuttingDown() && stl_pub.getNumSubscribers() < (int) RVIZ_ON + (int) USE_evalPath);
	ROS_INFO("Subscriber found!");
	readSTLfile(ros::package::getPath("request")+"/meshes/asciiavion.stl");
	
	ROS_INFO("Waiting for the subscriber for the path!");
	while(!ros::isShuttingDown() && marker_pub.getNumSubscribers() < (int) RVIZ_ON + (int) USE_evalPath);
	ROS_INFO("Subscriber found!");
	readPath();
	pathMaxSize = path.size();
	
	/*nav_msgs::Path p;
	if(path.empty())
		readPath();	
	p.header.frame_id = "/kopt_frame";
	p.header.stamp = ros::Time::now();	
	// Start point	
	p.poses.push_back(path[0]);
	for(int i=1; i<path.size(); i++) {
		p.poses.push_back(path[i]);
		p.poses.push_back(path[i]);
	}
	marker_pubINIT.publish(p);*/
	
	publishStl();
	publishPath();
	
	ROS_INFO("Ready to receive obstacle pose from Publish point on RVIZ");
	
	ros::spin();
	
	return 0;
}

void poseObstacle(const geometry_msgs::PointStamped& clicked_pose) {
	setupSystem();
	
	/* obstacles */
	reg_t * obs = new reg_t;
	obs->setNumDimensions(3);
	obs->occupied = 0;
	obs->center[0] = clicked_pose.point.x;
	obs->center[1] = clicked_pose.point.y;
	obs->center[2] = clicked_pose.point.z;
	
	obs->size[0] = obsBoxSize;
	obs->size[1] = obsBoxSize;
	obs->size[2] = obsBoxSize;
	if(g_max_obs_dim<obs->size[0])
		g_max_obs_dim=obs->size[0];
	if(g_max_obs_dim<obs->size[1])
		g_max_obs_dim=obs->size[1];
	if(g_max_obs_dim<obs->size[2])
		g_max_obs_dim=obs->size[2];
	sys_t::obstacles.push_back(obs);
	
	// publish obstacles for rviz 
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/kopt_frame";
	marker.header.stamp = ros::Time::now();
	marker.ns = "obstacles";
	marker.id = 0; // enumerate when adding more obstacles
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = clicked_pose.point.x;
	marker.pose.position.y = clicked_pose.point.y;
	marker.pose.position.z = clicked_pose.point.z;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 1;

	marker.scale.x = obsBoxSize;
	marker.scale.y = obsBoxSize;
	marker.scale.z = obsBoxSize;

	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 0.5;

	marker.lifetime = ros::Duration();
	obstacle_pub.publish(marker);
	//ros::Duration(sleep_time).sleep();
	
	xmin_obs = obs->center[0]-0.5*obs->size[0]-g_security_distance;
	xmax_obs = obs->center[0]+0.5*obs->size[0]+g_security_distance;	
	ymin_obs = obs->center[1]-0.5*obs->size[1]-g_security_distance;
	ymax_obs = obs->center[1]+0.5*obs->size[1]+g_security_distance;	
	zmin_obs = obs->center[2]-0.5*obs->size[2]-g_security_distance;
	zmax_obs = obs->center[2]+0.5*obs->size[2]+g_security_distance;

	if(USE_evalPath)
		results_pub.publish(planForHiddenTrangles());
	else
		planForHiddenTrangles();
}

void readPath() {
	std::string line;
	
	file.open(pkgPath+"/data/tour.txt");
	if (file.is_open())	{
		while(getline(file,line)) {
			// Fill vector<VPconfig>
			geometry_msgs::PoseStamped tmpPose;			
			std::vector<std::string> poseStr;
			
			boost::split(poseStr, line, boost::is_any_of("\t"));
			
			tmpPose.header.seq = std::atoi(poseStr[0].c_str());
			tmpPose.header.stamp = ros::Time::now();
			tmpPose.header.frame_id = "/kopt_frame";
			
			tmpPose.pose.position.x = std::atof(poseStr[1].c_str());
			tmpPose.pose.position.y = std::atof(poseStr[2].c_str());
			tmpPose.pose.position.z = std::atof(poseStr[3].c_str());
			tmpPose.pose.orientation.x = std::atof(poseStr[4].c_str());
			tmpPose.pose.orientation.y = std::atof(poseStr[5].c_str());
			tmpPose.pose.orientation.z = std::atof(poseStr[6].c_str());
			tmpPose.pose.orientation.w = std::atof(poseStr[7].c_str());
			
			path.push_back(tmpPose);
			
			// Fill octree
			if(path.size() > 1)
				fillOctree(path[path.size()-2], path[path.size()-1]);
		}
	}
	else {
		ROS_ERROR("Error on path file");
		ros::shutdown();
	}
	file.close();
	
	// Fill octree for the first viewpoint
	fillOctree(path[path.size()-1], path[0]);
}

// Add id of the viewpoint in each cube between the viewpoint and the observed triangle
void fillOctree(geometry_msgs::PoseStamped previousVP, geometry_msgs::PoseStamped vp) {
	std::vector<double> xs;
	std::vector<double> ys;
	std::vector<double> zs;
	
	/* Fill the area between the viewpoint and the triangle */	
	// 0<VP_id<mesh_size  <=> Start point => No triangle to inspect
	if(vp.header.seq > 0 && vp.header.seq <= mesh.size()) {
		xs = {	mesh[vp.header.seq-1].poses[0].pose.position.x,
				mesh[vp.header.seq-1].poses[1].pose.position.x,
				mesh[vp.header.seq-1].poses[2].pose.position.x,
				vp.pose.position.x};
				
		ys = {	mesh[vp.header.seq-1].poses[0].pose.position.y,
				mesh[vp.header.seq-1].poses[1].pose.position.y,
				mesh[vp.header.seq-1].poses[2].pose.position.y,
				vp.pose.position.y};
				
		zs = {	mesh[vp.header.seq-1].poses[0].pose.position.z,
				mesh[vp.header.seq-1].poses[1].pose.position.z,
				mesh[vp.header.seq-1].poses[2].pose.position.z,
				vp.pose.position.z};
	}
	else {
		xs = {vp.pose.position.x};
		ys = {vp.pose.position.y};
		zs = {vp.pose.position.z};
	}		
	
	auto minmaxX = std::minmax_element(xs.begin(), xs.end());
	auto minmaxY = std::minmax_element(ys.begin(), ys.end());
	auto minmaxZ = std::minmax_element(zs.begin(), zs.end());
	
	
	// Values in the octree E [0,octreeSize]
	for(int z=floor(octreeScaling((double)*minmaxZ.first)); z<=ceil(octreeScaling((double)*minmaxZ.second)); z++) {
		for(int y=floor(octreeScaling((double)*minmaxY.first)); y<=ceil(octreeScaling((double)*minmaxY.second)); y++) {
			for(int x=floor(octreeScaling((double)*minmaxX.first)); x<=ceil(octreeScaling((double)*minmaxX.second)); x++) {
				(*octree)(x,y,z).push_back(vp.header.seq);
			}
		}
	}
	
	/* Fill the area between the viewpoint and the previous one */
	xs.clear();
	xs = {previousVP.pose.position.x, vp.pose.position.x};
	ys.clear();
	ys = {previousVP.pose.position.y, vp.pose.position.y};
	zs.clear();
	zs = {previousVP.pose.position.z, vp.pose.position.z};

	minmaxX = std::minmax_element(xs.begin(), xs.end());
	minmaxY = std::minmax_element(ys.begin(), ys.end());
	minmaxZ = std::minmax_element(zs.begin(), zs.end());
	
	
	// Values in the octree E [0,octreeSize]
	for(int z=floor(octreeScaling((double)*minmaxZ.first)); z<=ceil(octreeScaling((double)*minmaxZ.second)); z++) {
		for(int y=floor(octreeScaling((double)*minmaxY.first)); y<=ceil(octreeScaling((double)*minmaxY.second)); y++) {
			for(int x=floor(octreeScaling((double)*minmaxX.first)); x<=ceil(octreeScaling((double)*minmaxX.second)); x++) {
				(*octree)(x,y,z).push_back(previousVP.header.seq);
				(*octree)(x,y,z).push_back(vp.header.seq);
			}
		}
	}
}

double octreeScaling(double value) {
	return value*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0;
}

void setupSystem() {
	ROS_INFO("Initialisation");
	
	// Problem setup with the last calculated path from tour.txt file
	// TODO
	/*sleep_time = 0.01;
	//sleep_time = 0.0;
	obsBoxSize = 50;
	pkgPath = ros::package::getPath("koptplanner");
	g_security_distance = 5.0;
	octreeSize = 32;
	scaleVP = sqrt(SQ(problemBoundary.size[0])+SQ(problemBoundary.size[1])+SQ(problemBoundary.size[2]))/70.0;
	scaleVP/=1;	*/	
	
	if(mesh.empty())
		readSTLfile(ros::package::getPath("request")+"/meshes/asciiavion.stl");
	
	if(path.empty())
		readPath();
	
	reinitRRTs = NULL;
	VP = NULL;
	lookupTable = NULL;
	plannerArrayBool = false;
	g_closed_tour = false;
	res_g = new koptplanner::inspection::Response();
	
	/* loading the parameters */
#ifdef USE_FIXEDWING_MODEL
	if(!ros::param::get("fixedwing/speed", g_speed))
		koptError = MISSING_PARAMETER;
	if(!ros::param::get("fixedwing/Rmin", sys_t::r_min))
		koptError = MISSING_PARAMETER;
	if(!ros::param::get("fixedwing/maxClimbSinkRate", g_maxClimbSinkRate))
		koptError = MISSING_PARAMETER;
#else
	if(!ros::param::get("rotorcraft/maxSpeed", g_speed))
		koptError = MISSING_PARAMETER;
	if(!ros::param::get("rotorcraft/maxAngularSpeed", g_maxAngularSpeed))
		koptError = MISSING_PARAMETER;
#endif
	if(!ros::param::get("camera/horizontal", g_camAngleHorizontal))
		koptError = MISSING_PARAMETER;
	if(!ros::param::get("camera/vertical", g_camAngleVertical))
		koptError = MISSING_PARAMETER;
	if(!ros::param::get("camera/pitch", g_camPitch))
		koptError = MISSING_PARAMETER;
	if(koptError == MISSING_PARAMETER) {
		ROS_ERROR("Missing parameter.");
		std::fstream plannerLog;
		plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
		if(!plannerLog.is_open())
			ROS_ERROR("Could not open report.log");
		plannerLog << "-->Missing parameter. Error ocurred while loading parameters from parameter file 'koptParam.yaml'.\n";
		plannerLog.close();
		return;
	}
	/* loading the optional parameters */
#ifdef USE_FIXEDWING_MODEL
	vp_tol = 2.0e-6*sqrt(SQ(spaceSize[0])+SQ(spaceSize[1])+SQ(spaceSize[2]));
	g_rrt_scope = 0.25*sqrt(SQ(spaceSize[0])+SQ(spaceSize[1])+SQ(spaceSize[2]));
	g_rrt_it = 100;
	g_rrt_it_init = 0;
	g_max_obstacle_depth = 3;
	g_discretization_step = 1.0e-2*sqrt(SQ(spaceSize[0])+SQ(spaceSize[1]));
	g_angular_discretization_step = 0.2;
	g_const_A = 1.0e6;
	g_const_B = 3.0;
	g_const_C = 1.0e12;
	g_const_D = 1.0;
	g_lazy_obstacle_check = false;
	g_security_distance = sys_t::r_min;
#elif defined USE_ROTORCRAFT_MODEL
	vp_tol = 2.0e-6*sqrt(SQ(spaceSize[0])+SQ(spaceSize[1])+SQ(spaceSize[2]));
	g_rrt_scope = 0.125*sqrt(SQ(spaceSize[0])+SQ(spaceSize[1])+SQ(spaceSize[2]));
	g_rrt_it = 50;
	g_rrt_it_init = 0;
	g_max_obstacle_depth = 3;
	g_discretization_step = 5.0e-3*sqrt(SQ(spaceSize[0])+SQ(spaceSize[1]));
	g_angular_discretization_step = 0.2;
	g_const_A = 1.0; // NA
	g_const_B = 1.0; // NA
	g_const_C = 1.0; // NA
	g_const_D = 1.0;
	g_lazy_obstacle_check = false;
	g_security_distance = 5.0;
#else
	a_model_has_to_be_defined;
#endif
	ros::param::get("algorithm/vp_tol", vp_tol);
	ros::param::get("algorithm/rrt_scope", g_rrt_scope);
	ros::param::get("algorithm/rrt_it", g_rrt_it);
	ros::param::get("algorithm/rrt_it_init", g_rrt_it_init);
	ros::param::get("algorithm/max_obstacle_depth", g_max_obstacle_depth);
	ros::param::get("algorithm/discretization_step", g_discretization_step);
	ros::param::get("algorithm/angular_discretization_step", g_angular_discretization_step);
	ros::param::get("algorithm/const_A", g_const_A);
	g_const_A /= g_maxClimbSinkRate;
	ros::param::get("algorithm/const_B", g_const_B);
	ros::param::get("algorithm/const_C", g_const_C);
	ros::param::get("algorithm/const_D", g_const_D);
	ros::param::get("algorithm/lazy_obstacle_check", g_lazy_obstacle_check);
	ros::param::get("algorithm/security_distance", g_security_distance);
  
#ifdef USE_FIXEDWING_MODEL
	g_scale = 1.0e5/sqrt(SQ(spaceSize[0])+SQ(spaceSize[1])+SQ(spaceSize[2]));
#else
	g_scale = g_speed*1.0e5/sqrt(SQ(spaceSize[0])+SQ(spaceSize[1])+SQ(spaceSize[2]));
#endif
  
  	/* initialize problem setup */
	g_cost = DBL_MAX;
#ifdef __TIMING_INFO__
	time_DBS = 0;
	time_RRTS = 0;
	time_RRTS_req = 0;
	time_LKH = 0;
	time_LKH_ALL = 0;
	time_READ = 0;
	time_CHANGE_PATH = 0;
#endif
	time_start = 0;
	
	problemBoundary.setNumDimensions(3);
	assert(spaceCenter.size() == 3 && spaceSize.size());
	for(int i=0; i<3;i++) {
		problemBoundary.size[i] = spaceSize[i];
		problemBoundary.center[i] = spaceCenter[i];
	}
	
	if(lookupTable)	{
		for(int i = 0; i < LOOKUPTABLE_SIZE; i++)
			delete[] lookupTable[i];
		delete[] lookupTable;
		lookupTable = NULL;
	}
	
	g_tourlength = pkgPath + "/data/tourlength.m";
	file.open(g_tourlength.c_str(), std::ios::out);
	if(file.is_open()) {
		file << "length=[";
		file.close();
	}
	else {
		ROS_WARN("tourlength file not found");
	}
}

int planForHiddenTrangles() {
#ifdef __TIMING_INFO__
	timeval time;
	gettimeofday(&time, NULL);
	millisecStart = time.tv_sec * 1000 + time.tv_usec / 1000;
	time_start = millisecStart;
#endif

	// Get ids from viewpoints which has the triangle hides by the obstacle box
	// Values in the octree E [0,octreeSize]
	for(int z=floor(octreeScaling(zmin_obs)); z>0 && z<octreeSize && z<=ceil(octreeScaling(zmax_obs)); z++) {
		for(int y=floor(octreeScaling(ymin_obs)); y>0 && y<octreeSize && y<=ceil(octreeScaling(ymax_obs)); y++) {
			for(int x=floor(octreeScaling(xmin_obs)); x>0 && x<octreeSize && x<=ceil(octreeScaling(xmax_obs)); x++) {
				vectorChangedVP.insert(std::end(vectorChangedVP), std::begin((*octree)(x,y,z)), std::end((*octree)(x,y,z)));
			}
		}
	}
	
	// Remove duplicates
	std::unordered_set<double> setKO(std::begin(vectorChangedVP), std::end(vectorChangedVP));
	vectorChangedVP.assign(std::begin(setKO), std::end(setKO));
	
	// If a waypoint is in vectorChangedVP, then it's necessary to remove it from the path
	bool modifiedPath = false;	
	for(std::vector<int>::iterator it = vectorChangedVP.begin(); it != vectorChangedVP.end(); it++) {
		if(*it > mesh.size()) {
			for(std::vector<geometry_msgs::PoseStamped>::iterator it_path = std::begin(path); it_path != std::end(path); it_path++) {
				if(it_path->header.seq == *it) {
					ROS_INFO("Delete waypoint: ID:%d, x:%f, y:%f, z:%f", *it, it_path->pose.position.x, it_path->pose.position.y, it_path->pose.position.z);
					path.erase(it_path);
					vectorChangedVP.erase(it);
					it--;
					modifiedPath = true;
					break;
				}
			}
		}
	}
			
	if(vectorChangedVP.size() > 0) {
		modifiedPath = true;
		
		// Add the last viewpoints OK as start and stop points of the LKH
		int cptVP_KO = 0;
		std::vector<std::pair<int,int>> indicesPathKO;
		std::pair<int,int> boundsPathKO;
				
		ROS_INFO("Number of VPs KO:%d", (int) vectorChangedVP.size());
		
		for(int i=1; i<path.size(); i++) {
			if(std::find(vectorChangedVP.begin(), vectorChangedVP.end(), path[i].header.seq) != vectorChangedVP.end()) {
				// Case when the last VP in the path is KO: if it is alone, the 3 last VPs are used for the LKH (which needs more than 2 VPs)
				if(i == path.size()-1) {
					if(cptVP_KO == 0) {
						// While the previous point OK is a waypoint, it is removed from the path
						while(path[i-2].header.seq > mesh.size()) {
							path.erase(path.begin()+i-2);
							i--;
						}
						boundsPathKO.first = i-2;
					}
					
					// While the next point OK is a waypoint, it is removed from the path
					while(path[i].header.seq > mesh.size()) {
						path.erase(path.begin()+i);
					}
					boundsPathKO.second = i;
					indicesPathKO.push_back(boundsPathKO);
					vectorChangedVP.push_back(path[i].header.seq);
					cptVP_KO = 0;
				}
				else if(cptVP_KO == 0) {
					// While the previous point OK is a waypoint, it is removed from the path
					while(path[i-1].header.seq > mesh.size()) {
						path.erase(path.begin()+i-1);
						i--;
					}
					boundsPathKO.first = i-1;
					vectorChangedVP.push_back(path[i-1].header.seq);
				}
				cptVP_KO++;
			}
			else if(cptVP_KO > 0) {
				// While the next point OK is a waypoint, it is removed from the path
				while(path[i].header.seq > mesh.size()) {
					path.erase(path.begin()+i);
				}
				boundsPathKO.second = i;
				indicesPathKO.push_back(boundsPathKO);
				vectorChangedVP.push_back(path[i].header.seq);
				cptVP_KO = 0;
			}	
		}
		
		ROS_INFO("Number of subpaths:%d", (int) indicesPathKO.size());
		
		//tmpPath used to change the path vector during the multiples LKH to avoid to change the indices because of the waypoints
		std::vector<geometry_msgs::PoseStamped> tmpPath(path);
			
		for(int j=indicesPathKO.size()-1; j>-1; j--) {
			maxID = indicesPathKO[j].second - indicesPathKO[j].first+1;
			ROS_INFO("maxID:%d", maxID);
						
#ifdef USE_FIXEDWING_MODEL
			VP = new StateVector[2*maxID];
			/* load lookup table */
			std::fstream lookupFile;
			lookupFile.open((pkgPath+"/lookupTable/lookupTable50x50.txt").c_str(), std::ios::in);
			lookupTable = new double*[LOOKUPTABLE_SIZE];
			for (int i = 0; i<LOOKUPTABLE_SIZE; i++) {
				lookupTable[i] = new double[LOOKUPTABLE_SIZE];
				for (int j = 0; j<LOOKUPTABLE_SIZE; j++) {
					lookupFile >> lookupTable[i][j];
				}
			}
#else
			VP = new StateVector[maxID];
#endif
			/* ------------- TSP -------------- */
			std::vector<double> xs;
			std::vector<double> ys;
			std::vector<double> zs;

#ifdef __TIMING_INFO__
			gettimeofday(&time, NULL);
			time_READ -= time.tv_sec * 1000000 + time.tv_usec;
#endif			
			for(int i = 0; i<maxID; i++) {				
				
				if(path[indicesPathKO[j].first+i].header.seq > mesh.size()) {
					ROS_ERROR("Waypoint KO in the LKH");
					cleanVariables();
					return 4;
				}
				else {
					// /!\ indiceMesh = indicePath - 1 (indicePath add the startup point at indice 0)
					if(path[indicesPathKO[j].first+i].header.seq > 0) {
						xs = {	mesh[path[indicesPathKO[j].first+i].header.seq-1].poses[0].pose.position.x,
								mesh[path[indicesPathKO[j].first+i].header.seq-1].poses[1].pose.position.x,
								mesh[path[indicesPathKO[j].first+i].header.seq-1].poses[2].pose.position.x,
								path[indicesPathKO[j].first+i].pose.position.x};
			
						ys = {	mesh[path[indicesPathKO[j].first+i].header.seq-1].poses[0].pose.position.y,
								mesh[path[indicesPathKO[j].first+i].header.seq-1].poses[1].pose.position.y,
								mesh[path[indicesPathKO[j].first+i].header.seq-1].poses[2].pose.position.y,
								path[indicesPathKO[j].first+i].pose.position.y};
					
						zs = {	mesh[path[indicesPathKO[j].first+i].header.seq-1].poses[0].pose.position.z,
								mesh[path[indicesPathKO[j].first+i].header.seq-1].poses[1].pose.position.z,
								mesh[path[indicesPathKO[j].first+i].header.seq-1].poses[2].pose.position.z,
								path[indicesPathKO[j].first+i].pose.position.z};
					}
					else {
						xs = {path[indicesPathKO[j].first+i].pose.position.x};				
						ys = {path[indicesPathKO[j].first+i].pose.position.y};						
						zs = {path[indicesPathKO[j].first+i].pose.position.z};
					}
				
					if(isVisibilityBoxOK(xs, ys, zs)) {
						tf::Pose pose;
						tf::poseMsgToTF(path[indicesPathKO[j].first+i].pose, pose);
		
						VP[i][0] = path[indicesPathKO[j].first+i].pose.position.x;
						VP[i][1] = path[indicesPathKO[j].first+i].pose.position.y;
						VP[i][2] = path[indicesPathKO[j].first+i].pose.position.z;
						VP[i][3] = tf::getYaw(pose.getRotation());
					}
					else if(path[indicesPathKO[j].first+i].header.seq > 0) {
						// Remove the current VP because the obstacle hides is triangle
						xs.pop_back();
						ys.pop_back();
						zs.pop_back();
			
						geometry_msgs::Pose newVP = selectViewpointFromFile(xs, ys, zs, indicesPathKO[j].first+i);
						if(newVP.orientation.x == -1 && newVP.orientation.y == -1 && newVP.orientation.z == -1 && newVP.orientation.w == -1)
							return 1; // No configuration for the triangle
						else {
							tf::Pose pose;
							tf::poseMsgToTF(newVP, pose);
	
							VP[i][0] = newVP.position.x;
							VP[i][1] = newVP.position.y;
							VP[i][2] = newVP.position.z;			
							VP[i][3] = tf::getYaw(pose.getRotation());
						}
					}
					else {
						ROS_ERROR("Start point KO");
						cleanVariables();
						return 3;
					}
				}
				//ROS_INFO("VP: ID:%d, x:%f, y:%f, z:%f", path[indicesPathKO[j].first+i].header.seq, VP[i][0], VP[i][1], VP[i][2]);
			}
	
#ifdef __TIMING_INFO__
			gettimeofday(&time, NULL);
			time_READ += time.tv_sec * 1000000 + time.tv_usec;
#endif
			// static function variable reset variables:
			Gain23Static = 0;
			GreedyTourmark = 0;
			SFCTourRank = 0;
			ReadPenaltiesStatic = 0;
		
			double ** vals = new double* [maxID];
		
			if(reinitRRTs==NULL)
				reinitRRTs = new int[maxID];
			for(int i = 0; i<maxID; i++) {
				reinitRRTs[i] = 1;
				vals[i] = new double[3];
				vals[i][0] = VP[i][0]*g_scale;
				vals[i][1] = VP[i][1]*g_scale;
				vals[i][2] = VP[i][2]*g_scale;
			}
		
			/* use provided interface of the TSP solver */
			std::string params = "MOVE_TYPE=5\n";
			params += "GAIN23=NO\n";
			params += "PRECISION=1\n";
			params += "PATCHING_C=3\n";
			params += "PATCHING_A=2\n";
			//params += "RUNS=1\n";
			params += "TIME_LIMIT=5\n";
			params += "TRACE_LEVEL=0\n";
			params += "OUTPUT_TOUR_FILE="+pkgPath+"/data/tempTour.txt\n";
			params += "EOF";

			std::string prob = "NAME:inspection\n";
#ifdef USE_FIXEDWING_MODEL
			prob += "TYPE:ATSP\n";
			prob += "EDGE_WEIGHT_FORMAT:FULL_MATRIX\n";
			prob += "EDGE_WEIGHT_TYPE:RRTFIXEDWING_3D\n";
#else
			prob += "TYPE:TSP\n";
			prob += "EDGE_WEIGHT_TYPE:RRT_3D\n";
#endif
			std::stringstream ss; ss<<maxID;
			prob += "DIMENSION:"+ss.str()+"\n";
			prob += "FIXED_EDGES_SECTION\n"+ss.str()+" 1\n-1\n";
			prob += "NODE_COORD_SECTION\n";
			prob += "EOF";
			
#ifdef __TIMING_INFO__
			gettimeofday(&time, NULL);
			time_LKH -= time.tv_sec * 1000000 + time.tv_usec;
			time_LKH_ALL -= time.tv_sec * 1000000 + time.tv_usec;
#endif
			size_t length = params.length();
			char * par;
			assert(par = (char*) malloc(length+10));
			strcpy(par, params.c_str());
			length = prob.length();
			char * pro;
			assert(pro = (char*) malloc(length+10));
			strcpy(pro, prob.c_str());

			/* call TSP solver */
			ROS_INFO("Start LKH");
			LKHmainFunction(maxID,vals,par,pro);
		
			while(!ros::isShuttingDown() && res_g->inspectionPath.poses.empty());
			
#ifdef __TIMING_INFO__
			gettimeofday(&time, NULL);
			time_LKH += time.tv_sec * 1000000 + time.tv_usec;
			time_LKH_ALL += time.tv_sec * 1000000 + time.tv_usec;
#endif
			for(int i = 0; i<maxID; i++) {
				delete[] vals[i];
				delete plannerArray[i];
			}
			delete[] vals;
			vals = NULL;
			delete[] plannerArray;
			plannerArray = NULL;
			delete[] reinitRRTs;
			reinitRRTs = NULL;				
			plannerArrayBool = false;
			/* initialize problem setup */
			g_cost = DBL_MAX;
#ifdef __TIMING_INFO__
			time_DBS = 0;
			time_RRTS = 0;
			time_RRTS_req = 0;
			time_LKH = 0;
#endif
			time_start = 0;
			delete par;
			delete pro;

#ifdef __TIMING_INFO__
			gettimeofday(&time, NULL);
			time_CHANGE_PATH -= time.tv_sec * 1000000 + time.tv_usec;
#endif
			ROS_INFO("CHANGE PATH");
			changeInspectionPath(indicesPathKO[j].first, tmpPath);	
	
#ifdef __TIMING_INFO__
			gettimeofday(&time, NULL);
			time_CHANGE_PATH += time.tv_sec * 1000000 + time.tv_usec;
#endif
			delete[] VP;
			VP = NULL;
			res_g->inspectionPath.poses.clear();
		}
		path = tmpPath;
		//std::vector<geometry_msgs::PoseStamped>().swap(tmpPath);
		//std::vector<std::pair<int,int>>().swap(indicesPathKO);
	}
	else
		ROS_INFO("Not need to change the inspection path!");
		
	if(modifiedPath && !USE_evalPath) {
		savePathToFile();
	}
		
#ifdef __TIMING_INFO__
	gettimeofday(&time, NULL);
	ROS_INFO("Calculation time was:\t\t\t%i ms", (int)((long)(time.tv_sec * 1000 + time.tv_usec / 1000) - millisecStart));
	ROS_INFO("Choice VPs time consumption:\t\t%i ms", (int)(time_READ/1000));
	ROS_INFO("LKH time consumption:\t\t\t%i ms", (int)(time_LKH_ALL/1000));
	ROS_INFO("Change of path time consumption:\t%i ms", (int)(time_CHANGE_PATH/1000));
	ROS_INFO("Initial RRT* time consumption:\t\t%i ms", (int)(time_RRTS/1000));
	ROS_INFO("Distance evaluation time:\t\t%i ms", (int)(time_RRTS_req/1000));
#endif	

	publishPath();
	cleanVariables();
	
	return 0;
}

bool isVisibilityBoxOK(std::vector<double> xs, std::vector<double> ys, std::vector<double> zs) {
	auto minmaxX = std::minmax_element(xs.begin(), xs.end()); //*minmaxX.first = minX ; *minmaxX.second = maxX
 	auto minmaxY = std::minmax_element(ys.begin(), ys.end()); //*minmaxY.first = minY ; *minmaxY.second = maxY
	auto minmaxZ = std::minmax_element(zs.begin(), zs.end()); //*minmaxZ.first = minZ ; *minmaxZ.second = maxZ
	
	return (xmin_obs < *minmaxX.first && xmax_obs < *minmaxX.first || xmin_obs > *minmaxX.second && xmax_obs > *minmaxX.second ||
			ymin_obs < *minmaxY.first && ymax_obs < *minmaxY.first || ymin_obs > *minmaxY.second && ymax_obs > *minmaxY.second ||
			zmin_obs < *minmaxZ.first && zmax_obs < *minmaxZ.first || zmin_obs > *minmaxZ.second && zmax_obs > *minmaxZ.second);
}

geometry_msgs::Pose selectViewpointFromFile(std::vector<double> xs, std::vector<double> ys, std::vector<double> zs, int pathID) {
	bool VP_OK = false;
	std::string line;
	geometry_msgs::Pose VPtmp;

	
	file.open((pkgPath+"/viewpoints/viewpoint_"+std::to_string(path[pathID].header.seq)+".txt").c_str(), std::ios::in);
	if (file.is_open())	{
		while(!VP_OK && getline(file,line)) {
			std::vector<std::string> pose;
			boost::split(pose, line, boost::is_any_of("\t"));

			// StateVector for Rotorcraft = [x,y,z,yaw]
			VPtmp.position.x = std::atof(pose[0].c_str());
			VPtmp.position.y = std::atof(pose[1].c_str());
			VPtmp.position.z = std::atof(pose[2].c_str());
			VPtmp.orientation.x = std::atof(pose[3].c_str());
			VPtmp.orientation.y = std::atof(pose[4].c_str());
			VPtmp.orientation.z = std::atof(pose[5].c_str());
			VPtmp.orientation.w = std::atof(pose[6].c_str());
			
			xs.push_back(VPtmp.position.x); 
			ys.push_back(VPtmp.position.y);			
			zs.push_back(VPtmp.position.z);
			
			VP_OK = isVisibilityBoxOK(xs, ys, zs);
			
			if(!VP_OK) {
				xs.pop_back();
				ys.pop_back();
				zs.pop_back();
			}
		}
		file.close();
		
		if(VP_OK) {
			return VPtmp;
		}
		else {
			ROS_ERROR("No collision free configuration for triangle %d", path[pathID].header.seq);
			cleanVariables();
			VPtmp.orientation.x = -1;
			VPtmp.orientation.y = -1;
			VPtmp.orientation.z = -1;
			VPtmp.orientation.w = -1;
			return VPtmp;
		}
	}
	else {
		ROS_ERROR("Viewpoint %d file not found", path[pathID].header.seq);
		ros::shutdown();
	}
}

void changeInspectionPath(int firstIDPathKO, std::vector<geometry_msgs::PoseStamped> &tmpPath) {
	// Temporary save of the new IDs order in the subpath KO
	std::vector<int> VPnewIDs;
	int indPath = firstIDPathKO;
	
	std::vector<geometry_msgs::PoseStamped> waypoints;
	
	for(std::vector<geometry_msgs::PoseStamped>::iterator it = res_g->inspectionPath.poses.begin(); it != res_g->inspectionPath.poses.end(); it++) {
		if(it->pose.position.x != (it+1)->pose.position.x && it->pose.position.y != (it+1)->pose.position.y && it->pose.position.z != (it+1)->pose.position.z) {
			int j;
			for(j=0; j<maxID; j++) {
				tf::Pose pose;
				tf::poseMsgToTF(it->pose, pose);

				if(	std::fabs(VP[j][0] - it->pose.position.x) < threshold &&
					std::fabs(VP[j][1] - it->pose.position.y) < threshold &&
					std::fabs(VP[j][2] - it->pose.position.z) < threshold /*&&
					std::fabs(VP[j][3] - tf::getYaw(pose.getRotation())) < threshold*/) {

					// Change the path configuration for the VP
					VPnewIDs.push_back(tmpPath[firstIDPathKO+j].header.seq);
					tmpPath[indPath].pose.position.x = it->pose.position.x;
					tmpPath[indPath].pose.position.y = it->pose.position.y;
					tmpPath[indPath].pose.position.z = it->pose.position.z;
					tmpPath[indPath].pose.orientation.x = it->pose.orientation.x;
					tmpPath[indPath].pose.orientation.y = it->pose.orientation.y;
					tmpPath[indPath].pose.orientation.z = it->pose.orientation.z;
					tmpPath[indPath].pose.orientation.w = it->pose.orientation.w;
					indPath++;
					break;
				}
			}
			if(j == maxID) {
				ROS_INFO("Add waypoint: x:%f, y:%f, z:%f", it->pose.position.x, it->pose.position.y, it->pose.position.z);
				// LKH may add waypoints to avoid the obstacle
				VPnewIDs.push_back(pathMaxSize);
				pathMaxSize++;
				//publishViewpoint(*it);
				waypoints.push_back(*it);
			}
		}
	}
	
	// Change the IDs. Not done before to keep the match with VP[]
	int oldPathSize = tmpPath.size();
		
	for(int i=0; i<VPnewIDs.size(); i++) {
		if(VPnewIDs[i] >= oldPathSize) {
			tmpPath.insert(tmpPath.begin()+firstIDPathKO+i, waypoints[0]);
			tmpPath[firstIDPathKO+i].header.seq = VPnewIDs[i];
			waypoints.erase(waypoints.begin());
		}
		else
			tmpPath[firstIDPathKO+i].header.seq = VPnewIDs[i];
	}
	
	//std::vector<int>().swap(VPnewIDs);
	//std::vector<geometry_msgs::PoseStamped>().swap(waypoints);
}

void cleanVariables() {
	ROS_INFO("Variables cleaning");
	if(!sys_t::obstacles.empty()) {
		for(typename std::list<reg_t*>::iterator it = sys_t::obstacles.begin(); it != sys_t::obstacles.end(); it++)
			delete (*it);
		sys_t::obstacles.clear();
	}
		
	maxID = 0;
	
	if(lookupTable) {
		for(int i = 0; i < LOOKUPTABLE_SIZE; i++)
			delete[] lookupTable[i];
		delete[] lookupTable;
		lookupTable = NULL;
	}	
	
	for(int z=(int) zmin_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; z<=(int) zmax_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; z++) {
		for(int y=(int) ymin_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; y<=(int) ymax_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; y++) {
			for(int x=(int) xmin_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; x<=(int) xmax_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; x++) {
				vector<int>().swap((*octree)(x,y,z));
			}
		}
	}
	//delete octree;
	std::vector<nav_msgs::Path>().swap(mesh); 	
	std::vector<int>().swap(vectorChangedVP); 
	std::vector<geometry_msgs::PoseStamped>().swap(path);
		
	ROS_INFO("Ready to receive obstacle pose from Publish point on RVIZ");
}

void readSTLfile(std::string name) {
	std::fstream f;
	f.open(name.c_str());
	assert(f.is_open());
	int MaxLine = 0;
	char* line;
	double maxX = -DBL_MAX;
	double maxY = -DBL_MAX;
	double maxZ = -DBL_MAX;
	double minX = DBL_MAX;
	double minY = DBL_MAX;
	double minZ = DBL_MAX;
	assert(line = (char *) malloc(MaxLine = 80));
	f.getline(line, MaxLine);
	if(0 != strcmp(strtok(line, " "), "solid")) {
		ROS_ERROR("Invalid mesh file! Make sure the file is given in ascii-format.");
		ros::shutdown();
	}
	assert(line = (char *) realloc(line, MaxLine));
	f.getline(line, MaxLine);
	int k = 0;
	while(0 != strcmp(strtok(line, " "), "endsolid") && !ros::isShuttingDown()) {
		int q = 0;
		nav_msgs::Path p;
		geometry_msgs::PoseStamped v1;
		for(int i = 0; i<7; i++)
		{
			while(line[q] == ' ')
				q++;
			if(line[q] == 'v')
			{
				const double yawTrafo = 0.0;      // used to rotate the mesh before processing
				const double scaleFactor = 1.0;   // used to scale the mesh before processing
				const double offsetX = 0.0;       // used to offset the mesh before processing
				const double offsetY = 0.0;       // used to offset the mesh before processing
				const double offsetZ = 0.0;       // used to offset the mesh before processing

				geometry_msgs::PoseStamped vert;
				char* v = strtok(line+q," ");
				v = strtok(NULL," ");
				double xtmp = atof(v)/scaleFactor;
				v = strtok(NULL," ");
				double ytmp = atof(v)/scaleFactor;
				vert.pose.position.x = cos(yawTrafo)*xtmp-sin(yawTrafo)*ytmp;
				vert.pose.position.y =  sin(yawTrafo)*xtmp+cos(yawTrafo)*ytmp;
				v = strtok(NULL," ");
				vert.pose.position.z =  atof(v)/scaleFactor;
				vert.pose.position.x -= offsetX;
				vert.pose.position.y -= offsetY;
				vert.pose.position.z -= offsetZ;
				if(maxX<vert.pose.position.x)
					maxX=vert.pose.position.x;
				if(maxY<vert.pose.position.y)
					maxY=vert.pose.position.y;
				if(maxZ<vert.pose.position.z)
					maxZ=vert.pose.position.z;
				if(minX>vert.pose.position.x)
					minX=vert.pose.position.x;
				if(minY>vert.pose.position.y)
					minY=vert.pose.position.y;
				if(minZ>vert.pose.position.z)
					minZ=vert.pose.position.z;
				vert.pose.orientation.x =  0.0;
				vert.pose.orientation.y =  0.0;
				vert.pose.orientation.z =  0.0;
				vert.pose.orientation.w =  1.0;
				p.poses.push_back(vert);
				if(p.poses.size() == 1)
					v1 = vert;
			}
			assert(line = (char *) realloc(line, MaxLine));
			f.getline(line, MaxLine);
		}
		p.poses.push_back(v1);
		p.header.frame_id = "/kopt_frame";
		p.header.stamp = ros::Time::now();
		p.header.seq = k;
		mesh.push_back(p);
		k++;
	}
	free(line);
	f.close();
	ROS_INFO("Mesh area is bounded by: [%2.2f,%2.2f]x[%2.2f,%2.2f]x[%2.2f,%2.2f]", minX,maxX,minY,maxY,minZ,maxZ);
}


// Publishers
void publishStl() {
	ros::Rate r(50.0);
	
	/* publish STL file to rviz */
	for(std::vector<nav_msgs::Path>::iterator it = mesh.begin(); it != mesh.end() && ros::ok(); it++) {
		stl_pub.publish(*it);
		r.sleep();
	}
}

void publishPath() {
	nav_msgs::Path pathToPub;
	
	if(path.empty())
		readPath();	
	
	pathToPub.header.frame_id = "/kopt_frame";
	pathToPub.header.stamp = ros::Time::now();	
	
	// Start point	
	pathToPub.poses.push_back(path[0]);
		
	for(int i=1; i<path.size(); i++) {
		pathToPub.poses.push_back(path[i]);
		pathToPub.poses.push_back(path[i]);
		
		// Publish viewpoint
		StateVector stateVP;
		tf::Pose pose;
		tf::poseMsgToTF(path[i].pose, pose);
		
		stateVP[0] = path[i].pose.position.x;
		stateVP[1] = path[i].pose.position.y;
		stateVP[2] = path[i].pose.position.z;
		stateVP[3] = tf::getYaw(pose.getRotation());
		//publishViewpoint(path[i]);
	}
		
	marker_pub.publish(pathToPub);
}

void publishViewpoint(geometry_msgs::PoseStamped vp) {
	/* display sampled viewpoint in rviz */
	visualization_msgs::Marker point;
	point.header.frame_id = "/kopt_frame";
	point.header.stamp = ros::Time::now();
	point.id = vp.header.seq;
	point.ns = "Viewpoints";
	point.type = visualization_msgs::Marker::ARROW;
	point.pose.position.x = vp.pose.position.x;
	point.pose.position.y = vp.pose.position.y;
	point.pose.position.z = vp.pose.position.z;

	point.pose.orientation.x = vp.pose.orientation.x;
	point.pose.orientation.y = vp.pose.orientation.y;
	point.pose.orientation.z = vp.pose.orientation.z;
	point.pose.orientation.w = vp.pose.orientation.w;
	
	double scaleVP = sqrt(SQ(spaceSize[0])+SQ(spaceSize[1])+SQ(spaceSize[2]))/70.0;
	scaleVP/=1;	
	
	point.scale.x = scaleVP;
	point.scale.y = scaleVP/25.0;
	point.scale.z = scaleVP/25.0;
	point.color.r = 0.8f;
	point.color.g = 0.5f;
	point.color.b = 0.0f;
	point.color.a = 0.7;
	point.lifetime = ros::Duration();
	viewpoint_pub.publish(point);

	ros::Duration(sleep_time).sleep();
}

void savePathToFile() {
	// Clear old path
	file.open((pkgPath+"/data/tour.txt").c_str(), std::ios::out | std::ofstream::trunc);
	file.close();
	
	file.open((pkgPath+"/data/tour.txt").c_str(), std::ios::out | std::ios::app);
	if(file.is_open()) {
		for(std::vector<geometry_msgs::PoseStamped>::iterator it = path.begin(); it != path.end(); it++) {
			file << it->header.seq << "\t";
			file << std::setprecision(8) << it->pose.position.x << "\t"; 
			file << std::setprecision(8) << it->pose.position.y << "\t";
			file << std::setprecision(8) << it->pose.position.z << "\t";
			file << std::setprecision(8) << it->pose.orientation.x << "\t";
			file << std::setprecision(8) << it->pose.orientation.y << "\t";
			file << std::setprecision(8) << it->pose.orientation.z << "\t";
			file << std::setprecision(8) << it->pose.orientation.w << "\n";
		}
	}
	file.close();
}

void generateNewTourFile() {
	std::string line;
	std::string lineVP;
	std::fstream fileVP;
	std::fstream fileTour;
	pkgPath = ros::package::getPath("koptplanner");

	// Clear old path
	fileTour.open((pkgPath+"/data/tour.txt").c_str(), std::ios::out | std::ofstream::trunc);
	fileTour.close();

	fileTour.open((pkgPath+"/data/tour.txt").c_str(), std::ios::out | std::ios::app);
	if (fileTour.is_open())	{
		file.open((pkgPath+"/data/tourSAVE.txt").c_str(), std::ios::in);
		if (file.is_open())	{
			while(getline(file,line)) {
				fileTour << line << "\n";
			}		
		}
		else {
			ROS_ERROR("tempTourSave file not found!");
			ros::shutdown();
		}
		file.close();
	}
	else {
		ROS_ERROR("tour file not found!");	file.close();	
		ros::shutdown();
	}
	fileTour.close();
}
