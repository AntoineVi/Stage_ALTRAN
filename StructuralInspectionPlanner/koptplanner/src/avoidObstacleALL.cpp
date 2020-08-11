/*!
 * \file avoidObstacle.cpp
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
//#include <boost/filesystem.hpp>
#include "octree.h"
#include <unordered_set>
#include <map>

//#define OCTREE_SCALING(x) (x*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0)
//#define OCTREE_SCALING(x) (x*31.0/1000.0+31.0/2.0)

#ifdef __TIMING_INFO__
	long time_DBS;
	long time_RRTS;
	long time_RRTS_req;
	long time_LKH;
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

std::string pkgPath = ros::package::getPath("koptplanner");

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

// First and last indices on path of the modified trajectory
int indPathFirstVP;

void readSTLfile(std::string name);
void publishStl();
void publishViewpoint(geometry_msgs::PoseStamped vp);
void publishPath();

void poseObstacle(const geometry_msgs::PointStamped& clicked_pose);
void planForHiddenTrangles();
void setupSystem();
void readPath();
void fillOctree(geometry_msgs::PoseStamped previousVP, geometry_msgs::PoseStamped vp);
double octreeScaling(double value);
geometry_msgs::Pose selectViewpointFromFile(std::vector<double> xs, std::vector<double> ys, std::vector<double> zs, int VP_id);
bool isTriangleVisible(std::vector<double> xs, std::vector<double> ys, std::vector<double> zs);
void changeInspectionPath();
void cleanVariables();

void checkNewTourFile();
void generateNewTourFile();

int main(int argc, char **argv)
{
	ros::init(argc, argv, "koptplanner");
	ros::NodeHandle n;

	marker_pub = n.advertise<nav_msgs::Path>("visualization_marker", 1);
	viewpoint_pub = n.advertise<visualization_msgs::Marker>("viewpoint_marker", 1);  
	obstacle_pub = n.advertise<visualization_msgs::Marker>("scenario", 1);
	stl_pub = n.advertise<nav_msgs::Path>("stl_mesh", 1);

	clickedPoint_sub = n.subscribe("clicked_point", 1, poseObstacle);

	generateNewTourFile();
	
	// Problem setup with the last calculated path from tour.txt file
	sleep_time = 0.01;
	//sleep_time = 0.0;
	obsBoxSize = 50;
	pkgPath = ros::package::getPath("koptplanner");
	g_security_distance = 5.0;
	octreeSize = 32;
	
	readSTLfile(ros::package::getPath("request")+"/meshes/asciiavion.stl");
	octree = new Octree<std::vector<int>>(octreeSize);
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
	ros::Duration(sleep_time).sleep();
	
	xmin_obs = obs->center[0]-0.5*obs->size[0]-g_security_distance;
	xmax_obs = obs->center[0]+0.5*obs->size[0]+g_security_distance;	
	ymin_obs = obs->center[1]-0.5*obs->size[1]-g_security_distance;
	ymax_obs = obs->center[1]+0.5*obs->size[1]+g_security_distance;	
	zmin_obs = obs->center[2]-0.5*obs->size[2]-g_security_distance;
	zmax_obs = obs->center[2]+0.5*obs->size[2]+g_security_distance;

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
	
	// Fill octree for the first and last viewpoints
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
	
	/* Fill the area between the viewpoint and the triangle 
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
	}*/
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
	
	if(octree == NULL) {
		octree = new Octree<std::vector<int>>(octreeSize);
		readPath();
	}
	
	reinitRRTs = NULL;
	VP = NULL;
	lookupTable = NULL;
	plannerArrayBool = false;
	g_closed_tour = true;
	res_g = new koptplanner::inspection::Response();

	// static function variable reset variables:
	Gain23Static = 1;
	GreedyTourmark = 1;
	SFCTourRank = 1;
	ReadPenaltiesStatic = 1;
	
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
	
	indPathFirstVP = 0;
}

void planForHiddenTrangles() {
#ifdef __TIMING_INFO__
		timeval time;
		gettimeofday(&time, NULL);
		millisecStart = time.tv_sec * 1000 + time.tv_usec / 1000;
		time_start = millisecStart;
#endif

	// Get ids from viewpoints which has the triangle hides by the obstacle box
	// Values in the octree E [0,octreeSize]
	for(int z=floor(octreeScaling(zmin_obs)); z<octreeSize && z<=ceil(octreeScaling(zmax_obs)); z++) {
		for(int y=floor(octreeScaling(ymin_obs)); y<octreeSize && y<=ceil(octreeScaling(ymax_obs)); y++) {
			for(int x=floor(octreeScaling(xmin_obs)); x<octreeSize && x<=ceil(octreeScaling(xmax_obs)); x++) {
				vectorChangedVP.insert(std::end(vectorChangedVP), std::begin((*octree)(x,y,z)), std::end((*octree)(x,y,z)));
			}
		}
	}
	
	// Remove duplicates
	std::unordered_set<double> setKO(std::begin(vectorChangedVP), std::end(vectorChangedVP));
	vectorChangedVP.assign(std::begin(setKO), std::end(setKO));
	
	// If a waypoint is in vectorChangedVP, then it's necessary to remove it from the path
	bool modifiedPath = false;	
	vectorChangedVP.erase(std::remove_if(vectorChangedVP.begin(), vectorChangedVP.end(),
		[&modifiedPath](int x){
			if(x > mesh.size()) {
				for(std::vector<geometry_msgs::PoseStamped>::iterator it_path = std::begin(path); it_path != std::end(path); it_path++) {
					if(it_path->header.seq == x) {
						ROS_INFO("Delete waypoint: ID:%d, x:%f, y:%f, z:%f", x, it_path->pose.position.x, it_path->pose.position.y, it_path->pose.position.z);
						path.erase(it_path);
						modifiedPath = true;
						return true;
					}
				}
			}
			return false;
		}), vectorChangedVP.end());
	
	if(vectorChangedVP.size() > 0) {
#ifdef __TIMING_INFO__
		gettimeofday(&time, NULL);
		time_READ -= time.tv_sec * 1000000 + time.tv_usec;
#endif
		modifiedPath = true;
		
		// Add the last viewpoints OK as start and stop points
		int cptVP_KO = 0;
		int nbVP_KO = vectorChangedVP.size();
		
		ROS_INFO("Number of VPs KO:%d", nbVP_KO);
		
		for(int i=0; i<path.size(); i++) {
			if(std::find(vectorChangedVP.begin(), vectorChangedVP.end(), path[i].header.seq) != vectorChangedVP.end()) {
				if(cptVP_KO == 0) {
					indPathFirstVP = i-1;
					vectorChangedVP.insert(vectorChangedVP.begin(), path[i-1].header.seq);
				}
				cptVP_KO++;
			}
			else {
				if(cptVP_KO > 0 && cptVP_KO <= nbVP_KO) {
					vectorChangedVP.insert(vectorChangedVP.begin()+cptVP_KO, path[i].header.seq);
					if(cptVP_KO == nbVP_KO) {
						break;
					}						
				}
			}		
		}
				
		maxID = vectorChangedVP.size();
		ROS_INFO("Number of VPs in the modified path:%d", maxID);

		if(VP)
			delete[] VP;
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

		if(reinitRRTs==NULL)
			reinitRRTs = new int[maxID];
		for(int q = 0; q<maxID; q++) {
			reinitRRTs[q] = 1;
		}
			
		/* ------------- TSP -------------- */
		double ** vals = new double* [maxID];
		std::vector<double> xs;
		std::vector<double> ys;
		std::vector<double> zs;
		
		for(int i = 0; i<maxID; i++) {
			vals[i] = new double[3];
				
			// /!\ Indice in indiceMesh = indicePath - 1 (indicePath add the startup point at indice 0)
			if(path[indPathFirstVP+i].header.seq > 0) {
				xs = {	mesh[path[indPathFirstVP+i].header.seq-1].poses[0].pose.position.x,
						mesh[path[indPathFirstVP+i].header.seq-1].poses[1].pose.position.x,
						mesh[path[indPathFirstVP+i].header.seq-1].poses[2].pose.position.x,
						path[indPathFirstVP+i].pose.position.x};
				
				ys = {	mesh[path[indPathFirstVP+i].header.seq-1].poses[0].pose.position.y,
						mesh[path[indPathFirstVP+i].header.seq-1].poses[1].pose.position.y,
						mesh[path[indPathFirstVP+i].header.seq-1].poses[2].pose.position.y,
						path[indPathFirstVP+i].pose.position.y};
						
				zs = {	mesh[path[indPathFirstVP+i].header.seq-1].poses[0].pose.position.z,
						mesh[path[indPathFirstVP+i].header.seq-1].poses[1].pose.position.z,
						mesh[path[indPathFirstVP+i].header.seq-1].poses[2].pose.position.z,
						path[indPathFirstVP+i].pose.position.z};
			}
			else {
				xs = {path[indPathFirstVP+i].pose.position.x};				
				ys = {path[indPathFirstVP+i].pose.position.y};						
				zs = {path[indPathFirstVP+i].pose.position.z};
			}
			
			if(isTriangleVisible(xs, ys, zs)) {
				tf::Pose pose;
				tf::poseMsgToTF(path[indPathFirstVP+i].pose, pose);
			
				VP[i][0] = path[indPathFirstVP+i].pose.position.x;
				VP[i][1] = path[indPathFirstVP+i].pose.position.y;
				VP[i][2] = path[indPathFirstVP+i].pose.position.z;
				VP[i][3] = tf::getYaw(pose.getRotation());
			}
			else {
				// Remove the current VP because the obstacle hides is triangle
				xs.pop_back();
				ys.pop_back();
				zs.pop_back();
				
				geometry_msgs::Pose newVP = selectViewpointFromFile(xs, ys, zs, path[indPathFirstVP+i].header.seq);
				tf::Pose pose;
				tf::poseMsgToTF(newVP, pose);
		
				VP[i][0] = newVP.position.x;
				VP[i][1] = newVP.position.y;
				VP[i][2] = newVP.position.z;			
				VP[i][3] = tf::getYaw(pose.getRotation());
			}
					
			vals[i][0] = VP[i][0]*g_scale;
			vals[i][1] = VP[i][1]*g_scale;
			vals[i][2] = VP[i][2]*g_scale;
		}
		ROS_INFO("Number of VPs in the modified path:%d", vectorChangedVP.size());
		
#ifdef __TIMING_INFO__
		gettimeofday(&time, NULL);
		time_READ += time.tv_sec * 1000000 + time.tv_usec;
#endif
		/* use provided interface of the TSP solver */
		std::string params = "MOVE_TYPE=5\n";
		params += "PRECISION=1\n";
		params += "PATCHING_C=3\n";
		params += "PATCHING_A=2\n";
		params += "RUNS=1\n";
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
		prob += "NODE_COORD_SECTION\n";
		prob += "EOF";
#ifdef __TIMING_INFO__
		gettimeofday(&time, NULL);
		time_LKH -= time.tv_sec * 1000000 + time.tv_usec;
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
	
#ifdef __TIMING_INFO__
		gettimeofday(&time, NULL);
		time_LKH += time.tv_sec * 1000000 + time.tv_usec;
#endif
		for(int i = 0; i<maxID; i++) {
		  delete[] vals[i];
		}
		delete[] vals;
	}
	else
		ROS_INFO("Not need to change the inspection path!");
		
	if(modifiedPath) {
#ifdef __TIMING_INFO__
		gettimeofday(&time, NULL);
		time_CHANGE_PATH -= time.tv_sec * 1000000 + time.tv_usec;
#endif
		ROS_INFO("Change the path");
		changeInspectionPath();
#ifdef __TIMING_INFO__
		gettimeofday(&time, NULL);
		time_CHANGE_PATH += time.tv_sec * 1000000 + time.tv_usec;
#endif
	}
		
#ifdef __TIMING_INFO__
	gettimeofday(&time, NULL);
	ROS_INFO("Calculation time was:\t\t\t%i ms", (int)((long)(time.tv_sec * 1000 + time.tv_usec / 1000) - millisecStart));
	ROS_INFO("Choice VPs time consumption:\t\t%i ms", (int)(time_READ/1000));
	ROS_INFO("LKH time consumption:\t\t\t%i ms", (int)(time_LKH/1000));
	ROS_INFO("Change of path time consumption:\t%i ms", (int)(time_CHANGE_PATH/1000));
	ROS_INFO("Initial RRT* time consumption:\t\t%i ms", (int)(time_RRTS/1000));
	ROS_INFO("Distance evaluation time:\t\t%i ms", (int)(time_RRTS_req/1000));
#endif
	
	checkNewTourFile();
	publishPath();
	cleanVariables();
	
	ROS_INFO("Ready to receive obstacle pose from Publish point on RVIZ");
}

bool isTriangleVisible(std::vector<double> xs, std::vector<double> ys, std::vector<double> zs) {
	auto minmaxX = std::minmax_element(xs.begin(), xs.end()); //*minmaxX.first = minX ; *minmaxX.second = maxX
 	auto minmaxY = std::minmax_element(ys.begin(), ys.end()); //*minmaxY.first = minY ; *minmaxY.second = maxY
	auto minmaxZ = std::minmax_element(zs.begin(), zs.end()); //*minmaxZ.first = minZ ; *minmaxZ.second = maxZ
	
	return (xmin_obs < *minmaxX.first && xmax_obs < *minmaxX.first || xmin_obs > *minmaxX.second && xmax_obs > *minmaxX.second ||
			ymin_obs < *minmaxY.first && ymax_obs < *minmaxY.first || ymin_obs > *minmaxY.second && ymax_obs > *minmaxY.second ||
			zmin_obs < *minmaxZ.first && zmax_obs < *minmaxZ.first || zmin_obs > *minmaxZ.second && zmax_obs > *minmaxZ.second);
}

geometry_msgs::Pose selectViewpointFromFile(std::vector<double> xs, std::vector<double> ys, std::vector<double> zs, int VP_id) {
	bool VP_OK = false;
	std::string line;
	geometry_msgs::Pose VPtmp;
	
	file.open((pkgPath+"/viewpoints/viewpoint_"+std::to_string(VP_id)+".txt").c_str());
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
			
			VP_OK = isTriangleVisible(xs, ys, zs);
			
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
			ROS_ERROR("No collision free viewpoint for triangle %d", VP_id);
			ros::shutdown();
		}
	}
	else {
		ROS_ERROR("Viewpoint %d file not found", VP_id);
		ros::shutdown();
	}
}

void changeInspectionPath() {
	bool addedPath = false;
	int waypointID = 0;
	
	// Clear old path
	file.open((pkgPath+"/data/tour.txt").c_str(), std::ios::out | std::ofstream::trunc);
	file.close();
	
	file.open((pkgPath+"/data/tour.txt").c_str(), std::ios::out | std::ios::app);
	if(file.is_open()) {
		for(std::vector<geometry_msgs::PoseStamped>::iterator it_path = std::begin(path); it_path != std::end(path); it_path++) {
			if(std::find(vectorChangedVP.begin(), vectorChangedVP.end(), it_path->header.seq) == vectorChangedVP.end()) {
				file << it_path->header.seq << "\t";
				file << std::setprecision(8) << it_path->pose.position.x << "\t"; 
				file << std::setprecision(8) << it_path->pose.position.y << "\t";
				file << std::setprecision(8) << it_path->pose.position.z << "\t";
				file << std::setprecision(8) << it_path->pose.orientation.x << "\t";
				file << std::setprecision(8) << it_path->pose.orientation.y << "\t";
				file << std::setprecision(8) << it_path->pose.orientation.z << "\t";
				file << std::setprecision(8) << it_path->pose.orientation.w << "\n";
			}
			else if(!addedPath) {
				//file << "\n";
				for(std::vector<geometry_msgs::PoseStamped>::iterator it = res_g->inspectionPath.poses.begin(); it != res_g->inspectionPath.poses.end()-1; it++) {
					if(it->pose.position.x != (it+1)->pose.position.x && it->pose.position.y != (it+1)->pose.position.y && it->pose.position.z != (it+1)->pose.position.z) {
						int i;
						for(i=0; i<maxID; i++) {
							tf::Pose pose;
							tf::poseMsgToTF(it->pose, pose);
						
							if(	std::fabs(VP[i][0] - it->pose.position.x) < threshold &&
								std::fabs(VP[i][1] - it->pose.position.y) < threshold &&
								std::fabs(VP[i][2] - it->pose.position.z) < threshold /*&&
								std::fabs(VP[i][3] - tf::getYaw(pose.getRotation())) < threshold*/) {
							
								file << path[indPathFirstVP+i].header.seq << "\t";
								file << std::setprecision(8) << it->pose.position.x << "\t";
								file << std::setprecision(8) << it->pose.position.y << "\t";
								file << std::setprecision(8) << it->pose.position.z << "\t";
								file << std::setprecision(8) << it->pose.orientation.x << "\t";
								file << std::setprecision(8) << it->pose.orientation.y << "\t";
								file << std::setprecision(8) << it->pose.orientation.z << "\t";
								file << std::setprecision(8) << it->pose.orientation.w << "\n";
								break;
							}
						 }
						 if(i == maxID) {
							ROS_INFO("Add waypoint: x:%f, y:%f, z:%f", it->pose.position.x, it->pose.position.y, it->pose.position.z);
							// LKH added waypoints to avoid the obstacle
							file << path.size()+waypointID << "\t";
							file << std::setprecision(8) << it->pose.position.x << "\t";
							file << std::setprecision(8) << it->pose.position.y << "\t";
							file << std::setprecision(8) << it->pose.position.z << "\t";
							file << std::setprecision(8) << it->pose.orientation.x << "\t";
							file << std::setprecision(8) << it->pose.orientation.y << "\t";
							file << std::setprecision(8) << it->pose.orientation.z << "\t";
							file << std::setprecision(8) << it->pose.orientation.w << "\n";
							waypointID++;
							//publishViewpoint(*it);
						}
					}
				}
				addedPath = true;
			}
		}
		file.close();
	}
	else {
		ROS_ERROR("Can't open tour.txt file");
		ros::shutdown();
	}
}

void cleanVariables() {
	ROS_INFO("Variables cleaning");
	// tidy up
	for(int i = 0; i<maxID; i++)
		delete plannerArray[i];
	delete[] plannerArray;
	delete[] reinitRRTs;
	reinitRRTs = NULL;
	
	if(!sys_t::obstacles.empty()) {
		for(typename std::list<reg_t*>::iterator it = sys_t::obstacles.begin(); it != sys_t::obstacles.end(); it++)
			delete (*it);
		sys_t::obstacles.clear();
	}
	
	if(VP) {
		delete[] VP;
		VP = NULL;
	}
	maxID = 0;
	if(lookupTable) {
		for(int i = 0; i < LOOKUPTABLE_SIZE; i++)
			delete[] lookupTable[i];
		delete[] lookupTable;
		lookupTable = NULL;
	}
	plannerArrayBool = false;
	
	for(int z=(int) zmin_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; z<=(int) zmax_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; z++) {
		for(int y=(int) ymin_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; y<=(int) ymax_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; y++) {
			for(int x=(int) xmin_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; x<=(int) xmax_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; x++) {
				vector<int>().swap((*octree)(x,y,z));
			}
		}
	}
	octree = NULL;
	vector<nav_msgs::Path>().swap(mesh); 	
	vector<int>().swap(vectorChangedVP); 
	vector<geometry_msgs::PoseStamped>().swap(path);
	
	indPathFirstVP = 0;
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
	
	path.clear();
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
	
	//double scaleVP = sqrt(SQ(problemBoundary.size[0])+SQ(problemBoundary.size[1])+SQ(problemBoundary.size[2]))/70.0;
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

void checkNewTourFile() {
	std::string line;
	std::string lineVP;
	std::fstream fileVP;
	bool OK = false;
	pkgPath = ros::package::getPath("koptplanner");

	file.open((pkgPath+"/data/tour.txt").c_str(), std::ios::in);
	if (file.is_open())	{
		while(getline(file,line)) {
			std::vector<std::string> poseStr;
			
			boost::split(poseStr, line, boost::is_any_of("\t"));
			
			if(std::atoi(poseStr[0].c_str()) <= mesh.size()) {
						
				fileVP.open((pkgPath+"/viewpoints/viewpoint_"+poseStr[0]+".txt").c_str(), std::ios::in);
				if(fileVP.is_open()) {
					while(getline(fileVP,lineVP)) {
						std::vector<std::string> poseStrVP;
						boost::split(poseStrVP, lineVP, boost::is_any_of("\t"));
					
						if( std::fabs(std::atof(poseStr[1].c_str()) - std::atof(poseStrVP[0].c_str())) < threshold &&
							std::fabs(std::atof(poseStr[2].c_str()) - std::atof(poseStrVP[1].c_str())) < threshold &&
							std::fabs(std::atof(poseStr[3].c_str()) - std::atof(poseStrVP[2].c_str())) < threshold /*&&
							std::fabs(std::atof(poseStr[4].c_str()) - std::atof(poseStrVP[3].c_str())) < threshold &&
							std::fabs(std::atof(poseStr[5].c_str()) - std::atof(poseStrVP[4].c_str())) < threshold &&
							std::fabs(std::atof(poseStr[6].c_str()) - std::atof(poseStrVP[5].c_str())) < threshold &&
							std::fabs(std::atof(poseStr[7].c_str()) - std::atof(poseStrVP[6].c_str())) < threshold*/) {
							OK = true;
							break;
						}
					}
				}
				else {
					ROS_ERROR("viewpoint file not found!");
					ros::shutdown();
				}
				fileVP.close();
			
				if(!OK) {
					ROS_ERROR("Viewpoint %s corrupted", poseStr[0].c_str());
					//generateNewTourFile();
					ros::shutdown();
				}
				OK = false;
			}
		}
	}
	else {
		ROS_ERROR("tour file not found!");	file.close();
		ros::shutdown();
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
/*
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
		file.open("src/tempTourSave.txt", std::ios::in);
		if (file.is_open())	{
			while(getline(file,line)) {
				if(	line.find("NAME") == std::string::npos && line.find("COMMENT") == std::string::npos && line.find("TYPE") == std::string::npos && line.find("DIMENSION") == std::string::npos && line.find("TOUR_SECTION") == std::string::npos && line.find("-1") == std::string::npos && line.find("EOF") == std::string::npos) {
					line = std::to_string(std::atoi(line.c_str())-1);
					
					fileVP.open((pkgPath+"/viewpoints/viewpoint_"+line+".txt").c_str(), std::ios::in);
					if(fileVP.is_open()) {
						getline(fileVP,lineVP);
						fileTour << line << "\t" << lineVP << "\n";
					}
					else {
						ROS_ERROR("viewpoint file not found!");
						ros::shutdown();
					}
					fileVP.close();
				}
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
}*/
