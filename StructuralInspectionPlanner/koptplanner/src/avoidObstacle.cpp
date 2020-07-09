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
#include <boost/filesystem.hpp>
#include "octree.h"
#include <unordered_set>
#include <map>

#ifdef __TIMING_INFO__
 long time_DBS;
 long time_RRTS;
 long time_RRTS_req;
 long time_LKH;
 long time_READ;
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

std::vector<nav_msgs::Path>* mesh;

bool endPoint;
double scaleVP;
std::vector<double> spaceSize = {1000.0, 1000.0, 1000.0};
std::vector<double> spaceCenter = {0.0, 0.0, 0.0};

Octree<std::vector<int>>* octree;
double xmin_obs;
double xmax_obs;
double ymin_obs;
double ymax_obs;
double zmin_obs;
double zmax_obs;

std::vector<geometry_msgs::PoseStamped> path;
std::vector<int> listKO;
int octreeSize;

void setupSystem();
void cleanVariables();

void poseObstacle(const geometry_msgs::PointStamped& clicked_pose);

void readSTLfile(std::string name);
void publishStl();
void publishViewpoint(StateVector stateVP, int VP_id, double blue);
void publishPath();

bool inClickedObstacle(StateVector VPtmp);

bool isTriangleVisible(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax);


void debugNoVP(std::vector<int> listKO);
void publishVisibilityBox(int VP_id, double xmin, double ymin, double zmin, double xmax, double ymax, double zmax, double yaw);

void changeInspectionPath();
void createSavedTour();

void readPath();
void fillOctree(int VP_id, StateVector vp);
void selectViewpointFromFile(int VP_id, int VPKO_id);
void planForHiddenTrangles();

void generateNewTourFile();

std::vector<int> listKOComp;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "koptplanner");
	ros::NodeHandle n;

	marker_pub = n.advertise<nav_msgs::Path>("visualization_marker", 1);
	viewpoint_pub = n.advertise<visualization_msgs::Marker>("viewpoint_marker", 1);  
	obstacle_pub = n.advertise<visualization_msgs::Marker>("scenario", 1);
	stl_pub = n.advertise<nav_msgs::Path>("stl_mesh", 1);

	clickedPoint_sub = n.subscribe("clicked_point", 1, poseObstacle);

	ROS_INFO("Ready to receive obstacle pose from Publish point on RVIZ");
	
	//generateNewTourFile();
	
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
			geometry_msgs::PoseStamped poseTmp;			
			std::vector<std::string> poseStr;
			
			boost::split(poseStr, line, boost::is_any_of("\t"));
			
			poseTmp.header.seq = std::atoi(poseStr[0].c_str());
			poseTmp.header.stamp = ros::Time::now();
			poseTmp.header.frame_id = "/kopt_frame";
			
			poseTmp.pose.position.x = std::atof(poseStr[1].c_str());
			poseTmp.pose.position.y = std::atof(poseStr[2].c_str());
			poseTmp.pose.position.z = std::atof(poseStr[3].c_str());			
			tf::Quaternion q = tf::createQuaternionFromRPY(std::atof(poseStr[4].c_str()),std::atof(poseStr[5].c_str()),std::atof(poseStr[6].c_str()));
			poseTmp.pose.orientation.x = q.x();
			poseTmp.pose.orientation.y = q.y();
			poseTmp.pose.orientation.z = q.z();
			poseTmp.pose.orientation.w = q.w();
			
			path.push_back(poseTmp);			
			
			// Fill octree
			StateVector stateVP;
			stateVP[0] = poseTmp.pose.position.x;
			stateVP[1] = poseTmp.pose.position.y;
			stateVP[2] = poseTmp.pose.position.z;
			stateVP[3] = std::atof(poseStr[6].c_str());
			fillOctree(poseTmp.header.seq, stateVP);
		}
	}
	else
		ROS_ERROR("Error on path file");		
	file.close();	
}

// Add id of the viewpoint in each cube between the viewpoint and the observed triangle
void fillOctree(int VP_id, StateVector vp) {
	std::vector<double> xs;
	std::vector<double> ys;
	std::vector<double> zs;
	
	// VP_id=0 <=> Start point => No triangle to inspect
	if(VP_id > 0) {
		xs = {(*mesh)[VP_id-1].poses[0].pose.position.x, (*mesh)[VP_id-1].poses[1].pose.position.x, (*mesh)[VP_id-1].poses[2].pose.position.x, vp[0]};
		ys = {(*mesh)[VP_id-1].poses[0].pose.position.y, (*mesh)[VP_id-1].poses[1].pose.position.y, (*mesh)[VP_id-1].poses[2].pose.position.y, vp[1]};
		zs = {(*mesh)[VP_id-1].poses[0].pose.position.z, (*mesh)[VP_id-1].poses[1].pose.position.z, (*mesh)[VP_id-1].poses[2].pose.position.z, vp[2]};
	}
	else {
		xs = {vp[0]};
		ys = {vp[1]};
		zs = {vp[2]};
	}		
	
	auto minmaxX = std::minmax_element(xs.begin(), xs.end());
	auto minmaxY = std::minmax_element(ys.begin(), ys.end());
	auto minmaxZ = std::minmax_element(zs.begin(), zs.end());
	
	// Values in the octree E [0,512]
	for(int z=(int) (((double)*minmaxZ.first)*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0); z<=(int) (((double)*minmaxZ.second)*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0)+1; z++) {
		for(int y=(int) (((double)*minmaxY.first)*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0); y<=(int) (((double)*minmaxY.second)*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0)+1; y++) {
			for(int x=(int) (((double)*minmaxX.first)*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0); x<=(int) (((double)*minmaxX.second)*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0)+1; x++) {
				(*octree)(x,y,z).push_back(VP_id);
			}
		}
	}
}

void setupSystem() {
	ROS_INFO("Initialisation");
	
	sleep_time = 0.01;
	//sleep_time = 0.0;
	obsBoxSize = 50;
	pkgPath = ros::package::getPath("koptplanner");
	g_security_distance = 5.0;
	octreeSize = 32;
	
	octree = new Octree<std::vector<int>>(octreeSize);

	// TODO not here need to see to click
	readSTLfile(ros::package::getPath("request")+"/meshes/asciiavion.stl");
	readPath();
	
	//publishStl();
	//publishPath();

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

	scaleVP = sqrt(SQ(problemBoundary.size[0])+SQ(problemBoundary.size[1])+SQ(problemBoundary.size[2]))/70.0;
	scaleVP/=1;	
	
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

void planForHiddenTrangles() {
#ifdef __TIMING_INFO__
		timeval time;
		gettimeofday(&time, NULL);
		millisecStart = time.tv_sec * 1000 + time.tv_usec / 1000;
		time_start = millisecStart;
#endif

	// Get ids from viewpoints which has the triangle hides by the obstacle box
	// Values in the octree E [0,octreeSize]
	for(int z=(int) zmin_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; z<=(int) zmax_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; z++) {
		for(int y=(int) ymin_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; y<=(int) ymax_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; y++) {
			for(int x=(int) xmin_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; x<=(int) xmax_obs*(octreeSize-1)/spaceSize[0]+(octreeSize-1)/2.0; x++) {
				listKO.insert(std::end(listKO), std::begin((*octree)(x,y,z)), std::end((*octree)(x,y,z)));
			}
		}
	}
	std::unordered_set<double> setKO(std::begin(listKO), std::end(listKO));
	listKO.assign(std::begin(setKO), std::end(setKO));
	/*
	for(int i=0; i<path.size(); i++) {
		selectViewpointFromFile(i, i);
	}
	
	std::unordered_set<double> s(std::begin(listKOComp), std::end(listKOComp));
	listKOComp.assign(std::begin(s), std::end(s));
	
	ROS_INFO("maxID_octree:%d, maxID_all:%d", (int) listKO.size(), (int) listKOComp.size());
	
	std::unordered_multiset<int> st;
	st.insert(listKO.begin(), listKO.end());
	st.insert(listKOComp.begin(), listKOComp.end());
	auto predicate = [&st](const int& k){ return st.count(k) > 1; };
	listKO.erase(std::remove_if(listKO.begin(), listKO.end(), predicate), listKO.end());
		
	for(int i=0; i<listKO.size(); i++) {
		ROS_INFO("id:%d", listKO[i]);
	}	
	listKOComp.clear();*/
	
	// TODO a inverse
	if(listKO.size() > 0) {
#ifdef __TIMING_INFO__
		gettimeofday(&time, NULL);
		time_READ -= time.tv_sec * 1000000 + time.tv_usec;
#endif
		bool addFirstVP = false;
		bool addSecondVP = false;
		maxID = listKO.size() + 2;
		ROS_INFO("maxID:%d", maxID);
		
		if(VP) {
		delete[] VP;
		}
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
		
		// Add the last viewpoints OK as start and stop points
		for(int i=0; i<path.size(); i++) {
			if(std::find(listKO.begin(), listKO.end(), path[i].header.seq) != listKO.end()) {
				if(!addFirstVP && !addSecondVP) {
					listKO.insert(listKO.begin(), path[i-1].header.seq);
					
					tf::Pose pose;
					tf::poseMsgToTF(path[i-1].pose, pose);					
					VP[0][0] = path[i-1].pose.position.x;
					VP[0][1] = path[i-1].pose.position.y;
					VP[0][2] = path[i-1].pose.position.z;
					VP[0][3] = tf::getYaw(pose.getRotation());
					addFirstVP = true;
				}
			}
			else if(addFirstVP && !addSecondVP) {
				listKO.push_back(path[i].header.seq);

				tf::Pose pose;
				tf::poseMsgToTF(path[i].pose, pose);
				VP[maxID-1][0] = path[i].pose.position.x;
				VP[maxID-1][1] = path[i].pose.position.y;
				VP[maxID-1][2] = path[i].pose.position.z;
				VP[maxID-1][3] = tf::getYaw(pose.getRotation());
				addSecondVP = true;
				break;
			}
		}
	
		if(reinitRRTs==NULL)
			reinitRRTs = new int[maxID];
		for(int q = 0; q<maxID; q++) {
			reinitRRTs[q] = 1;
		}
	
		/* ------------- TSP -------------- */
		double ** vals = new double* [maxID];
		for(int i = 0; i<maxID; i++) {
			vals[i] = new double[3];
			
			if(i > 0 && i<maxID-1)
				selectViewpointFromFile(listKO[i], i);
			
			vals[i][0] = VP[i][0]*g_scale;
			vals[i][1] = VP[i][1]*g_scale;
			vals[i][2] = VP[i][2]*g_scale;
		}
	
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
	
		for(int i = 0; i<maxID; i++)
			delete plannerArray[i];
		delete[] plannerArray;
		delete[] reinitRRTs;
		reinitRRTs = NULL;
	
		changeInspectionPath();
		publishPath();
	}
	else
		ROS_INFO("Not need to change the inspection path!");
		
#ifdef __TIMING_INFO__
	gettimeofday(&time, NULL);
	ROS_INFO("Calculation time was:\t\t\t%i ms", (int)((long)(time.tv_sec * 1000 + time.tv_usec / 1000) - millisecStart));
	ROS_INFO("Choice VPs time consumption:\t\t%i ms", (int)(time_READ/1000));
	ROS_INFO("LKH time consumption:\t\t\t%i ms", (int)(time_LKH/1000));
	ROS_INFO("Initial RRT* time consumption:\t\t%i ms", (int)(time_RRTS/1000));
	ROS_INFO("Distance evaluation time:\t\t%i ms", (int)(time_RRTS_req/1000));
#endif

	cleanVariables();
}

bool isTriangleVisible(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax) {
	return (xmin_obs < xmin && xmax_obs < xmin || xmin_obs > xmax && xmax_obs > xmax ||
			ymin_obs < ymin && ymax_obs < ymin || ymin_obs > ymax && ymax_obs > ymax ||
			zmin_obs < zmin && zmax_obs < zmin || zmin_obs > zmax && zmax_obs > zmax);
}

void selectViewpointFromFile(int VP_id, int VPKO_id) {
	bool VP_OK = false;
	std::string line;
	StateVector VPtmp;
	
	std::vector<double> xs;
	std::vector<double> ys;
	std::vector<double> zs;
	
	if(VP_id > 0) {
		xs = {(*mesh)[VP_id-1].poses[0].pose.position.x, (*mesh)[VP_id-1].poses[1].pose.position.x, (*mesh)[VP_id-1].poses[2].pose.position.x};
		ys = {(*mesh)[VP_id-1].poses[0].pose.position.y, (*mesh)[VP_id-1].poses[1].pose.position.y, (*mesh)[VP_id-1].poses[2].pose.position.y};
		zs = {(*mesh)[VP_id-1].poses[0].pose.position.z, (*mesh)[VP_id-1].poses[1].pose.position.z, (*mesh)[VP_id-1].poses[2].pose.position.z};
	}
	
	file.open((pkgPath+"/viewpoints/viewpoint_"+std::to_string(VP_id)+".txt").c_str());
	if (file.is_open())	{
		while(!VP_OK && getline(file,line)) {
			std::vector<std::string> pose;
			boost::split(pose, line, boost::is_any_of("\t"));

			// StateVector for Rotorcraft = [x,y,z,yaw]
			VPtmp[0] = std::atof(pose[0].c_str());
			VPtmp[1] = std::atof(pose[1].c_str());
			VPtmp[2] = std::atof(pose[2].c_str());
			VPtmp[3] = std::atof(pose[5].c_str());
			
			xs.push_back(VPtmp[0]);
			auto minmaxX = std::minmax_element(xs.begin(), xs.end());
 
			ys.push_back(VPtmp[1]);
			auto minmaxY = std::minmax_element(ys.begin(), ys.end());
			
			zs.push_back(VPtmp[2]);
			auto minmaxZ = std::minmax_element(zs.begin(), zs.end());
			
			VP_OK = isTriangleVisible(*minmaxX.first, *minmaxY.first, *minmaxZ.first, *minmaxX.second, *minmaxY.second, *minmaxZ.second);
			
			if(!VP_OK) {
				xs.pop_back();
				ys.pop_back();
				zs.pop_back();
				// TODO debug
				//listKOComp.push_back(VP_id);
			}
			/*else{
				for(int i=0; i <xs.size(); i++) {
					VPtmp[0] = xs[i];
					VPtmp[1] = ys[i];
					VPtmp[2] = zs[i];
					VPtmp[3] = std::atof(pose[5].c_str());
					publishViewpoint(VPtmp, VP_id+i, 0.0);
				}
				publishVisibilityBox(VP_id, *minmaxX.first, *minmaxY.first, *minmaxZ.first, *minmaxX.second, *minmaxY.second, *minmaxZ.second, std::atof(pose[5].c_str()));	
			}*/
		}
		file.close();
		//TODO debug
		
		if(VP_OK) {
			// VP = global variable
			VP[VPKO_id][0] = VPtmp[0];
			VP[VPKO_id][1] = VPtmp[1];
			VP[VPKO_id][2] = VPtmp[2];
			VP[VPKO_id][3] = VPtmp[3];
			publishViewpoint(VPtmp, VP_id, 0.0);
		}
		else {
			ROS_ERROR("No collision free viewpoint for triangle %d", VP_id);
			VP[VPKO_id][0] = sys_t::obstacles.front()->center[0];
			VP[VPKO_id][1] = sys_t::obstacles.front()->center[1];
			VP[VPKO_id][2] = sys_t::obstacles.front()->center[2];
			VP[VPKO_id][3] = 0.0;
			ROS_INFO("VP= x:%f, y:%f, z:%f (Center of the obstacle)", VP[VP_id][0], VP[VP_id][1], VP[VP_id][2]);
			//ros::shutdown();
		}
	}
	else {
		ROS_ERROR("Viewpoint %d file not found", VP_id);
	}
}

void changeInspectionPath() {
	bool addedPath = false;
	
	//TODO no point found
	
	// Clear old path
	file.open((pkgPath+"/data/tour.txt").c_str(), std::ios::out);
	file.close();
	
	file.open((pkgPath+"/data/tour.txt").c_str(), std::ios::out | std::ios::app);
	if(file.is_open()) {
		for(std::vector<geometry_msgs::PoseStamped>::iterator it_path = std::begin(path); it_path != std::end(path); it_path++) {
			if(std::find(listKO.begin(), listKO.end(), it_path->header.seq) == listKO.end()) {
				tf::Pose pose;
				tf::poseMsgToTF(it_path->pose, pose);
			
				file << it_path->header.seq << "\t";
				file << it_path->pose.position.x << "\t" << it_path->pose.position.y << "\t" << it_path->pose.position.z << "\t";
				file << "0\t0\t" << tf::getYaw(pose.getRotation()) << "\n";
			}
			else if(!addedPath) {
				ROS_INFO("size path:%d", (int) path.size());
				ROS_INFO("size:%d", (int) res_g->inspectionPath.poses.size());
			
				for(int i=0; i<res_g->inspectionPath.poses.size(); i+=2) {
					tf::Pose pose;
					tf::poseMsgToTF(res_g->inspectionPath.poses[i].pose, pose);
				
					file << listKO[i/2] << "\t";
					file << res_g->inspectionPath.poses[i].pose.position.x << "\t";
					file << res_g->inspectionPath.poses[i].pose.position.y << "\t";
					file << res_g->inspectionPath.poses[i].pose.position.z << "\t";
					file << "0\t0\t" << tf::getYaw(pose.getRotation()) << "\n";
				}
				addedPath = true;
			}
		}
		file.close();
	}
	else
		ROS_ERROR("Can't open tour.txt file");
}

void cleanVariables() {
	ROS_INFO("Variables cleaning");
	// tidy up
	if(!sys_t::obstacles.empty()) {
		for(typename std::list<reg_t*>::iterator it = sys_t::obstacles.begin(); it != sys_t::obstacles.end(); it++)
			delete (*it);
		sys_t::obstacles.clear();
	}
	
	delete[] VP;
	VP = NULL;
	maxID = 0;
	if(lookupTable) {
		for(int i = 0; i < LOOKUPTABLE_SIZE; i++)
			delete[] lookupTable[i];
		delete[] lookupTable;
		lookupTable = NULL;
	}
	plannerArrayBool = false;
	listKO.clear();
	path.clear();
}

void readSTLfile(std::string name) {
	mesh = new std::vector<nav_msgs::Path>;
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
		mesh->push_back(p);
		k++;
	}
	free(line);
	f.close();
	ROS_INFO("Mesh area is bounded by: [%2.2f,%2.2f]x[%2.2f,%2.2f]x[%2.2f,%2.2f]", minX,maxX,minY,maxY,minZ,maxZ);
}

void publishStl() {
	ros::Rate r(50.0);
	
	/* publish STL file to rviz */
	for(std::vector<nav_msgs::Path>::iterator it = mesh->begin(); it != mesh->end() && ros::ok(); it++) {
		stl_pub.publish(*it);
		r.sleep();
	}
}

void publishViewpoint(StateVector stateVP, int VP_ind, double blue) {
	/* display sampled viewpoint in rviz */
	visualization_msgs::Marker point;
	point.header.frame_id = "/kopt_frame";
	point.header.stamp = ros::Time::now();
	point.id = VP_ind;
	point.ns = "Viewpoints";
	point.type = visualization_msgs::Marker::ARROW;
	point.pose.position.x = stateVP[0];
	point.pose.position.y = stateVP[1];
	point.pose.position.z = stateVP[2];
	double scaleVP;

#if DIMENSIONALITY>4
	tf::Quaternion q = tf::createQuaternionFromRPY(stateVP[3],stateVP[4],stateVP[5]);
#else
	tf::Quaternion q = tf::createQuaternionFromRPY(0,0,stateVP[3]);
#endif
	point.pose.orientation.x = q.x();
	point.pose.orientation.y = q.y();
	point.pose.orientation.z = q.z();
	point.pose.orientation.w = q.w();
	
	if(problemBoundary.size)
		scaleVP = sqrt(SQ(problemBoundary.size[0])+SQ(problemBoundary.size[1])+SQ(problemBoundary.size[2]))/70.0;
	else
		scaleVP = sqrt(SQ(spaceSize[0])+SQ(spaceSize[1])+SQ(spaceSize[2]))/70.0;
	scaleVP/=1;
	point.scale.x = scaleVP;
	point.scale.y = scaleVP/25.0;
	point.scale.z = scaleVP/25.0;
	point.color.r = 0.8f;
	point.color.g = 0.5f;
	point.color.b = 0.0f+blue;
	point.color.a = 0.7;
	point.lifetime = ros::Duration();
	viewpoint_pub.publish(point);

	ros::Duration(sleep_time).sleep();
}

void debugNoVP(std::vector<int> listKO) {
	int next_id = 0;
	std::string line;
	
	for(int i=0; i<listKO.size(); i++) {
		file.open((pkgPath+"/viewpoints/viewpoint_"+std::to_string(listKO[i])+".txt").c_str());
		if (file.is_open())	{
			while(getline(file,line)) {
				StateVector VPtmp;
				std::vector<std::string> pose;
				boost::split(pose, line, boost::is_any_of("\t"));

				// StateVector for Rotorcraft = [x,y,z,yaw]
				VPtmp[0] = std::atof(pose[0].c_str());
				VPtmp[1] = std::atof(pose[1].c_str());
				VPtmp[2] = std::atof(pose[2].c_str());
				VPtmp[3] = std::atof(pose[5].c_str());

				publishViewpoint(VPtmp, i*1000+next_id, float(i)/listKO.size());
				next_id++;
			}
			file.close();
		}
		else
			ROS_ERROR("File not found");
	}
	listKO.clear();
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
		publishViewpoint(stateVP, path[i].header.seq, 0);
	}
	
	// End point
	//pathToPub.poses.push_back(path[0]);				
		
	marker_pub.publish(pathToPub);	
}

// Euclidean distance between obstacle center and VP
bool inClickedObstacle(StateVector VPtmp) {
	return sqrt(SQ(VPtmp[0]-sys_t::obstacles.front()->center[0]) +
				SQ(VPtmp[1]-sys_t::obstacles.front()->center[1]) +
				SQ(VPtmp[2]-sys_t::obstacles.front()->center[2])) > sys_t::obstacles.front()->size[0]+g_security_distance;
}

void publishVisibilityBox(int VP_id, double xmin, double ymin, double zmin, double xmax, double ymax, double zmax, double yaw) {
	// publish obstacles for rviz 
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/kopt_frame";
	marker.header.stamp = ros::Time::now();
	marker.ns = "box";
	marker.id = VP_id; // enumerate when adding more obstacles
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = (xmax+xmin)/2.0;
	marker.pose.position.y = (ymax+ymin)/2.0;
	marker.pose.position.z = (zmax+zmin)/2.0;
	tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
	marker.pose.orientation.x = q.x();
	marker.pose.orientation.y = q.y();
	marker.pose.orientation.z = q.z();
	marker.pose.orientation.w = q.w();

	marker.scale.x = xmax-xmin;
	marker.scale.y = ymax-ymin;
	marker.scale.z = zmax-zmin;

	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 0.1;

	marker.lifetime = ros::Duration();
	obstacle_pub.publish(marker);
	ros::Duration(sleep_time).sleep();
}

void generateNewTourFile() {
	std::string line;
	std::string lineVP;
	std::fstream fileVP;
	std::fstream fileTour;
	pkgPath = ros::package::getPath("koptplanner");

	fileTour.open((pkgPath+"/data/tour.txt").c_str(), std::ios::out);
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
					else
						ROS_ERROR("viewpoint file not found!");
					fileVP.close();
				}
			}		
		}
		else
			ROS_ERROR("tempTourSave file not found!");
		file.close();
	}
	else
		ROS_ERROR("tour file not found!");	file.close();	
	fileTour.close();
}










