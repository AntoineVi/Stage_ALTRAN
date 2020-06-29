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

#ifdef __TIMING_INFO__
 long time_DBS;
 long time_RRTS;
 long time_RRTS_req;
 long time_LKH;
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

std::vector<StateVector> gVect;
std::string pkgPath;

ros::Subscriber sub;
int obsBoxSize;
ros::Publisher obstacle_pub;
ros::Publisher stl_pub;
bool initOK;
double sleep_time;

bool endPoint;
double scaleVP;
std::vector<double> spaceSize = {1000.0, 1000.0, 1000.0};
std::vector<double> spaceCenter = {0.0, 0.0, 0.0};

int count_files(std::string directory, std::string ext);
void init();
void clean();

void poseObstacle(const geometry_msgs::PointStamped& clicked_pose);
std::vector<nav_msgs::Path> * readSTLfile(std::string name);
void publish_stl();
void publish_viewpoint(StateVector stateVP, int VP_id);

bool viewpoints_generator(koptplanner::inspection::Request  &req, koptplanner::inspection::Response &res);
void viewpoint_reader(int VP_id);
void planFromSavedVPs();

bool inClickedObstacle(StateVector VPtmp);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "koptplanner");
	ros::NodeHandle n;
	
	marker_pub = n.advertise<nav_msgs::Path>("visualization_marker", 1);
	viewpoint_pub = n.advertise<visualization_msgs::Marker>("viewpoint_marker", 1);  
  	sub = n.subscribe("clicked_point", 1, poseObstacle);
  	obstacle_pub = n.advertise<visualization_msgs::Marker>("scenario", 1);
  	stl_pub = n.advertise<nav_msgs::Path>("stl_mesh", 1);
  	
  	initOK = false;
  	sleep_time = 1000.0;
  	obsBoxSize = 50;
  	
	publish_stl();
  	
	ROS_INFO("Ready to receive obstacle pose from Publish point");
	
	ros::ServiceServer service = n.advertiseService("inspectionPath", viewpoints_generator);
	ROS_INFO("Service started");
	
	ros::spin();

	return 0;
}

void poseObstacle(const geometry_msgs::PointStamped& clicked_pose) {
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

	ros::Rate r(sleep_time);
	r.sleep();
	
	planFromSavedVPs();
}

int count_files(std::string directory, std::string ext)
{
	namespace fs = boost::filesystem;
	fs::path Path(directory);
	int Nb_ext = 0;
	fs::directory_iterator end_iter; // Default constructor for an iterator is the end iterator

	for (fs::directory_iterator iter(Path); iter != end_iter; ++iter)
		if (iter->path().extension() == ext)
			++Nb_ext;

	return Nb_ext;
}

void publish_viewpoint(StateVector stateVP, int VP_ind) {
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

#if DIMENSIONALITY>4
	tf::Quaternion q = tf::createQuaternionFromRPY(stateVP[3],stateVP[4],stateVP[5]);
#else
	tf::Quaternion q = tf::createQuaternionFromRPY(0,0,stateVP[3]);
#endif
	point.pose.orientation.x = q.x();
	point.pose.orientation.y = q.y();
	point.pose.orientation.z = q.z();
	point.pose.orientation.w = q.w();

	double scaleVP = sqrt(SQ(problemBoundary.size[0])+SQ(problemBoundary.size[1])+SQ(problemBoundary.size[2]))/70.0;
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

	ros::Rate r(sleep_time);
	r.sleep();
}

void init() {
	ROS_INFO("Initialisation");

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

	pkgPath = ros::package::getPath("koptplanner");

	maxID = count_files((pkgPath+"/viewpoints").c_str(), ".txt"); // Number of files in viewpoints directory
	
	g_tourlength = pkgPath + "/data/tourlength.m";
	file.open(g_tourlength.c_str(), std::ios::out);
	if(file.is_open()) {
		file << "length=[";
		file.close();
	}
	else {
		ROS_WARN("tourlength file not found");
	}

	
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
	g_security_distance = 2.0;
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
#endif
	time_start = 0;
	millisecStart = 0;
	
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
	if(reinitRRTs==NULL)
		reinitRRTs = new int[maxID];
	for(int q = 0; q<maxID; q++) {
		reinitRRTs[q] = 1;
	}
		
	scaleVP = sqrt(SQ(problemBoundary.size[0])+SQ(problemBoundary.size[1])+SQ(problemBoundary.size[2]))/70.0;
	scaleVP/=1;
	
	initOK = true;
}

bool viewpoints_generator(koptplanner::inspection::Request  &req, koptplanner::inspection::Response &res) {
	ROS_INFO("Request received");

	if(!initOK)
		init();
		
	koptError = SUCCESSFUL;
	/* clean viewpoints file */
	for(int i=0; i<maxID; i++) {
		file.open((pkgPath+"/viewpoints/viewpoint_"+std::to_string(i)+".txt").c_str(), std::ios::out);
		file.close();
	}
  
	/* preparing log file */
	std::fstream plannerLog;
	plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::out);
	plannerLog << "Inspection path planner version 1.0.0\nScenario:\n";
	plannerLog << "Number of facets:\t" << req.inspectionArea.size() << "\n";
	plannerLog << "Number of obstacles:\t" << req.obstacles.size() << "\n";
	plannerLog << "Space center:\t\t(" << req.spaceCenter[0] << ", " << req.spaceCenter[1] << ", " << req.spaceCenter[2] << ")\n";
	plannerLog << "Space size:\t\t(" << req.spaceSize[0] << ", " << req.spaceSize[1] << ", " << req.spaceSize[2] << ")\n";
	if(req.requiredPoses.size()>0)
		plannerLog << "Starting point:\t\t(" << req.requiredPoses[0].position.x << ", " << req.requiredPoses[0].position.y << ", " << req.requiredPoses[0].position.z << ")\n";
	if(req.requiredPoses.size()>1)
		plannerLog << "Final point:\t\t(" << req.requiredPoses[1].position.x << ", " << req.requiredPoses[1].position.y << ", " << req.requiredPoses[1].position.z << ")\n";
	plannerLog << "Angle of incidence:\t" << req.incidenceAngle <<"rad\n";
	plannerLog << "Range:\t\t\t(" << req.minDist << ", " << req.maxDist << ")m\n";
	plannerLog << "Number of iterations:\t" << (int)(req.numIterations) <<"\n";
#ifdef USE_FIXEDWING_MODEL
	plannerLog << "Speed:\t\t\t" << g_speed <<"m/s\n";
	plannerLog << "Rmin:\t\t\t" << sys_t::r_min <<"m\n";
#else
	plannerLog << "Linear speed:\t\t" << g_speed <<"m/s\n";
	plannerLog << "Yaw rate:\t\t" << g_maxAngularSpeed <<"rad/s\n";
#endif
	plannerLog << "Field of view [h,v]:\t[" << g_camAngleHorizontal << ", " << g_camAngleVertical << "]rad\n";
	plannerLog << "Camera pitch:\t\t" << g_camPitch <<"rad\n\n";
	plannerLog.close();

	res_g = &res;

	g_max_obs_dim = 0;
	if(req.requiredPoses.size() < 1){ 
		ROS_ERROR("No start position defined! This is required.");
		std::string pkgPath = ros::package::getPath("koptplanner");
		std::fstream plannerLog;
		plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
		if(!plannerLog.is_open())
			ROS_ERROR("Could not open report.log");
		plannerLog << "-->No start position defined! This is required by the current implementation.\n";
		plannerLog.close();
		koptError = NO_START_POSITION;
		return true;
	}

	timeval time;
	gettimeofday(&time, NULL);
	millisecStart = time.tv_sec * 1000 + time.tv_usec / 1000;
	time_start = millisecStart;
	ros::Rate visualizerRate(1000);

	tri_t::setCamBoundNormals();
	std::vector<tri_t*> tri;
	tri_t::setParam(req.incidenceAngle, req.minDist, req.maxDist); // incidence angle from surface plane
  
	/* startpoint and further strictly required positions */
	for(std::vector<geometry_msgs::Pose>::iterator itFixedPoses = req.requiredPoses.begin(); itFixedPoses != req.requiredPoses.end() && (itFixedPoses != req.requiredPoses.end()-1 || req.requiredPoses.size()==1); itFixedPoses++) {
		tri_t * tmp = new tri_t;
		tmp->x1[0] = itFixedPoses->position.x;
		tmp->x1[1] = itFixedPoses->position.y;
		tmp->x1[2] = itFixedPoses->position.z;
		tf::Pose pose;
		tf::poseMsgToTF(*itFixedPoses, pose);
		double yaw_angle = tf::getYaw(pose.getRotation());
#ifdef USE_ROTORCRAFT_MODEL
		tmp->x2[0] = yaw_angle;
		tmp->x2[1] = 0.0;
		tmp->x2[2] = 0.0;
#elif defined USE_FIXEDWING_MODEL
		tmp->x2[0] = 0.0;
		tmp->x2[1] = 0.0;
		tmp->x2[2] = yaw_angle;
#endif
		tmp->Fixpoint = true;
		tri.push_back(tmp);
		file.open(pkgPath+"/viewpoints/viewpoint_0.txt", std::ios::app | std::ios::out);
		if(!file.is_open())
			ROS_ERROR("Could not open viewpoint file");
		file << std::to_string(itFixedPoses->position.x)+"\t";
		file << std::to_string(itFixedPoses->position.y)+"\t";
		file << std::to_string(itFixedPoses->position.z)+"\t";
		file << std::to_string(itFixedPoses->orientation.x)+"\t";
		file << std::to_string(itFixedPoses->orientation.y)+"\t";
		file << std::to_string(itFixedPoses->orientation.z)+"\t";
		file << std::to_string(itFixedPoses->orientation.w)+"\n";
		file.close();
	}

	/* obstacles */
	std::vector<geometry_msgs::Pose>::iterator itPose = req.obstaclesPoses.begin();
	std::vector<int>::iterator itOccupancy = req.obstacleIntransparancy.begin();
	for(std::vector<shape_msgs::SolidPrimitive>::iterator it = req.obstacles.begin(); it != req.obstacles.end() && itPose != req.obstaclesPoses.end() && itOccupancy != req.obstacleIntransparancy.end(); it++, itPose++, itOccupancy++) {
		if(it->type != shape_msgs::SolidPrimitive::BOX) {
			ROS_ERROR("Invalid obstacle shape: Only BOX is supported at this time.");
			std::string pkgPath = ros::package::getPath("koptplanner");
			std::fstream plannerLog;
			plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
			if(!plannerLog.is_open())
				ROS_ERROR("Could not open report.log");
			plannerLog << "-->Invalid obstacle shape: Only BOX is supported at this time.\n";
			plannerLog.close();
			koptError = INVALID_OBSTACLE_SHAPE;
			return true;
    	}
		reg_t * obs = new reg_t;
		obs->setNumDimensions(3);
		obs->occupied = *itOccupancy;
		obs->center[0] = itPose->position.x;
		obs->center[1] = itPose->position.y;
		obs->center[2] = itPose->position.z;
		assert(it->type == shape_msgs::SolidPrimitive::BOX);
		obs->size[0] = it->dimensions[shape_msgs::SolidPrimitive::BOX_X];
		obs->size[1] = it->dimensions[shape_msgs::SolidPrimitive::BOX_Y];
		obs->size[2] = it->dimensions[shape_msgs::SolidPrimitive::BOX_Z];
		if(g_max_obs_dim<obs->size[0])
			g_max_obs_dim=obs->size[0];
		if(g_max_obs_dim<obs->size[1])
			g_max_obs_dim=obs->size[1];
		if(g_max_obs_dim<obs->size[2])
			g_max_obs_dim=obs->size[2];
		sys_t::obstacles.push_back(obs);
	}
	/* mesh */
	for(std::vector<geometry_msgs::Polygon>::iterator it = req.inspectionArea.begin(); it != req.inspectionArea.end(); it++) {
		tri_t* tmp = new tri_t;
		tmp->x1[0] = (*it).points[0].x;
		tmp->x1[1] = (*it).points[0].y;
		tmp->x1[2] = (*it).points[0].z;

		tmp->x2[0] = (*it).points[1].x;
		tmp->x2[1] = (*it).points[1].y;
		tmp->x2[2] = (*it).points[1].z;

		tmp->x3[0] = (*it).points[2].x;
		tmp->x3[1] = (*it).points[2].y;
		tmp->x3[2] = (*it).points[2].z;

		Vector3f a = ((tmp->x2-tmp->x1).cross(tmp->x3-tmp->x2))/2;

		if(a.norm() == 0.0) {
			std::string pkgPath = ros::package::getPath("koptplanner");
			std::fstream plannerLog;
			plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
			if(!plannerLog.is_open())
				ROS_ERROR("Could not open report.log");
			plannerLog << "-->Triangle does not have any area. Normal can not be computed. Omitting!\n";
			plannerLog << "   x1: (" << tmp->x1[0] << ", " << tmp->x1[1] << ", " << tmp->x1[2] << ")\n";
			plannerLog << "   x2: (" << tmp->x2[0] << ", " << tmp->x2[1] << ", " << tmp->x2[2] << ")\n";
			plannerLog << "   x3: (" << tmp->x3[0] << ", " << tmp->x3[1] << ", " << tmp->x3[2] << ")\n";
			plannerLog.close();
		}
		else {
			tmp->init();
			tri.push_back(tmp);
		}
	}
	/* end point */
	if(req.requiredPoses.size()>1 && // selecting the same start and end point causes the LKH to fail
	  ((req.requiredPoses.end()-1)->position.x != req.requiredPoses[0].position.x ||
	  (req.requiredPoses.end()-1)->position.y != req.requiredPoses[0].position.y ||
	  (req.requiredPoses.end()-1)->position.z != req.requiredPoses[0].position.z ||
	  (req.requiredPoses.end()-1)->orientation.x != req.requiredPoses[0].orientation.x ||
	  (req.requiredPoses.end()-1)->orientation.y != req.requiredPoses[0].orientation.y ||
	  (req.requiredPoses.end()-1)->orientation.z != req.requiredPoses[0].orientation.z ||
	  (req.requiredPoses.end()-1)->orientation.w != req.requiredPoses[0].orientation.w))
	{
		g_closed_tour = false;
		endPoint = true;
		tri_t * tmp = new tri_t;
		tmp->x1[0] = (req.requiredPoses.end()-1)->position.x;
		tmp->x1[1] = (req.requiredPoses.end()-1)->position.y;
		tmp->x1[2] = (req.requiredPoses.end()-1)->position.z;
		tf::Pose pose;
		tf::poseMsgToTF(*(req.requiredPoses.end()-1), pose);
		double yaw_angle = tf::getYaw(pose.getRotation());
#ifdef USE_ROTORCRAFT_MODEL
		tmp->x2[0] = yaw_angle;
		tmp->x2[1] = 0.0;
		tmp->x2[2] = 0.0;
#elif defined USE_FIXEDWING_MODEL
		tmp->x2[0] = 0.0;
		tmp->x2[1] = 0.0;
		tmp->x2[2] = yaw_angle;
#endif
		tmp->Fixpoint = true;
		tri.push_back(tmp);
		
		file.open((pkgPath+"/viewpoints/viewpoint_"+std::to_string(tri.size())+".txt").c_str(), std::ios::app | std::ios::out);
		if(!file.is_open())
			ROS_ERROR("Could not open viewpoint file");
		file << std::to_string((req.requiredPoses.end()-1)->position.x)+"\t";
		file << std::to_string((req.requiredPoses.end()-1)->position.y)+"\t";
		file << std::to_string((req.requiredPoses.end()-1)->position.z)+"\t";
		file << std::to_string((req.requiredPoses.end()-1)->orientation.x)+"\t";
		file << std::to_string((req.requiredPoses.end()-1)->orientation.y)+"\t";
		file << std::to_string((req.requiredPoses.end()-1)->orientation.z)+"\t";
		file << std::to_string((req.requiredPoses.end()-1)->orientation.w)+"\n";
		file.close();
	}
 
	maxID = tri.size();
	if(maxID<4)	{
		ROS_ERROR("Too few inspection areas specified. A minium number of 4 is required, where only %i are given.", maxID);
		std::string pkgPath = ros::package::getPath("koptplanner");
		std::fstream plannerLog;
		plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
		if(!plannerLog.is_open())
			ROS_ERROR("Could not open report.log");
		plannerLog << "-->Too few inspection areas specified. A minium number of 4 is required, where only " << maxID << " are given.\n";
		plannerLog.close();
		koptError = TOO_FEW_INSPECTION_AREAS;
		return true;
	}

	int koptPlannerIteration = 0;
	/* run planner for numIterations */
	ROS_INFO("Start AGP");
	while(!ros::isShuttingDown() && koptPlannerIteration < req.numIterations)
	{
		ROS_INFO("STARTING ITERATION %i", koptPlannerIteration);
		plannerLog.open((pkgPath+"/data/report.log").c_str(), std::ios::app | std::ios::out);
		if(!plannerLog.is_open())
			ROS_ERROR("Could not open report.log");
		plannerLog << "\n====================================\n";
		plannerLog << "====== Starting Iteration  " << (koptPlannerIteration>9 ? "" : " ") << koptPlannerIteration << " ======\n";
		plannerLog << "====================================\n";
		plannerLog.close();

		/* -------- ART -- GALLERY -------- */
		for(int i = 0; i < maxID; i++)
		{
			StateVector * s1 = NULL;
			StateVector * s2 = NULL;
			/* sample viewpoint */
			StateVector VPtmp = tri[i]->dualBarrierSampling_vect(s1,s2,&VP[i],gVect);
			//StateVector VPtmp = tri[i]->dualBarrierSampling(s1,s2,&VP[i]);
			if(koptPlannerIteration == 0 || vp_tol<(VPtmp - VP[i]).norm()) {
				VP[i] = VPtmp;
				reinitRRTs[i] = 1; // delete (if existing) and reinitialize rrt* tree
			}
			else
				reinitRRTs[i] = 2; // only add iterations to the rrt* tree and don't adapt the viewpoint pose

			delete s1;
			s1 = NULL;
			delete s2;
			s2 = NULL;
			
			for(int j=0; j<gVect.size(); j++) {
				publish_viewpoint(gVect[j], i*10000+koptPlannerIteration*1000+j);				

				file.open((pkgPath+"/viewpoints/viewpoint_"+std::to_string(i)+".txt").c_str(), std::ios::app | std::ios::out);
				if(!file.is_open())
					ROS_ERROR("Could not open viewpoint file");
				file << std::to_string(gVect[j][0])+"\t";
				file << std::to_string(gVect[j][1])+"\t";
				file << std::to_string(gVect[j][2])+"\t";
				file << std::to_string(gVect[j][3])+"\t";
				file << std::to_string(gVect[j][4])+"\t";
				file << std::to_string(gVect[j][5])+"\t";
				file << std::to_string(gVect[j][6])+"\n";
				file.close();
			}
			gVect.clear();
    	}    
    
		if(koptError != SUCCESSFUL)	{
		  ROS_ERROR("Error occured! ID: %i", koptError);
		  return koptError == SUCCESSFUL;
		}
		koptPlannerIteration++;
	} // while

	res.cost = g_cost;

	gettimeofday(&time, NULL);
	ROS_INFO("Calculation time was:\t\t\t%i ms", (int)((long)(time.tv_sec * 1000 + time.tv_usec / 1000) - millisecStart));
#ifdef __TIMING_INFO__
	ROS_INFO("Viewpoint sampling time consumption:\t%i ms", (int)(time_DBS/1000));
#endif

	if(koptError == SUCCESSFUL)
		ROS_INFO("sending back response");
	else
		ROS_ERROR("Error occured! ID: %i", koptError);
		
		
	for(typename std::vector<tri_t*>::iterator it = tri.begin(); it != tri.end(); it++)
		delete (*it);
	tri.clear();
	
	clean();
		
	return koptError == SUCCESSFUL;
}

bool inClickedObstacle(StateVector VPtmp) {
	return sqrt(SQ(VPtmp[0]-sys_t::obstacles.front()->center[0]) +
				SQ(VPtmp[1]-sys_t::obstacles.front()->center[1]) +
				SQ(VPtmp[2]-sys_t::obstacles.front()->center[2])) > sys_t::obstacles.front()->size[0]/2.0+g_security_distance;
}

void viewpoint_reader(int VP_id) {
	bool VP_OK = false;
	StateVector VPtmp;
	std::string line;

	file.open((pkgPath+"/viewpoints/viewpoint_"+std::to_string(VP_id)+".txt").c_str());
	if (file.is_open())	{
		while(!VP_OK && getline(file,line)) {
			std::vector<std::string> pose;
			boost::split(pose, line, boost::is_any_of("\t"));

			VPtmp[0] = std::atof(pose[0].c_str());
			VPtmp[1] = std::atof(pose[1].c_str());
			VPtmp[2] = std::atof(pose[2].c_str());
			VPtmp[3] = std::atof(pose[3].c_str());
									
			VP_OK = inClickedObstacle(VPtmp);
		}
		file.close();
		
		if(VP_OK) {
			// VP = global variable
			VP[VP_id][0] = VPtmp[0];
			VP[VP_id][1] = VPtmp[1];
			VP[VP_id][2] = VPtmp[2];
			publish_viewpoint(VPtmp, VP_id);
		}
		else {
			ROS_ERROR("No collision free viewpoint for triangle %d", VP_id);
			ROS_INFO("VP: x:%f, y:%f, z:%f", VP[VP_id][0], VP[VP_id][1], VP[VP_id][2]);
		}		
	}
	else {
		ROS_ERROR("Viewpoint %d file not found", VP_id);
	}
}

//recoller les trajectoires (ptpPlanner / Tour_ ?

void planFromSavedVPs() {
	if(!initOK)
		init();

	/* ------------- TSP -------------- */
	double ** vals = new double* [maxID];
	for(int i = 0; i<maxID; i++) {
		vals[i] = new double[3];
				
		viewpoint_reader(i);

		vals[i][0] = VP[i][0]*g_scale;
		vals[i][1] = VP[i][1]*g_scale;
		vals[i][2] = VP[i][2]*g_scale;
	}
	
	/* use provided interface of the TSP solver */
	std::string params = "MOVE_TYPE=5\n";
	params += "PRECISION=1\n";
	params += "PATCHING_C=3\n";
	params += "PATCHING_A=2\n";
	params += "RUNS=1\n";
	params += "TIME_LIMIT=5\n";
	params += "TRACE_LEVEL=0\n";
	params += "OUTPUT_TOUR_FILE=tempTour.txt\n";
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
	timeval time;
	gettimeofday(&time, NULL);
	if(millisecStart == 0)
		millisecStart = time.tv_sec * 1000 + time.tv_usec / 1000;
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
	
	//ros::Rate r(sleep_time);
	//r.sleep();

#ifdef __TIMING_INFO__
	gettimeofday(&time, NULL);
	time_LKH += time.tv_sec * 1000000 + time.tv_usec;
#endif
	for(int i = 0; i<maxID; i++) {
      delete[] vals[i];
	}
	delete[] vals;
	
	file.open(g_tourlength.c_str(), std::ios::app | std::ios::out);
	if(file.is_open()) {
		file << "];\n";
		gettimeofday(&time, NULL);
		file << "timeEval = " << (int)((long)(time.tv_sec * 1000 + time.tv_usec / 1000) - millisecStart) << ";\n";
#ifdef __TIMING_INFO__
		file << "timeLKH = " << (int)(time_LKH/1000) << ";\n";
		file << "timeRRTs = " << (int)(time_RRTS/1000) << ";\n";
		file << "timeDistEval = " << (int)(time_RRTS_req/1000) << ";\n";
		file << "timeDBS = " << (int)(time_DBS/1000) << ";\n";
#endif
		file.close();
	}
#ifdef __TIMING_INFO__
	ROS_INFO("LKH time consumption:\t\t\t%i ms", (int)(time_LKH/1000));
	ROS_INFO("Initial RRT* time consumption:\t\t%i ms", (int)(time_RRTS/1000));
	ROS_INFO("Distance evaluation time:\t\t%i ms", (int)(time_RRTS_req/1000));
#endif

	for(int i = 0; i<maxID; i++)
		delete plannerArray[i];
	delete[] plannerArray;
	delete[] reinitRRTs;
	reinitRRTs = NULL;
	clean();
}

void clean() {
	ROS_INFO("Variables cleaning");
	// tidy up
	if(!sys_t::obstacles.empty()) {
		for(typename std::list<reg_t*>::iterator it = sys_t::obstacles.begin(); it != sys_t::obstacles.end(); it++)
			delete (*it);
		sys_t::obstacles.clear();
	}
	
	delete[] VP;
	VP = NULL;
	maxID = count_files((pkgPath+"/viewpoints").c_str(), ".txt"); // Number of files in viewpoints
	if(lookupTable) {
		for(int i = 0; i < LOOKUPTABLE_SIZE; i++)
			delete[] lookupTable[i];
		delete[] lookupTable;
		lookupTable = NULL;
	}
	plannerArrayBool = false;
	initOK = false;
}

std::vector<nav_msgs::Path> * readSTLfile(std::string name) {
	std::vector<nav_msgs::Path> * mesh = new std::vector<nav_msgs::Path>;
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
	return mesh;
}

void publish_stl() {
	std::vector<nav_msgs::Path>* mesh = readSTLfile(ros::package::getPath("request")+"/meshes/asciiavion.stl");
	ros::Rate r(50.0);
	
	/* publish STL file to rviz */
	for(std::vector<nav_msgs::Path>::iterator it = mesh->begin(); it != mesh->end() && ros::ok(); it++) {
		stl_pub.publish(*it);
		r.sleep();
	}
}
