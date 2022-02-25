#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "/home/lizhi/old_ubuntu/rrt_star_planner/src/rrt_star_planner/include/rrt_star_planner/rrtstarplan.h"//这个文件路径可能有问/home/lizhi/download codes/rrtstar_planner/src/rrtstar_planner/include/
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <time.h>
#include <pluginlib/class_list_macros.h>  
#include <Eigen/Dense>
#include <cstdlib>
#include <cstddef>

#define success false
#define running true
#define PI 3.1415926

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrtstar_planner::RRT, nav_core::BaseGlobalPlanner)

namespace rrtstar_planner{
    bool status=running;

    using namespace std;
    using costmap_2d::NO_INFORMATION;
    using costmap_2d::FREE_SPACE;

RRT::RRT()
{

}

RRT::RRT(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 
{
    initialize(name, costmap_ros);
}

void RRT::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if(!initialized_)
        {
            costmap_ros_ = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
            costmap_ = costmap_ros_->getCostmap(); //get the costmap_ from costmap_ros_
            footprint = costmap_ros_->getRobotFootprint();
            rrt_publisher = pn.advertise<visualization_msgs::Marker> ("path_planner_rrt",1000);

        // initialize other planner parameters
        /*ros::NodeHandle private_nh("~/" + name);
        private_nh.param("step_size", step_size_, costmap_->getResolution());
        private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);*/
            world_model_ = new base_local_planner::CostmapModel(*costmap_);

            initialized_ = true;
        }
        else
        {
            ROS_WARN("This planner has already been initialized... doing nothing");
        }
    }

void RRT::initNode(RRT::rrtNode &newNode, const geometry_msgs::PoseStamped& start)
{
    newNode.posX=start.pose.position.x;
    newNode.posY=start.pose.position.y;
    newNode.parentID = 0;
    newNode.nodeID = 0;
    newNode.cost=0;
    rrtTree.push_back(newNode);
}

/**
* Returns the current RRT tree
* @return RRT Tree
*/
vector<RRT::rrtNode> RRT::getTree()
{
    return rrtTree;
}


vector<RRT::rrtNode> RRT::getNearestNeighbor(int tempNodeID)//不要忘记补头文件；目的是在一定范围内找到新节点附近的近邻节点
{
    double win_r=0.15;
    vector<RRT::rrtNode> rrtNeighbor;
    for(int i=0;i<getTreeSize();i++)
    {
        if ( getEuclideanDistance(getPosX(tempNodeID),getPosY(tempNodeID),getPosX(i),getPosY(i))<=win_r)//新节点和每一个节点
        {
            //这里应该是一个专门用来存储近邻节点的向量 
            rrtNeighbor.push_back(getNode(i));
        }   
    }
    return rrtNeighbor;
}

/**
* For setting the rrtTree to the inputTree
* @param rrtTree
*/
void RRT::setTree(vector<RRT::rrtNode> input_rrtTree)
{
    rrtTree = input_rrtTree;
}

/**
* to get the number of nodes in the rrt Tree
* @return tree size
*/
int RRT::getTreeSize()
{
    return rrtTree.size();
}

/**
* adding a new node to the rrt Tree
*/
void RRT::addNewNode(RRT::rrtNode node)
{
    rrtTree.push_back(node);
}

void RRT::deleteNewNode()
{
    rrtTree.pop_back();
}

/**
* removing a node from the RRT Tree
* @return the removed tree
*/
RRT::rrtNode RRT::removeNode(int id)
{
    RRT::rrtNode tempNode = rrtTree[id];
    rrtTree.erase(rrtTree.begin()+id);
    return tempNode;
}

/**
* getting a specific node
* @param node id for the required node
* @return node in the rrtNode structure
*/
RRT::rrtNode RRT::getNode(int id)
{
    return rrtTree[id];
}

/**
* return a node from the rrt tree nearest to the given point
* @param X position in X cordinate
* @param Y position in Y cordinate
* @return nodeID of the nearest Node
*/
int RRT::getNearestNodeID(double X, double Y)
{
    int i, returnID;
    double distance = 9999, tempDistance;
    for(i=0; i<this->getTreeSize(); i++)
    {
        tempDistance = getEuclideanDistance(X,Y, getPosX(i),getPosY(i));
        if (tempDistance < distance)
        {
            distance = tempDistance;
            returnID = i;
        }
    }
    return returnID;
}

/**
* returns X coordinate of the given node
*/
double RRT::getPosX(int nodeID)
{
    return rrtTree[nodeID].posX;
}

/**
* returns Y coordinate of the given node
*/
double RRT::getPosY(int nodeID)
{
    return rrtTree[nodeID].posY;
}

/**
* set X coordinate of the given node
*/
void RRT::setPosX(int nodeID, double input_PosX)
{
    rrtTree[nodeID].posX = input_PosX;
}

/**
* set Y coordinate of the given node
*/
void RRT::setPosY(int nodeID, double input_PosY)
{
    rrtTree[nodeID].posY = input_PosY;
}

/**
* returns parentID of the given node
*/
RRT::rrtNode RRT::getParent(int id)
{
    return rrtTree[rrtTree[id].parentID];
}

/**
* set parentID of the given node
*/
void RRT::setParentID(int nodeID, int parentID)
{
    rrtTree[nodeID].parentID = parentID;
}

/**
* add a new childID to the children list of the given node
*/
void RRT::addChildID(int nodeID, int childID)
{
    rrtTree[nodeID].children.push_back(childID);
}

/**
* returns the children list of the given node
*/
vector<int> RRT::getChildren(int id)
{
    return rrtTree[id].children;
}

/**
* returns number of children of a given node
*/
int RRT::getChildrenSize(int nodeID)
{
    return rrtTree[nodeID].children.size();
}

/**
* returns euclidean distance between two set of X,Y coordinates
*/
double RRT::getEuclideanDistance(double sourceX, double sourceY, double destinationX, double destinationY)
{
    return sqrt(pow(destinationX - sourceX,2) + pow(destinationY - sourceY,2));
}

/**
* returns path from root to end node
* @param endNodeID of the end node
* @return path containing ID of member nodes in the vector form
*/
vector<int> RRT::getRootToEndPath(int endNodeID)
{
    vector<int> path;
    path.push_back(endNodeID);
    while(rrtTree[path.front()].nodeID != 0)//path.front()返回的是ID
    {
        std::cout<<rrtTree[path.front()].nodeID<<endl;
        path.insert(path.begin(),rrtTree[path.front()].parentID);//这里的ID有问题导致循环跳不出去
        //path.begin()是最后一个新节点的ID，随后插入的是前一个节点的父节点
    }
    return path;
}

/*
bool RRT::judgeangle2(RRT::rrtNode rrtNeighbor,int NeighborParent, int NeighborID, RRT::rrtNode tempNode,int tempNodeID)
{
    vector<double> n1,n2;
    n1.clear();n2.clear();
    if(NeighborParent==0)
    {
        n1.push_back(getPosX(tempNodeID) - getPosX(NeighborID));
        n1.push_back(getPosY(tempNodeID) - getPosY(NeighborID));
        n2.push_back(0.0001);
        n2.push_back(0.0001);
    }
    else
    {
        n1.push_back(getPosX(NeighborID)-getPosX(NeighborParent));
        n1.push_back(getPosY(NeighborID)-getPosY(NeighborParent));
        n2.push_back(getPosX(tempNodeID)-getPosX(NeighborID));
        n2.push_back(getPosY(tempNodeID)-getPosY(NeighborID));
    }
    double phy=acos((n1[0]*n2[0]+n1[1]*n2[1])/sqrt((pow(n1[0],2)+pow(n1[1],2))*(pow(n2[0],2)+pow(n2[1],2))));
    return (abs(phy)<=PI/6) ? true : false;
}*/

void RRT::initializeMarkers(visualization_msgs::Marker &sourcePoint,
    visualization_msgs::Marker &goalPoint,
    visualization_msgs::Marker &randomPoint,
    visualization_msgs::Marker &rrtTreeMarker,
    visualization_msgs::Marker &rrtTreeMarker1,
    visualization_msgs::Marker &rrtTreeMarker2,
    visualization_msgs::Marker &finalPath)//对于RRT*的标记来说，添加新的即可
    {
    //init headers
	sourcePoint.header.frame_id    = goalPoint.header.frame_id    = randomPoint.header.frame_id    = rrtTreeMarker.header.frame_id    = rrtTreeMarker1.header.frame_id    = rrtTreeMarker2.header.frame_id    =finalPath.header.frame_id    = "map";
	sourcePoint.header.stamp       = goalPoint.header.stamp       = randomPoint.header.stamp       = rrtTreeMarker.header.stamp       = rrtTreeMarker1.header.stamp       = rrtTreeMarker2.header.stamp       =finalPath.header.stamp       = ros::Time::now();
	sourcePoint.ns                 = goalPoint.ns                 = randomPoint.ns                 = rrtTreeMarker.ns                 = rrtTreeMarker1.ns                 = rrtTreeMarker2.ns                 =finalPath.ns                 = "map";
	sourcePoint.action             = goalPoint.action             = randomPoint.action             = rrtTreeMarker.action             = rrtTreeMarker1.action             = rrtTreeMarker2.action             =finalPath.action             = visualization_msgs::Marker::ADD;
	sourcePoint.pose.orientation.w = goalPoint.pose.orientation.w = randomPoint.pose.orientation.w = rrtTreeMarker.pose.orientation.w = rrtTreeMarker1.pose.orientation.w = rrtTreeMarker2.pose.orientation.w =finalPath.pose.orientation.w = 1.0;

    //setting id for each marker
    sourcePoint.id    = 0;
	goalPoint.id      = 1;
	randomPoint.id    = 2;
	rrtTreeMarker.id  = 3;
    finalPath.id      = 4;
    rrtTreeMarker1.id = 5;//这一行是新加的
    rrtTreeMarker2.id = 6;

	//defining types
	rrtTreeMarker.type                                    = visualization_msgs::Marker::LINE_LIST;
    rrtTreeMarker1.type                                   = visualization_msgs::Marker::LINE_LIST;
    rrtTreeMarker2.type                                   = visualization_msgs::Marker::LINE_LIST;
	finalPath.type                                        = visualization_msgs::Marker::LINE_STRIP;
	sourcePoint.type  = goalPoint.type = randomPoint.type = visualization_msgs::Marker::SPHERE;

	//setting scale
	rrtTreeMarker.scale.x = 0.04;
    rrtTreeMarker1.scale.x= 0.02;
    rrtTreeMarker2.scale.x= 0.02;
	finalPath.scale.x     = 0.01;
	sourcePoint.scale.x   = goalPoint.scale.x = randomPoint.scale.x = 2;
    sourcePoint.scale.y   = goalPoint.scale.y = randomPoint.scale.y = 2;
    sourcePoint.scale.z   = goalPoint.scale.z = randomPoint.scale.z = 1;

    //assigning colors
	sourcePoint.color.r   = 1.0f;
	goalPoint.color.g     = 1.0f;
    randomPoint.color.b   = 1.0f;

	rrtTreeMarker.color.r = 0.8f;
	rrtTreeMarker.color.g = 0.4f;

    rrtTreeMarker1.color.r = 0;
    rrtTreeMarker1.color.g = 1.0f;

    rrtTreeMarker2.color.r = 0;
    rrtTreeMarker2.color.g = 0;
    rrtTreeMarker2.color.b = 1.0f;

	finalPath.color.r = 0.2f;
	finalPath.color.g = 0.2f;
	finalPath.color.b = 1.0f;

	sourcePoint.color.a = goalPoint.color.a = randomPoint.color.a = rrtTreeMarker.color.a = rrtTreeMarker1.color.a = rrtTreeMarker2.color.a = finalPath.color.a = 1.0f;
    }

void RRT::generateTempPoint(RRT::rrtNode &tempNode,double goalX, double goalY,costmap_2d::Costmap2D* costmap_)
{
    float probability=0.2;
    double maxx = costmap_->getSizeInMetersX() - costmap_->getOriginX();
    double minx = costmap_->getOriginX();
    double maxy = costmap_->getSizeInMetersY() - costmap_->getOriginY();
    double miny = costmap_->getOriginY();

    if (rand()/double(RAND_MAX)<probability)
    {
        tempNode.posX=goalX;
        tempNode.posY=goalY;
        //std::cout<<"rand: "<<tempNode.posX<<" "<<tempNode.posY<<endl;
    }
    else
    {
        double x = double(rand())/double(RAND_MAX)*(maxx - minx) + minx;
        double y = double(rand())/double(RAND_MAX)*(maxy - miny) + miny;
        //int x = rand() % maxx ;
        //int y = rand() % maxy ;
        //std::cout<<"Random X: "<<x <<endl<<"Random Y: "<<y<<endl;
        tempNode.posX = x;
        tempNode.posY = y;
        //std::cout<<"rand: "<<tempNode.posX<<" "<<tempNode.posY<<endl;
    }
}

bool RRT::judgeangle1(RRT::rrtNode tempNode)
{
    int nearestNodeID = getNearestNodeID(tempNode.posX,tempNode.posY);
    RRT::rrtNode nearestNode = getNode(nearestNodeID);

    double theta = atan2(tempNode.posY - nearestNode.posY,tempNode.posX - nearestNode.posX);//这个角度意义不对
    vector<double> n1,n2;
    if(nearestNode.parentID==0)
    {
        n1.push_back(tempNode.posX - nearestNode.posX);
        n1.push_back(tempNode.posY - nearestNode.posY);
        n2.push_back(0.0001);
        n2.push_back(0.0001);
    }
    else
    {
        n1.push_back(tempNode.posX - nearestNode.posX);
        n1.push_back(tempNode.posY - nearestNode.posY);
        n2.push_back(nearestNode.posX-getPosX(nearestNode.parentID));
        n2.push_back(nearestNode.posY-getPosY(nearestNode.parentID));
    }
    
    double phy = acos((n1[0]*n2[0]+n1[1]*n2[1])/(sqrt(n1[0]*n1[0]+n1[1]*n1[1])*sqrt(n2[0]*n2[0]+n2[1]*n2[1])));
    return (abs(phy)<=PI) ? true : false;
}

bool RRT::addNewPointtoRRT(RRT::rrtNode &tempNode, double rrtStepSize)
//addNewPointtoRRT(RRT &myRRT, RRT::rrtNode &tempNode, int rrtStepSize, vector< vector<geometry_msgs::Point> > &obstArray)//传进该函数中的tempNode是随机点
{
    int nearestNodeID = getNearestNodeID(tempNode.posX,tempNode.posY);//由此启发：是否可以写一个myRRT.getNearestNeighbor?具体函数写在rrt.cpp中

    RRT::rrtNode nearestNode = getNode(nearestNodeID);//这里getNode 获得的节点是一个结构体包含很多信息

    double theta = atan2(tempNode.posY - nearestNode.posY,tempNode.posX - nearestNode.posX);

    //if(theta<=PI/4)//出现问题是因为如果不满足该条件，没有后续动作会强行链接上一次存储在Marker中的值
        tempNode.posX = nearestNode.posX + (rrtStepSize * cos(theta));//这里tempNode变成了新节点
        tempNode.posY = nearestNode.posY + (rrtStepSize * sin(theta));

    if(checkIfInsideBoundary(tempNode) && checkIfOutsideObstacles(tempNode))//checkIfOutsideObstacles(obstArray,tempNode))
    {
        tempNode.parentID = nearestNodeID;
        tempNode.nodeID = rrtTree.size();
        //myRRT.addNewNode(tempNode);//这里tempNode表示新节点，今后RRT×的操作就基于这个新节点
        tempNode.cost=sqrt(pow(nearestNode.posX - tempNode.posX,2) + pow(nearestNode.posY - tempNode.posY,2))+\
                      nearestNode.cost;//计算每个新节点的代价,私有成员不能随意调用
                      //std::cout<<"tempNode.cost= "<<tempNode.cost<<endl;//////////////////////////////////////////
                      //std::cout<<"tempNode.posX: "<<tempNode.posX<<"  tempNode.posY"<<tempNode.posY<<endl;
       // rrtTree.push_back(tempNode);//这里tempNode表示新节点，今后RRT×的操作就基于这个新节点
       rrtTree.push_back(tempNode);
    
        return true;
    }
    else
        return false;
}


bool RRT::checkIfInsideBoundary(RRT::rrtNode &tempNode)
{
    if(tempNode.posX < costmap_->getOriginX() || tempNode.posY < costmap_->getOriginY()  \
    || tempNode.posX > costmap_->getSizeInMetersX() - costmap_->getOriginX() \
    || tempNode.posY > costmap_->getSizeInMetersY() - costmap_->getOriginY() ) 
    return false;
    else return true;
}

bool RRT::checkIfOutsideObstacles(RRT::rrtNode tempNode)
{
    unsigned int gridx,gridy;
    unsigned char* grid = costmap_->getCharMap();
    if(costmap_->worldToMap(tempNode.posX, tempNode.posY, gridx, gridy))
    {     
        int index = costmap_->getIndex(gridx, gridy);
        if(grid[index]!=FREE_SPACE&&grid[index]!=NO_INFORMATION)
        {
                return false;
        }
        else
        return true;
    }
    else
    return false;
}

void RRT::addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker, RRT::rrtNode &tempNode)//针对RRT*来说另外写一个函数
{//这个函数实际上就是画图

geometry_msgs::Point point;

point.x = tempNode.posX;//这里的tempNode 是新生成的节点
point.y = tempNode.posY;
point.z = 0;
rrtTreeMarker.points.push_back(point);

RRT::rrtNode parentNode = getParent(tempNode.nodeID);

point.x = parentNode.posX;
point.y = parentNode.posY;
point.z = 0;

rrtTreeMarker.points.push_back(point);//之所以把新生成的节点和其父节点都加入到rrtTreeMarker，是因为visualization_msgs::Marker LINE_LIST 的性质，链接两个新加入的节点
}


void RRTStarprocess1(visualization_msgs::Marker &rrtTreeMarker1, RRT::rrtNode &q_min,RRT::rrtNode &tempNode)
{
    geometry_msgs::Point point;

    point.x = tempNode.posX;//这里的tempNode 是新生成的节点
    point.y = tempNode.posY;
    point.z = 0;
    rrtTreeMarker1.points.push_back(point);

    point.x = q_min.posX;
    point.y = q_min.posY;
    point.z = 0;
    rrtTreeMarker1.points.push_back(point);
}

void RRTStarprocess2(visualization_msgs::Marker &rrtTreeMarker2, RRT::rrtNode &q_min1, RRT::rrtNode &tempNode)
{
    geometry_msgs::Point point;

    point.x=tempNode.posX;
    point.y=tempNode.posY;
    point.z=0;
    rrtTreeMarker2.points.push_back(point);

    point.x=q_min1.posX;
    point.y=q_min1.posY;
    point.z=0;
    rrtTreeMarker2.points.push_back(point);
}


bool RRT::checkNodetoGoal(double X, double Y, RRT::rrtNode &tempNode)
{
    double distance = sqrt(pow(X-tempNode.posX,2)+pow(Y-tempNode.posY,2));
    //std::cout<<"距离终点的距离： "<<distance<<endl;
    if(distance < 0.05)
    {
        return true;
    }
    return false;
}


void RRT::setFinalPathData(vector< vector<int> > &rrtPaths,  int i, visualization_msgs::Marker &finalpath, double goalX, double goalY)
{
    RRT::rrtNode tempNode;
    geometry_msgs::Point point;
    for(int j=0; j<rrtPaths[i].size();j++)
    {
        tempNode = getNode(rrtPaths[i][j]);

        point.x = tempNode.posX;
        point.y = tempNode.posY;
        point.z = 0;

        finalpath.points.push_back(point);
    }

    point.x = goalX;
    point.y = goalY;
    finalpath.points.push_back(point);
}

double caldistance(double sourceX, double sourceY, double destinationX, double destinationY)
{
    return sqrt(pow(destinationX - sourceX,2) + pow(destinationY - sourceY,2));
}





bool RRT::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan )
{

    plan.clear();
    rrtTree.clear();

	//defining markers
    visualization_msgs::Marker sourcePoint;
    visualization_msgs::Marker goalPoint;
    visualization_msgs::Marker randomPoint;
    visualization_msgs::Marker rrtTreeMarker;
    visualization_msgs::Marker rrtTreeMarker1;
    visualization_msgs::Marker rrtTreeMarker2;
    visualization_msgs::Marker finalPath;

    initializeMarkers(sourcePoint, goalPoint, randomPoint, rrtTreeMarker, rrtTreeMarker1, rrtTreeMarker2, finalPath);

    srand(time(NULL));

    RRT::rrtNode newNode;

    initNode(newNode,start);//初始化节点

    double rrtStepSize = 0.05;//

    vector< vector<int> > rrtPaths;
    vector<int> path;
    int rrtPathLimit = 1;

    int shortestPathLength = 9999;
    int shortestPath = -1;

    RRT::rrtNode tempNode;

    //vector< vector<geometry_msgs::Point> >  obstacleList = getObstacles();
    bool addNodeResult = false, nodeToGoal = false;
    std::cout<<"start: "<<start.pose.position.x<<"  "<<start.pose.position.y<<endl;
    std::cout<<"goal: "<<goal.pose.position.x<<"  "<<goal.pose.position.y<<endl;
    status=running;
    while(ros::ok() && status)
    {
        double goalX=goal.pose.position.x;
        double goalY=goal.pose.position.y;
        if(rrtPaths.size() < rrtPathLimit)
        {
            do
            {
                generateTempPoint(tempNode,goalX,goalY,costmap_);
                //std::cout<<"tempnode generated"<<endl;
                judgeangle1(tempNode);
            }
            while(!judgeangle1(tempNode));
            
            addNodeResult = addNewPointtoRRT(tempNode,rrtStepSize);//addNewPointtoRRT(myRRT,tempNode,rrtStepSize,obstacleList);

            if(addNodeResult)
            {
                // std::cout<<"tempnode accepted"<<endl;
                addBranchtoRRTTree(rrtTreeMarker,tempNode);//RRT*应该是写在这一行的下一行（先确确实实的加入了新节点，才能由新节点找近邻节点）
               // std::cout<<"tempnode printed"<<endl;

//RRT*核心部分
            //由此启发：是否可以写一个myRRT.getNearestNeighbor?具体函数写在rrt.cpp中
//重选父节点过程
                vector<RRT::rrtNode> rrtNeighbor=getNearestNeighbor(tempNode.nodeID);//如果出错，确认这里的tempNode是否是新生成的节点，以及赋值语句的正确性
                        //std::cout<<"rrtNeighbor.size= "<<rrtNeighbor.size()<<endl;///////////////////////////////////////////////
                int nearestNodeID = getNearestNodeID(tempNode.posX,tempNode.posY);
                RRT::rrtNode nearestNode = getNode(nearestNodeID);
                RRT::rrtNode q_min=nearestNode;
                double C_min=tempNode.cost;//！！！注意之前还没有任何关于cost的操作,tempNode.cost
                for(int k=0;k<rrtNeighbor.size();k++)
                {
                    if(checkIfInsideBoundary(rrtNeighbor[k]) && checkIfOutsideObstacles(rrtNeighbor[k])\
                        &&rrtNeighbor[k].cost+\
                        caldistance(tempNode.posX,tempNode.posY,rrtNeighbor[k].posX,rrtNeighbor[k].posY)<C_min) 
                        {
                            //if(myRRT.judgeangle(rrtNeighbor[k],rrtNeighbor[k].parentID,rrtNeighbor[k].nodeID,\
                              tempNode,tempNode.nodeID))
                              //{
                                  q_min = rrtNeighbor[k];
                                  C_min = rrtNeighbor[k].cost+\
                                  caldistance(tempNode.posX,tempNode.posY,rrtNeighbor[k].posX,rrtNeighbor[k].posY);
                              //}
                        }
                }
                RRTStarprocess1(rrtTreeMarker1,q_min,tempNode);//这里仿照addBranchtoRRTTree写一个新的划线函数,rrtTreeMarker1是新的标记
                tempNode.cost=C_min;

                //找到问题的原因是：有时候parentID等于nodeID
                if(tempNode.nodeID!=q_min.nodeID)
                    tempNode.parentID=q_min.nodeID;//RRT*第一过程核心，重选父节点////////////////////////

                rrtTree.pop_back();
                rrtTree.push_back(tempNode);//这里tempNode表示新节点，今后RRT×的操作就基于这个新节点
                //std::cout<<"x坐标和y坐标"<<myRRT.getNode(tempNode.nodeID).posX<<" , "<<myRRT.getNode(tempNode.nodeID).posY<<endl;
                //std::cout<<"tempNode.nodeID: "<<tempNode.nodeID<<"    tempNode.parentID: "<<tempNode.parentID<<endl;


//重布线过程
                //没有必要重新再找临近节点了，因为每针对一个新节点，在上一过程已经找到了临近节点
                RRT::rrtNode q_min1=nearestNode;
                double C_min1=tempNode.cost+caldistance(q_min.posX,q_min.posY,tempNode.posX,tempNode.posY);
                
                for(int k=0;k<rrtNeighbor.size();k++)
                {
                        if(checkIfInsideBoundary(rrtNeighbor[k]) && checkIfOutsideObstacles(rrtNeighbor[k])\
                        &&(tempNode.cost+caldistance(tempNode.posX,tempNode.posY,rrtNeighbor[k].posX,rrtNeighbor[k].posY))\
                         < rrtNeighbor[k].cost)//在这个过程中只有当rrtNeighbor==tempNode时才满足条件，因此才会出现q_min1==tempNode的情况
                        {
                            //if(myRRT.judgeangle(tempNode,tempNode.parentID,tempNode.nodeID,rrtNeighbor[k],rrtNeighbor[k].nodeID))//路径绕不出去的原因在这个角度判断条件
                            //{
                                q_min1=rrtNeighbor[k];
                                C_min1=tempNode.cost+\
                                caldistance(tempNode.posX,tempNode.posY,rrtNeighbor[k].posX,rrtNeighbor[k].posY);
                                if(q_min1.nodeID!=tempNode.nodeID)
                                   setParentID(q_min1.nodeID, tempNode.nodeID);
                            //}
                        }
                }
                RRTStarprocess2(rrtTreeMarker2,q_min1,tempNode);
//判断终止
               nodeToGoal = checkNodetoGoal(goalX, goalY,tempNode);
                //std::cout<<"nodeToGoal的值： "<<nodeToGoal<<endl;
                if(nodeToGoal)
                {
                    //std::cout<<"最后一个点的ID： "<<tempNode.nodeID<<endl;
                    path = getRootToEndPath(tempNode.nodeID);//path向量是一个包含组成最终路径的节点ID的向量,这里没有goal的ID
                    rrtPaths.push_back(path);
                    std::cout<<"New Path Found. Total paths "<<rrtPaths.size()<<endl;
                    int i=0;
                    do
                    {
                        geometry_msgs::PoseStamped pose=start;
                        RRT::rrtNode pathNode=getNode(path[i]);
                        pose.pose.position.x=pathNode.posX;
                        pose.pose.position.y=pathNode.posY;
                        plan.push_back(pose);
                        i++;
                    }while(path[i]!=tempNode.nodeID);
                    plan.push_back(goal);
                    //ros::Duration(10).sleep();
                    //std::cout<<"got Root Path"<<endl;
                }
            }
        }
    else //if(rrtPaths.size() >= rrtPathLimit)
        {
            status = success;
            std::cout<<"Finding Optimal Path"<<endl;
            
            for(int i=0; i<rrtPaths.size();i++)
            {
                if(rrtPaths[i].size() < shortestPath)
                {
                    shortestPath = i;
                    shortestPathLength = rrtPaths[i].size();
                }
            }
            setFinalPathData(rrtPaths, shortestPath, finalPath, goalX, goalY);//该函数的作用是画出路径
            rrt_publisher.publish(finalPath);
            return  true;
        }
        rrt_publisher.publish(rrtTreeMarker);
        rrt_publisher.publish(rrtTreeMarker1);
        rrt_publisher.publish(rrtTreeMarker2);
        //ros::spinOnce();
        //ros::Duration(0.01).sleep();
    }
}
}

