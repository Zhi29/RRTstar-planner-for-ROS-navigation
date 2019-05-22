#ifndef rrtstarplan_h
#define rrtstarplan_h

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <vector>

using std::string;
using namespace std;
namespace rrtstar_planner {
	class RRT : public nav_core::BaseGlobalPlanner {

        public:

            RRT();
            RRT(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            RRT(double input_PosX, double input_PosY);

            struct rrtNode{
                int nodeID;
                double posX;
                double posY;
                int parentID;
                double cost;
                //double theta;
                vector<int> children;
            };

            vector<rrtNode> getTree();
            vector<rrtNode> getNearestNeighbor(int tempNodeID);//这一行是新加的
            void setTree(vector<rrtNode> input_rrtTree);
            int getTreeSize();

            void addNewNode(rrtNode node);
            void deleteNewNode();
            rrtNode removeNode(int nodeID);
            rrtNode getNode(int nodeID);

            double getPosX(int nodeID);
            double getPosY(int nodeID);
            void setPosX(int nodeID, double input_PosX);
            void setPosY(int nodeID, double input_PosY);

            rrtNode getParent(int nodeID);
            void setParentID(int nodeID, int parentID);

            void addChildID(int nodeID, int childID);
            vector<int> getChildren(int nodeID);
            int getChildrenSize(int nodeID);

            int getNearestNodeID(double X, double Y);
            vector<int> getRootToEndPath(int endNodeID);

            bool judgeangle1(RRT myRRT, rrtNode tempNode);
            //bool judgeangle2(rrtNode rrtNeighbor,int NeighborParent,int NeighborID,rrtNode tempNode,int tempNodeID);

            void initializeMarkers(visualization_msgs::Marker &sourcePoint,
                                    visualization_msgs::Marker &goalPoint,
                                    visualization_msgs::Marker &randomPoint,
                                    visualization_msgs::Marker &rrtTreeMarker,
                                    visualization_msgs::Marker &rrtTreeMarker1,
                                    visualization_msgs::Marker &rrtTreeMarker2,
                                    visualization_msgs::Marker &finalPath);
            
            void initNode(RRT::rrtNode &newNode, const geometry_msgs::PoseStamped& start);

            void generateTempPoint(RRT::rrtNode &tempNode,double goalX, double goalY,costmap_2d::Costmap2D* costmap_);
            bool judgeangle1(RRT::rrtNode tempNode);
            bool addNewPointtoRRT(RRT::rrtNode &tempNode, double rrtStepSize);
            bool checkIfInsideBoundary(RRT::rrtNode &tempNode);
            bool checkIfOutsideObstacles(RRT::rrtNode tempNode);
            void addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker, RRT::rrtNode &tempNode);
            bool checkNodetoGoal(double X, double Y, RRT::rrtNode &tempNode);
            void setFinalPathData(vector< vector<int> > &rrtPaths,  int i, visualization_msgs::Marker &finalpath, double goalX, double goalY);


            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan
               );
            
            vector<rrtNode> rrtTree;
            ros::NodeHandle pn;

        private:
            double getEuclideanDistance(double sourceX, double sourceY, double destinationX, double destinationY);
            bool initialized_;
            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D* costmap_;
            base_local_planner::WorldModel* world_model_;
            std::vector<geometry_msgs::Point> footprint;
	};
};

#endif