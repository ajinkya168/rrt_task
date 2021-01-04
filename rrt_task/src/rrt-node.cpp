#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include "rrtImpl.h"

RRT initRRT();

Node runRRT(ros::Publisher, int);

void display_map(ros::Publisher marker_pub);

void start_and_end(ros::Publisher marker_pub);

void addEdge(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher, bool);

void drawFinalPath(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher marker_pub);


Node init, goal;
static RRT rrt = initRRT();
static float goal_bias = 0.5;
static float sigma = 0.9;
static bool success = false;

static int init_x = 0;
static int init_y = 2.0;

static int goal_x = 0;
static int goal_y = -2.0;

static std::vector<visualization_msgs::Marker> obsVec;
static int path_node_index;
static bool pn_index_initialized = false;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_rrt");
    static ros::NodeHandle n;
    static ros::Publisher marker_pub=n.advertise<visualization_msgs::Marker>("visualization_marker",10);
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("some_chatter", 10);

    ros::Rate loop_rate(20);
    int frame_count=0;


    while(ros::ok())
    {
    start_and_end(marker_pub);
        
    if(!success)
    {
        Node next_node = runRRT(marker_pub,frame_count);
        geometry_msgs::Point next_point = next_node.point;

        if((rrt.getEuclideanDistance(next_point, goal.point) <= 1) && frame_count > 2)
        {
            addEdge(next_point, goal.point, marker_pub, false);
            next_node.children.push_back(goal);
            goal.parentId = next_node.id;
            success = true;
        }

    }

    if(success)
    {
        std::vector<Node> pathNodes;
        std::vector<Node> allNodes = rrt.getNodesList();

        pathNodes.push_back(goal);
        int tempParentId = goal.parentId;
        while(tempParentId != init.parentId)
        {
            for(int i = allNodes.size() - 1; i >=0; i--)
            {
                Node tempNode = allNodes[i];
                if((tempNode.id) == tempParentId)
                {
                    pathNodes.push_back(tempNode);
                    tempParentId = tempNode.parentId;
                }
            }
        }

            Node next;
            Node curr;
            for (int i = pathNodes.size() - 2; i >= 0; i--) {
                curr = pathNodes[i];
                next = pathNodes[i + 1];
                drawFinalPath(curr.point, next.point, marker_pub);
            }
    }


        while (marker_pub.getNumSubscribers() < 1) 
        {
            if (!ros::ok()) 
            {
                return 0;
            }
            ROS_WARN_ONCE("Please run Rviz in another terminal.");
            sleep(1);
        }

        ros::spinOnce();
        loop_rate.sleep();
        ++frame_count;

    }

    
    return 0;
}

void display_map(ros::Publisher marker_pub)
{
 visualization_msgs::Marker obj1, obj2;
 obj1.ns = obj2.ns = "obstacles";
 obj1.type = obj2.type = visualization_msgs::Marker::CUBE;
 obj1.header.frame_id = obj2.header.frame_id="map";
 obj1.header.stamp = obj2.header.stamp=ros::Time::now();
 obj1.lifetime = obj2.lifetime=ros::Duration();
 obj1.action = obj2.action=visualization_msgs::Marker::ADD;
 obj1.id = 0;
 obj2.id = 1;

 obj1.scale.x=3;obj2.scale.x=1;
 obj1.scale.y=1;obj2.scale.y=3;
 obj1.scale.z=obj2.scale.z=0.25;

obj1.pose.position.x=0;
obj1.pose.position.y=0;
obj1.pose.position.z=0.25;
obj1.pose.orientation.x=obj1.pose.orientation.y=obj1.pose.orientation.z=0.0;
obj1.pose.orientation.w=1;
obj1.color.a=1;
obj1.color.r=obj1.color.g=obj1.color.b=6.6f;

obj2.pose.position.x=1.5;
obj2.pose.position.y=1.0;
obj2.pose.position.z=0.25;
obj2.pose.orientation.x=obj2.pose.orientation.y=obj2.pose.orientation.z=0.0;
obj2.pose.orientation.w=1;
obj2.color.a=1;
obj2.color.r=obj2.color.g=obj2.color.b=6.6f;


marker_pub.publish(obj1);
marker_pub.publish(obj2);
obsVec.push_back(obj1);
obsVec.push_back(obj2);
}


void start_and_end(ros::Publisher marker_pub)
{
 visualization_msgs::Marker start,end;

 start.ns = end.ns = "start/end";
 start.type = end.type = visualization_msgs::Marker::POINTS;
 start.header.frame_id = end.header.frame_id="map";
 start.header.stamp = end.header.stamp=ros::Time::now();
 start.lifetime = end.lifetime=ros::Duration();
 start.action = end.action= visualization_msgs::Marker::ADD;
 start.id = 0;
 end.id = 1;
 start.color.a = 1.0f;
 start.color.g = 1.0f;
 start.scale.x = start.scale.y = 0.1;
 end.scale.x = end.scale.y = 0.1;
 end.color.a = 1.0f;
 end.color.r = 1.0f;

geometry_msgs::Point ps,pe;

ps.x=init_x;
ps.y=init_y;
pe.x=goal_x;
pe.y=goal_y;
start.points.push_back(ps);
end.points.push_back(pe);

 marker_pub.publish(start); 
 marker_pub.publish(end);

 
 display_map(marker_pub);

};

RRT initRRT()
{
 init.point.x = init_x;
 init.point.y = init_y; 
 init.parentId = -2;
 init.id = -1;
 goal.point.x = goal_x;
 goal.point.y = goal_y;
 goal.id= 10000;
 RRT rrt(init, goal, sigma, 10, -10, 10, -10);
 return rrt;

}


Node runRRT(ros::Publisher marker_pub, int frameid) {
    geometry_msgs::Point rand_point = rrt.getRandomConfig();
    geometry_msgs::Point tempP;
    tempP.x = 0;
    tempP.y = 1.5;
    Node rand_node(tempP);
    Node next_node(tempP);
    Node nearest_node = rrt.getNearestNode(rand_point);

    //decide whether to extend toward the goal or a random point
    double r = rand() / (double) RAND_MAX;
    if (r < goal_bias) {
        next_node = rrt.expand(nearest_node, goal, obsVec, frameid);
    } else {
        rand_node.point = rand_point;
        next_node = rrt.expand(nearest_node, rand_node, obsVec, frameid);
    }

    if ((next_node.point.x != nearest_node.point.x) && (next_node.point.y != nearest_node.point.y)) {
        std::cout << "Rand_config: \n" << rand_point << "nearest_node: \n" << nearest_node.point << "next_node: \n"
                  << (next_node).point << "\n\n";
        addEdge(nearest_node.point, next_node.point, marker_pub, false);
    }
    return next_node;
}



void drawFinalPath(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher marker_pub) {
    static visualization_msgs::Marker edge;
    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.ns = "finalPath";
    edge.id = 4;
    edge.action = visualization_msgs::Marker::ADD;
    edge.pose.orientation.w = 1;

    edge.scale.x = 0.04;

    edge.color.g = edge.color.r = 1;
    edge.color.a = 1.0;

    edge.points.push_back(p1);
    edge.points.push_back(p2);

    marker_pub.publish(edge);
}

void addEdge(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher marker_pub, bool isFinal) {
    static visualization_msgs::Marker edge, vertex;
    vertex.type = visualization_msgs::Marker::POINTS;
    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.ns = "edges";
    edge.id = 3;
    edge.action = visualization_msgs::Marker::ADD;
    edge.pose.orientation.w = 1;

    edge.scale.x = 0.02;
    if (!isFinal) {
        edge.color.r = 1.0;
    } else {
        edge.color.g = edge.color.r = 1;
    }
    edge.color.a = 1.0;

    edge.points.push_back(p1);
    edge.points.push_back(p2);

    marker_pub.publish(edge);
}
