#include "own_cost_map.h"

Own_Cost_Map::Own_Cost_Map()
:load_map_flag_(0),inflation_radios_(0.5)
{
    pub_own_cost_map_ = n_.advertise<nav_msgs::OccupancyGrid>("/own_cost_map",2);
    sub_map_ = n_.subscribe("/map", 2, &Own_Cost_Map::mapCb,this);
    while(load_map_flag_ == 0)
    {
        ros::spinOnce();
    }
    dynamic_reconfigure::Server<point_cloud_test::point_cloud_processe_Config> server;
    dynamic_reconfigure::Server<point_cloud_test::point_cloud_processe_Config>::CallbackType f;
    f = boost::bind(&Own_Cost_Map::dynamicCb,this,_1,_2);
    server.setCallback(f);
    sub_points_state_ = n_.subscribe("/points_state",1,&Own_Cost_Map::pointsStateCb,this);

    ros::spin();
}

Own_Cost_Map::~Own_Cost_Map()
{}

void Own_Cost_Map::dynamicCb(point_cloud_test::point_cloud_processe_Config &config,uint32_t level)
{
    cout << "the inflation radius:" << config.inflation_layer_radius << endl;
    cout << "     position offset:" << config.position_offset << endl;
    inflation_radios_ = config.inflation_layer_radius;
    position_offset_ = config.position_offset;
    addInflationLayerToOriginMap();
}

void Own_Cost_Map::mapCb(nav_msgs::OccupancyGrid map)
{
    origin_map_ = map;
    load_map_flag_ = 1;

    // origin_map_.header.stamp = map.header.stamp;
    // origin_map_.header.frame_id = map.header.frame_id;
    // origin_map_.info.map_load_time = map.info.map_load_time;
    // origin_map_.info.resolution = map.info.resolution;
    // origin_map_.info.width = map.info.width;
    // origin_map_.info.height = map.info.height;
    // origin_map_.info.origin.position.x = map.info.origin.position.x;
    // origin_map_.info.origin.position.y = map.info.origin.position.y;
    // origin_map_.info.origin.position.z = map.info.origin.position.z;
    // origin_map_.info.origin.orientation.x = map.info.origin.orientation.x;
    // origin_map_.info.origin.orientation.y = map.info.origin.orientation.y;
    // origin_map_.info.origin.orientation.z = map.info.origin.orientation.z;
    // origin_map_.info.origin.orientation.w = map.info.origin.orientation.w;
    // origin_map_.data.resize(map.data.size());
    // for(int i = 0;i < map.data.size();i++)
    // {
    //     origin_map_.data[i] = map.data[i];
    // }
}

void Own_Cost_Map::addInflationLayerToOriginMap()
{
    new_map_ = origin_map_;
    for(int i = 0;i < new_map_.data.size();i++)
    {
        int x0 = i % new_map_.info.width;
        int y0 = i / new_map_.info.width;
        int x1 , y1;
        if(new_map_.data[i] >= 65)
        {
            int det_r = inflation_radios_ / new_map_.info.resolution;
            for(x1 = x0 - det_r;x1 < x0 + det_r;x1++)
            {
                int height1 = sqrt(det_r * det_r - (x1 - x0) * (x1 - x0));
                for(y1 = y0 - height1;y1 < y0 + height1;y1++)
                {
                    if((x1 >= 0) && (x1 <= new_map_.info.width) && (y1 >= 0) && (y1 <= new_map_.info.height))
                    {
                        int map_node_state = new_map_.data[y1 * new_map_.info.width + x1];
                        new_map_.data[y1 * new_map_.info.width + x1] = map_node_state >= 65 ? 100:50;
                    }
                }
            }
        }
    }
    cout << "accomplete to add the inflation layer." << endl;
}

void Own_Cost_Map::pointsStateCb(geometry_msgs::PoseArray msg)
{
    //origin_points_ = msg;
    nav_msgs::OccupancyGrid new_map = new_map_;
    for(int i = 1;i < msg.poses.size();i++)
    {
        double v_length = sqrt(msg.poses[i].orientation.x * msg.poses[i].orientation.x + msg.poses[i].orientation.y * msg.poses[i].orientation.y);
        int x0 = (msg.poses[i].position.x + position_offset_ * msg.poses[i].orientation.x / v_length - origin_map_.info.origin.position.x) / origin_map_.info.resolution;
        int y0 = (msg.poses[i].position.y + position_offset_ * msg.poses[i].orientation.y / v_length - origin_map_.info.origin.position.y) / origin_map_.info.resolution;
        int det_r = inflation_radios_ / new_map.info.resolution;
        for(int x1 = x0 - det_r;x1 < x0 + det_r;x1++)
        {
            int height1 = sqrt(det_r * det_r - (x1 - x0) * (x1 - x0));
            for(int y1 = y0 - height1;y1 < y0 + height1;y1++)
            {
                if((x1 >= 0) && (x1 <= new_map.info.width) && (y1 >= 0) && (y1 <= new_map.info.height))
                {
                    int map_node_state = new_map.data[y1 * new_map.info.width + x1];
                    new_map.data[y1 * new_map.info.width + x1] = map_node_state >= 65 ? 100:50;
                }
            }
        }
    }
    pub_own_cost_map_.publish(new_map);
}

double Own_Cost_Map::distanceOffset()
{
    double ans;
    return ans;
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "own_cost_map");

    Own_Cost_Map own_cost_map_test;
    
    return 0;
}
