#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Dense>
#include <math.h>
#include <sstream>

using namespace Eigen;
using namespace std;

class Association{
public:
  Association(){
    obstacle_clustering_sub = nh.subscribe("obstacle_clustering", 10, &Association::obstacle_clusteringCallBack, this);
    associatied_obstacle_pub = nh.advertise<sensor_msgs::PointCloud>("associatied_obstacle", 1000);
  }
  void obstacle_clusteringCallBack(const sensor_msgs::PointCloud::ConstPtr& msg){
    int count;
    float distance_max, distance;
    cluster_num = 0;
    check = true;
    association.points.resize((int)msg->points.size());
    association.channels.resize((int)msg->channels.size());
    association.channels[0].name = msg->channels[0].name;
    association.channels[0].values.resize((int)msg->channels[0].values.size());
    association.channels[1].name = msg->channels[1].name;
    association.channels[1].values.resize((int)msg->channels[1].values.size());
    association.header.frame_id = msg->header.frame_id;

    for(int i = 0; i < (int)msg->channels[1].values.size(); i++){
      association.points[i].x = msg->points[i].x;
      association.points[i].y = msg->points[i].y;
      association.channels[0].values[i] = msg->channels[0].values[i];

      if(cluster_num_old == 0){
        association.channels[1].values[i] = msg->channels[1].values[i];
      }

      if(cluster_num < msg->channels[1].values[i] + 1){
        cluster_num = msg->channels[1].values[i] + 1;
      }
    }
    cluster_x.resize(cluster_num);
    cluster_y.resize(cluster_num);
    cluster_intensity.resize(cluster_num);
    cluster_size.resize(cluster_num);
    cluster_association.resize(cluster_num);
    for(int i = 0; i < cluster_num; i++){
      cluster_x[i] = 0;
      cluster_y[i] = 0;
      cluster_intensity[i] = 0;
      count = 0;
      for(int j = 0; j < (int)msg->points.size(); j++){
        if(i == (int)msg->channels[1].values[j]){
          cluster_x[i] += msg->points[j].x;
          cluster_y[i] += msg->points[j].y;
          cluster_intensity[i] += msg->channels[0].values[j];
          count += 1;
        }
      }
      cluster_x[i] = cluster_x[i] / count;
      cluster_y[i] = cluster_y[i] / count;
      cluster_intensity[i] = cluster_intensity[i] / count;

    }
    for(int i = 0; i < cluster_num; i++){
      distance_max = 0;
      cluster_size[i] = 0;
      for(int j = 0; j < (int)msg->points.size(); j++){
        if(i == (int)msg->channels[1].values[j]){
          distance = hypotf(cluster_x[i] - msg->points[j].x, cluster_y[i] - msg->points[j].y);
          if(distance_max < distance){
            distance_max = distance;
          }
        }
      }
      if(distance_max < 0.025){
        distance_max = 0.025;
      }
      cluster_size[i] = pow(distance_max, 2) * M_PI;
    }
  }

  void likelihood(){
    Matrix4f cov;
    Vector4f ave;
    Vector4f variable;
    Vector4f variable_ave;
    int count, alpha = 1;
    float log_f[cluster_num], ave_scalar, log_f_likelihood;

    for(int i = 0; i < cluster_num; i++){
      cluster_association[i] = cluster_num_old;
    }

    for(int i = 0; i < cluster_num_old; i++){
      cov = Matrix4f::Zero();
      ave = Vector4f::Zero();
      variable = Vector4f::Zero();
      variable_ave = Vector4f::Zero();
      for(int j = 0; j < cluster_num; j++){
        cov(0, 0) += pow(cluster_x_old[i] - cluster_x[j], 2);
        cov(1, 1) += pow(cluster_y_old[i] - cluster_y[j], 2);
        cov(2, 2) += pow(cluster_intensity_old[i] - cluster_intensity[j], 2);
        cov(3, 3) += pow(cluster_size_old[i] - cluster_size[j], 2);
        cov(1, 0) += (cluster_y_old[i] - cluster_y[j]) * (cluster_x_old[i] - cluster_x[j]);
        cov(2, 0) += (cluster_intensity_old[i] - cluster_intensity[j]) * (cluster_x_old[i] - cluster_x[j]);
        cov(2, 1) += (cluster_intensity_old[i] - cluster_intensity[j]) * (cluster_y_old[i] - cluster_y[j]);
        cov(3, 0) += (cluster_size_old[i] - cluster_size[j]) * (cluster_x_old[i] - cluster_x[j]);
        cov(3, 1) += (cluster_size_old[i] - cluster_size[j]) * (cluster_y_old[i] - cluster_y[j]);
        cov(3, 2) += (cluster_size_old[i] - cluster_size[j]) * (cluster_intensity_old[i] - cluster_intensity[j]);
      }
      cov(0, 0) = cov(0, 0) / cluster_num;
      cov(1, 1) = cov(1, 1) / cluster_num;
      cov(2, 2) = cov(2, 2) / cluster_num;
      cov(3, 3) = cov(3, 3) / cluster_num;
      cov(1, 0) = cov(1, 0) / cluster_num;
      cov(2, 0) = cov(2, 0) / cluster_num;
      cov(2, 1) = cov(2, 1) / cluster_num;
      cov(3, 0) = cov(3, 0) / cluster_num;
      cov(3, 1) = cov(3, 1) / cluster_num;
      cov(3, 2) = cov(3, 2) / cluster_num;
      cov(0, 1) = cov(1, 0);
      cov(0, 2) = cov(2, 0);
      cov(1, 2) = cov(2, 1);
      cov(0, 3) = cov(3, 0);
      cov(1, 3) = cov(3, 1);
      cov(2, 3) = cov(3, 2);

      ave(0) = cluster_x_old[i];
      ave(1) = cluster_y_old[i];
      ave(2) = cluster_intensity_old[i];
      ave(3) = cluster_size_old[i];
      log_f_likelihood = false;
      for(int j = 0; j < cluster_num; j++){
        variable(0) = cluster_x[j];
        variable(1) = cluster_y[j];
        variable(2) = cluster_intensity[j];
        variable(3) = cluster_size[j];
        variable_ave = variable - ave;
        ave_scalar = variable_ave.transpose() * cov.inverse() * variable_ave;

        //多変量正規分布
        log_f[j] = log(exp(- ave_scalar / 2) / sqrt(pow(2 * M_PI, 4) * cov.determinant()));

        if(log_f_likelihood < log_f[j] || log_f_likelihood == false){
          log_f_likelihood = log_f[j];
        }
      }
      if(isnan(log_f_likelihood)){
        cluster_association[i] = i;
      }
      for(int j = 0; j < cluster_num; j++){
        if(log_f_likelihood == log_f[j] && !isnan(log_f[j])){
          cluster_association[j] = i;
        }
      }
    }
    for(int i = 0; i < cluster_num; i++){
      if(cluster_association[i] == cluster_num_old){
        cluster_association[i] = cluster_num_old + alpha;
        alpha += 1;
      }
    }
  }

  void run(){
    check = false;
    while(ros::ok()){
      if(check == true){
        if(cluster_num_old != 0){
          likelihood();
          for(int i = 0; i < cluster_num; i++){
            for(int j = 0; j < (int)association.channels[1].values.size(); j++){
              if(i == association.channels[1].values[j]){
                //
                ROS_INFO("num:%d, clus_asso:%d", i, cluster_association[i]);
                association.channels[1].values[j] = cluster_association[i];
              }
            }
          }
          //
          ROS_INFO("=====");
          association.header.stamp = ros::Time::now();
          associatied_obstacle_pub.publish(association);
        }
        cluster_num_old = cluster_num;
        cluster_x_old.resize(cluster_num_old);
        cluster_y_old.resize(cluster_num_old);
        cluster_intensity_old.resize(cluster_num_old);
        cluster_size_old.resize(cluster_num_old);
        for(int i = 0; i < cluster_num_old; i++){
          cluster_x_old[i] = cluster_x[i];
          cluster_y_old[i] = cluster_y[i];
          cluster_intensity_old[i] = cluster_intensity[i];
          cluster_size_old[i] = cluster_size[i];
        }
      }
      ros::spinOnce();
    }
  }
private:
  ros::NodeHandle nh;
  ros::Subscriber obstacle_clustering_sub;
  ros::Publisher associatied_obstacle_pub;
  sensor_msgs::PointCloud association;
  vector<int> cluster_association;
  vector<float> cluster_x, cluster_y, cluster_intensity, cluster_size;
  vector<float> cluster_x_old, cluster_y_old, cluster_intensity_old, cluster_size_old;
  int cluster_num, cluster_num_old, check;

};

int main(int argc, char **argv){
  ros::init(argc, argv, "association");
  Association dynamic_obstacle_association;
  dynamic_obstacle_association.run();

  return 0;
}
