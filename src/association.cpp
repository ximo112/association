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
    association.header.stamp = msg->header.stamp;

    for(int i = 0; i < (int)msg->channels[1].values.size(); i++){
      association.points[i].x = msg->points[i].x;
      association.points[i].y = msg->points[i].y;
      association.channels[0].values[i] = msg->channels[0].values[i];
      
      association.channels[1].values[i] = msg->channels[1].values[i];

      if(cluster_num < msg->channels[1].values[i] + 1){
        cluster_num = msg->channels[1].values[i] + 1;
      }
    }
    cluster_x.resize(cluster_num);
    cluster_y.resize(cluster_num);
    cluster_size.resize(cluster_num);
    cluster_association.resize(cluster_num);
    for(int i = 0; i < cluster_num; i++){
      cluster_x[i] = 0;
      cluster_y[i] = 0;
      count = 0;
      for(int j = 0; j < (int)msg->points.size(); j++){
        if(i == msg->channels[1].values[j]){
          cluster_x[i] += msg->points[j].x;
          cluster_y[i] += msg->points[j].y;
          count += 1;

          cluster_association[i] = i;
        }
      }
      cluster_x[i] = cluster_x[i] / count;
      cluster_y[i] = cluster_y[i] / count;

    }
    for(int i = 0; i < cluster_num; i++){
      distance_max = 0;
      cluster_size[i] = 0;
      for(int j = 0; j < (int)msg->points.size(); j++){
        if(i == association.channels[1].values[j]){
          distance = hypotf(cluster_x[i] - association.points[j].x, cluster_y[i] - association.points[j].y);
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
    Matrix3f cov;
    Vector3f ave;
    Vector3f variable;
    Vector3f variable_ave;
    int i, j, k, count, alpha = 0;
    float log_f[cluster_num], ave_scalar, log_f_likelihood[cluster_num_old], likelihood_value[cluster_num];

    for(i = 0; i < cluster_num; i++){
      cluster_association[i] = -1;
      likelihood_value[i] = 0;
    }

    for(i = 0; i < association_num_max; i++){
      if(cluster_x_old[i] == 0 && cluster_y_old[i] == 0 && cluster_size_old[i] == 0){
      }else{
        //
        ROS_INFO("old:%d, cluster_x_old:%f, cluster_y_old:%f, cluster_size_old:%f", i, cluster_x_old[i], cluster_y_old[i], cluster_size_old[i]);

        cov = Matrix3f::Zero();
        ave = Vector3f::Zero();
        variable = Vector3f::Zero();
        variable_ave = Vector3f::Zero();
        for(j = 0; j < cluster_num; j++){
          cov(0, 0) += pow(cluster_x_old[i] - cluster_x[j], 2);
          cov(1, 1) += pow(cluster_y_old[i] - cluster_y[j], 2);
          cov(2, 2) += pow(cluster_size_old[i] - cluster_size[j], 2);
          cov(1, 0) += (cluster_y_old[i] - cluster_y[j]) * (cluster_x_old[i] - cluster_x[j]);
          cov(2, 0) += (cluster_size_old[i] - cluster_size[j]) * (cluster_x_old[i] - cluster_x[j]);
          cov(2, 1) += (cluster_size_old[i] - cluster_size[j]) * (cluster_y_old[i] - cluster_y[j]);
        }
        cov(0, 0) = cov(0, 0) / cluster_num;
        cov(1, 1) = cov(1, 1) / cluster_num;
        cov(2, 2) = cov(2, 2) / cluster_num;
        cov(1, 0) = cov(1, 0) / cluster_num;
        cov(2, 0) = cov(2, 0) / cluster_num;
        cov(2, 1) = cov(2, 1) / cluster_num;
        cov(0, 1) = cov(1, 0);
        cov(0, 2) = cov(2, 0);
        cov(1, 2) = cov(2, 1);

        ave(0) = cluster_x_old[i];
        ave(1) = cluster_y_old[i];
        ave(2) = cluster_size_old[i];
        log_f_likelihood[i] = -1;

        for(j = 0; j < cluster_num; j++){
          log_f[j] = hypotf(cluster_x[j] - cluster_x_old[i], cluster_y[j] - cluster_y_old[i]);
          if(log_f_likelihood[i] > log_f[j] || log_f_likelihood[i] == -1){
            log_f_likelihood[i] = log_f[j];
          }

          /*
             variable(0) = cluster_x[j];
             variable(1) = cluster_y[j];
             variable(2) = cluster_size[j];
             variable_ave = variable - ave;
             ave_scalar = variable_ave.transpose() * cov.inverse() * variable_ave;

          //多変量正規分布
          log_f[j] = log(exp(- ave_scalar / 2) / sqrt(pow(2 * M_PI, 3) * cov.determinant()));

          if(log_f_likelihood[i] < log_f[j] || log_f_likelihood[i] == false){
          log_f_likelihood[i] = log_f[j];
          }*/
        }

        for(j = 0; j < cluster_num; j++){
          if(log_f_likelihood[i] == log_f[j]){
            if(cluster_association[j] == -1){
              likelihood_value[j] = log_f_likelihood[i];
              cluster_association[j] = i;
            }else{
              if(likelihood_value[j] > log_f_likelihood[i]){
                likelihood_value[j] = log_f_likelihood[i];
                cluster_association[j] = i;
              }else{
                cluster_association[j] = -1;
              }
            }
            break;
          }
        }
      }
    }
    for(i = 0; i < cluster_num; i++){
      if(cluster_association[i] == -1){
        for(j = 0; j < cluster_num; j++){
          for(k = 0; k < cluster_num; k++){
            if(j == cluster_association[k]){
              break;
            }else if(k == cluster_num -1){
              cluster_association[i] = j;
            }
          }
        }
      }
    }
    //
    for(i = 0; i < cluster_num; i++){
      ROS_INFO("now:%d, cluster_x:%f, cluster_y:%f, cluster_size:%f, cluster_association:%d", i, cluster_x[i], cluster_y[i], cluster_size[i], cluster_association[i]);
    }
  }

  void run(){
    check = false;
    int a = 0;//
    while(ros::ok()){
      if(check == true){
        if(cluster_num_old != 0){
          likelihood();
          for(int i = 0; i < cluster_num; i++){
            for(int j = 0; j < (int)association.channels[1].values.size(); j++){
              if(i == association.channels[1].values[j]){
                association.channels[1].values[j] = cluster_association[i];
              }
            }
          }
          associatied_obstacle_pub.publish(association);
        }
        //
        cluster_num_old = cluster_num;
        association_num_max = 0;
        for(int i = 0; i < cluster_num_old; i++){
          if(association_num_max < cluster_association[i] + 1){
            association_num_max = cluster_association[i] + 1;
          }
        }
        cluster_x_old.clear();
        cluster_y_old.clear();
        cluster_size_old.clear();
        cluster_x_old.resize(association_num_max);
        cluster_y_old.resize(association_num_max);
        cluster_size_old.resize(association_num_max);
        for(int i = 0; i < cluster_num_old; i++){
          cluster_x_old[cluster_association[i]] = cluster_x[i];
          cluster_y_old[cluster_association[i]] = cluster_y[i];
          cluster_size_old[cluster_association[i]] = cluster_size[i];
        }
/*
        if(a > 1){
          break;
        }
        a += 1;
*/
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
  vector<float> cluster_x, cluster_y, cluster_size;
  vector<float> cluster_x_old, cluster_y_old, cluster_size_old;
  int cluster_num, cluster_num_old, check, association_num_max;

};

int main(int argc, char **argv){
  ros::init(argc, argv, "association");
  Association dynamic_obstacle_association;
  dynamic_obstacle_association.run();

  return 0;
}
