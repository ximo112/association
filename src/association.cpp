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
    objects_sub = nh.subscribe("objects", 10, &Association::objectsCallBack, this);
    associatied_obstacle_pub = nh.advertise<sensor_msgs::PointCloud>("associatied_obstacle", 1000);
  }
  void objectsCallBack(const sensor_msgs::PointCloud::ConstPtr& msg){
    callback = true;
    int count;
    float distance_max, distance;
    cluster_num = 0;
    association.points.clear();
    association.channels.clear();
    association.points.resize((int)msg->points.size());
    association.channels.resize((int)msg->channels.size());
    association.channels[0].name = msg->channels[0].name;
    association.channels[0].values.resize((int)msg->channels[0].values.size());
    association.header.frame_id = msg->header.frame_id;
    association.header.stamp = ros::Time::now();
    cluster_numbering_now.clear();
    cluster_numbering_now.resize((int)msg->channels[0].values.size());

    for(int i = 0; i < (int)msg->points.size(); i++){
      association.points[i].x = msg->points[i].x;
      association.points[i].y = msg->points[i].y;
      association.channels[0].values[i] = msg->channels[0].values[i];

      cluster_numbering_now[i] = msg->channels[0].values[i];

      if(cluster_num < msg->channels[0].values[i] + 1){
        cluster_num = msg->channels[0].values[i] + 1;
      }
    }
    cluster_x.clear();
    cluster_y.clear();
    cluster_association.clear();
    cluster_x.resize(cluster_num);
    cluster_y.resize(cluster_num);
    cluster_association.resize(cluster_num);
    for(int i = 0; i < cluster_num; i++){
      cluster_x[i] = msg->points[i].x;
      cluster_y[i] = msg->points[i].y;

      cluster_association[i] = i;
    }
  }

  void likelihood(){
    Matrix2f cov;
    Vector2f ave;
    Vector2f variable;
    Vector2f variable_ave;
    int i, j, k, count, alpha = 0, association_num;
    float log_f[cluster_num], ave_scalar, log_f_likelihood, likelihood_value[cluster_num], diff[cluster_num], diff_min;

    for(i = 0; i < cluster_num; i++){
      cluster_association[i] = -1;
      likelihood_value[i] = -FLT_MAX;
    }

    for(i = 0; i < association_num_max; i++){
      if(cluster_x_old[i] == 0 && cluster_y_old[i] == 0){
      }else{
        //
        ROS_INFO("old:%d, cluster_x_old:%f, cluster_y_old:%f", i, cluster_x_old[i], cluster_y_old[i]);

        cov = Matrix2f::Zero();
        ave = Vector2f::Zero();
        variable = Vector2f::Zero();
        variable_ave = Vector2f::Zero();
        for(j = 0; j < cluster_num; j++){
          cov(0, 0) += pow(cluster_x_old[i] - cluster_x[j], 2);
          cov(1, 1) += pow(cluster_y_old[i] - cluster_y[j], 2);
          cov(1, 0) += (cluster_y_old[i] - cluster_y[j]) * (cluster_x_old[i] - cluster_x[j]);
        }
        cov(0, 0) = cov(0, 0) / cluster_num;
        cov(1, 1) = cov(1, 1) / cluster_num;
        cov(1, 0) = cov(1, 0) / cluster_num;
        cov(0, 1) = cov(1, 0);

        ave(0) = cluster_x_old[i];
        ave(1) = cluster_y_old[i];

        log_f_likelihood = -FLT_MAX;
        for(j = 0; j < cluster_num; j++){

          variable(0) = cluster_x[j];
          variable(1) = cluster_y[j];
          variable_ave = variable - ave;
          ave_scalar = variable_ave.transpose() * cov.inverse() * variable_ave;

          //多変量正規分布
          log_f[j] = log(exp(- ave_scalar / 2) / sqrt(pow(2 * M_PI, 2) * cov.determinant()));

          //
          ROS_INFO("log_f:%f, x:%f, y:%f", log_f[j], cluster_x[j], cluster_y[j]);

          if(log_f_likelihood < log_f[j]){
            log_f_likelihood = log_f[j];
            association_num = j;
          }
        }
        //
        ROS_INFO("aso_num:%d, log_f_likelihood:%f", association_num, log_f_likelihood);

        if(log_f_likelihood != -FLT_MAX){

          if(cluster_association[association_num] == -1){
            cluster_association[association_num] = i;
            likelihood_value[association_num] = log_f_likelihood;
          }else{
            if(likelihood_value[association_num] < log_f_likelihood){
              cluster_association[association_num] = i;
              likelihood_value[association_num] = log_f_likelihood;
            }
          }

        }else{

          diff_min = -1;
          for(j = 0; j < cluster_num; j++){
            diff[j] = hypotf(cluster_x[j] - cluster_x_old[i], cluster_y[j] - cluster_y_old[i]);
            if(diff_min > diff[j] || diff_min == -1){
              diff_min = diff[j];
              association_num = j;
            }
          }
          cluster_association[association_num] = i;
          likelihood_value[association_num] = FLT_MAX;
        }

      }
    }
    //
    for(j = 0; j < cluster_num; j++){
      ROS_INFO("now:%d, old:%d, diff:%f", j, cluster_association[j], likelihood_value[j]);
    }

    for(i = 0; i < cluster_num; i++){
      if(cluster_association[i] == -1){
        for(j = 0; j < cluster_num; j++){
          for(k = 0; k < cluster_num; k++){
            if(j == cluster_association[k]){
              break;
            }
          }
          if(k == cluster_num){
            cluster_association[i] = j;
            break;
          }
        }
      }
    }
  }

  void run(){
    callback = false;
    float hoge_x, hoge_y;
    while(ros::ok()){
      if(callback == true){
        callback = false;
        if(cluster_num_old != 0){
          likelihood();
          for(int i = 0; i < cluster_num; i++){
            association.channels[0].values[i] = cluster_association[i];
          }
          //
          ROS_INFO("-----");

          associatied_obstacle_pub.publish(association);
        }
        cluster_num_old = cluster_num;
        association_num_max = 0;
        for(int i = 0; i < cluster_num_old; i++){
          if(association_num_max < cluster_association[i] + 1){
            association_num_max = cluster_association[i] + 1;
          }
        }
        cluster_x_old.clear();
        cluster_y_old.clear();
        cluster_x_old.resize(association_num_max);
        cluster_y_old.resize(association_num_max);
        for(int i = 0; i < cluster_num_old; i++){
          cluster_x_old[cluster_association[i]] = cluster_x[i];
          cluster_y_old[cluster_association[i]] = cluster_y[i];
        }
      }
      ros::spinOnce();
    }
  }
private:
  ros::NodeHandle nh;
  ros::Subscriber objects_sub;
  ros::Publisher associatied_obstacle_pub;
  sensor_msgs::PointCloud association;
  vector<int> cluster_association, cluster_numbering_now;
  vector<float> cluster_x, cluster_y;
  vector<float> cluster_x_old, cluster_y_old;
  int cluster_num, cluster_num_old, association_num_max, callback;

};

int main(int argc, char **argv){
  ros::init(argc, argv, "association");
  Association dynamic_obstacle_association;
  dynamic_obstacle_association.run();

  return 0;
}
