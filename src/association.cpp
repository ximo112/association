#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Dense>
#include <math.h>
#include <sstream>
#define particle_max 10 //散布するパーティクルの数
#define range 0.02 //散布するパーティクルの範囲

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
    float distance_max, distance, uniform1, uniform2;
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
    }
    cluster_num = (int)msg->points.size();

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
    int i, j, k, count, alpha = 0, association_num, check;
    float ave_scalar, log_f_likelihood, likelihood_value[association_num_max];
    MatrixXf log_f;
    log_f = MatrixXf::Zero(cluster_num_old, particle_max);

    for(i = 0; i < cluster_num; i++){
      cluster_association[i] = -1;
    }
    for(i = 0; i < cluster_num_old; i++){
      likelihood_value[i] = -FLT_MAX;
    }

    for(i = 0; i < cluster_num; i++){
      ROS_INFO("now:%d, cluster_x:%f, cluster_y:%f", i, cluster_x[i], cluster_y[i]);////

      cov = Matrix2f::Zero();
      ave = Vector2f::Zero();
      variable = Vector2f::Zero();
      variable_ave = Vector2f::Zero();
      for(j = 0; j < cluster_num_old; j++){
        for(k = 0; k < particle_max; k++){
          cov(0, 0) += pow(cluster_x[i] - particle_x(j, k), 2);
          cov(1, 1) += pow(cluster_y[i] - particle_y(j, k), 2);
          cov(1, 0) += (cluster_x[i] - particle_x(j, k)) * (cluster_y[i] - particle_y(j, k));
        }
      }
      cov(0, 0) = cov(0, 0) / (cluster_num_old * particle_max);
      cov(1, 1) = cov(1, 1) / (cluster_num_old * particle_max);
      cov(1, 0) = cov(1, 0) / (cluster_num_old * particle_max);
      cov(0, 1) = cov(1, 0);

      ave(0) = cluster_x[i];
      ave(1) = cluster_y[i];

      log_f_likelihood = -FLT_MAX;
      for(j = 0; j < cluster_num_old; j++){
        for(k = 0; k < particle_max; k++){
          variable(0) = particle_x(j, k);
          variable(1) = particle_y(j, k);
          variable_ave = variable - ave;
          ave_scalar = variable_ave.transpose() * cov.inverse() * variable_ave;

          //多変量正規分布
          log_f(j, k) = log(exp(- ave_scalar / 2) / sqrt(pow(2 * M_PI, 2) * cov.determinant()));

          ROS_INFO("log_f:%f, x:%f, y:%f, num:%d", log_f(j, k), particle_x(j, k), particle_y(j, k), cluster_association_old[j]);

          if(log_f_likelihood < log_f(j, k)){
            log_f_likelihood = log_f(j, k);
            association_num = cluster_association_old[j];
          }
        }
      }

      ROS_INFO("log_f_likelihood:%f, num:%d", log_f_likelihood, association_num);

      check = false;
      for(j = 0; j < cluster_num; j++){
        if(association_num == cluster_association[j]){
          check = true;
        }
      }

      if(likelihood_value[association_num] == -FLT_MAX){
        cluster_association[i] = association_num;
        likelihood_value[association_num] = log_f_likelihood;
      }else if(likelihood_value[association_num] < log_f_likelihood){
        ROS_WARN("%f", log_f_likelihood);////
        for(j = 0; j < cluster_num; j++){
          if(association_num == cluster_association[j]){
            cluster_association[j] = -1;
          }
        }
        cluster_association[i] = association_num;
        likelihood_value[association_num] = log_f_likelihood;
      }
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
    for(i = 0; i < cluster_num; i++){
      ROS_INFO("now:%d, asso:%d", i, cluster_association[i]);////
    }
  }

  void run(){
    ros::Rate loop_rate(20);
    callback = false;
    cluster_num_old = 0;
    float hoge_x, hoge_y;
    while(ros::ok()){
      if(callback == true){
        callback = false;
        if(cluster_num_old != 0){
          //前回の観測結果と現在の観測結果の対応関係を尤度関数を用いて行う
          likelihood();
          for(int i = 0; i < cluster_num; i++){
            association.channels[0].values[i] = cluster_association[i];
          }

          associatied_obstacle_pub.publish(association);
        }

        cluster_num_old = cluster_num;
        association_num_max = 0;
        for(int i = 0; i < cluster_num_old; i++){
          if(association_num_max < cluster_association[i] + 1){
            association_num_max = cluster_association[i] + 1;
          }
        }
        //前回の観測結果より移動後の位置予測としてパーティクルを散布
        cluster_association_old.clear();
        cluster_association_old.resize(cluster_num_old);
        particle_x = MatrixXf::Zero(cluster_num_old, particle_max);
        particle_y = MatrixXf::Zero(cluster_num_old, particle_max);
        for(int i = 0; i < cluster_num_old; i++){
          for(int j = 0; j < particle_max; j++){
            float uniform1 = ((float)rand() + 1) / ((float)RAND_MAX + 2);
            float uniform2 = ((float)rand() + 1) / ((float)RAND_MAX + 2);
            particle_x(i, j) = sqrt(-2 * log(uniform1)) * cos(2 * M_PI * uniform2) * range + cluster_x[i];
            particle_y(i, j) = sqrt(-2 * log(uniform1)) * sin(2 * M_PI * uniform2) * range + cluster_y[i];
          }
          cluster_association_old[i] = cluster_association[i];
        }
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
private:
  ros::NodeHandle nh;
  ros::Subscriber objects_sub;
  ros::Publisher associatied_obstacle_pub;
  sensor_msgs::PointCloud association;
  vector<int> cluster_association, cluster_numbering_now, cluster_association_old;
  vector<float> cluster_x, cluster_y;
  int cluster_num, cluster_num_old, association_num_max, callback;
  MatrixXf particle_x, particle_y;

};

int main(int argc, char **argv){
  ros::init(argc, argv, "association");
  Association dynamic_obstacle_association;
  dynamic_obstacle_association.run();

  return 0;
}
