#include <ros/ros.h>
#include <string.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <idmind_motorsboard/WheelsMB.h>

#define PRINTF_BLUE  	"\x1B[34m"
#define PRINTF_MAGENTA  "\x1B[35m"
#define PRINTF_CYAN  	"\x1B[36m"

#define LOOP_RATE 20


class Smoother
{
    public:

    Smoother(ros::NodeHandle *nh, float rate_){
        
        vx_prev = 0;
        wz_prev = 0;
    
        min_speed.resize(3);
        max_speed.resize(3);

        nh->param("/smoother/lin_max_speed", lin_max_vel, (float)0.8);
        nh->param("/smoother/lin_min_speed", lin_min_vel, (float)-0.8);
        nh->param("/smoother/lin_acc", lin_acc, (float)0.25);
        nh->param("/smoother/lin_decc", lin_decc, (float)0.4);
        nh->param("/smoother/angular_acc", ang_acc, (float)0.8);
        nh->param("/smoother/angular_max_speed", ang_max_speed, (float)1.5);
        nh->param("/smoother/exp_ct", exp_ct, (float)0.03);
        nh->param("/smoother/w_limit", w_limit, (float)50);
        nh->param("/smoother/debug",debug, (bool)false);
        nh->param("/smoother/scale_sensible_smoother", weightEnabled, (bool)false);
        nh->param("/smoother/security_step", sec_step, (float)10);
        nh->param("/smoother/base_width", base_width, (float)0.26);
        //You can select the in and out topics :)
        nh->param("/smoother/input_twist_topic", input_twist_topic, (std::string)"/cmd_vel_joy");
        nh->param("/smoother/output_wheelsMb_topic", output_wheelsMb_topic, (std::string)"/idmind_motors/set_velocities");
        
        twist_pre_smooth_sub = nh->subscribe<geometry_msgs::Twist>(input_twist_topic, 1, &Smoother::twistCb, this);
        wheelsMb_pub = nh->advertise<idmind_motorsboard::WheelsMB>(output_wheelsMb_topic,0);

        //Store original accelerations
        //TODO: Make original accelerations const 
        lin_acc_or = lin_acc;
        lin_decc_or = lin_decc;
        ang_acc_or = ang_acc;
        
        rate = rate_;
        ROS_INFO_COND(debug, PRINTF_CYAN"Lin max acc or: %.2f lin min decc or  %.2f", lin_acc_or, lin_decc_or);
        
    }
    
    void run(){
        smooth();
        convertToWheelsMb();
        wheelsMb_pub.publish(ret);
    }
    
    void twistCb(const geometry_msgs::Twist::ConstPtr &msg){
        ret = *msg;
    }   

    private:
    void convertToWheelsMb(){
        //Convert velocities to wheel linear velocity
        l_vel = -(ret.linear.x - ret.angular.z * base_width/2);
        r_vel = (ret.linear.x + ret.angular.z * base_width/2);
        wheels_msg.front_right=r_vel;
        wheels_msg.front_left = l_vel;

        wheels_msg.header.frame_id="odom";
        wheels_msg.header.stamp=ros::Time::now();
        wheels_msg.kinematics = "2wd";
    }
    void smooth(){
        
        //First cap the speeds
        ROS_INFO_COND(debug,PRINTF_CYAN "Before cap: [%.2f, %.2f]", ret.linear.x, ret.angular.z);
        
        cap_speed();
        
        ROS_INFO_COND(debug,PRINTF_CYAN "After  cap: [%.2f, %.2f]", ret.linear.x, ret.angular.z);
        
        if(weightEnabled)
            update_params();
        
        
        //Calculate the max and min speeds possibles in one step
        if(vx_prev > 0 || vx_prev == 0 ){
           
                min_speed[0] = vx_prev - lin_decc;
                max_speed[0] = vx_prev + lin_acc;
        }else{
           
                min_speed[0] = vx_prev - lin_acc;
                max_speed[0] = vx_prev + lin_decc;
        }
       

        min_speed[2] = wz_prev - ang_acc;
        max_speed[2] = wz_prev + ang_acc;

        //Ahora se checkea si salen de los bounds o no
        //x
        if( !( (ret.linear.x > min_speed[0] || ret.linear.x == min_speed[0]) && (ret.linear.x < max_speed[0] || ret.linear.x == max_speed[0]) ) ){
            if( ret.linear.x < min_speed[0])
                ret.linear.x = min_speed[0];
            
            if( ret.linear.x > max_speed[0])
                ret.linear.x = max_speed[0];
            
        }
        
        //z
        if(ret.angular.z > max_speed[2])
            ret.angular.z = max_speed[2];
        
        if(ret.angular.z < min_speed[2])
            ret.angular.z = min_speed[2];
        

        vx_prev = ret.linear.x;
        wz_prev = ret.angular.z;
        ROS_INFO_COND(debug,PRINTF_CYAN "Speed: Vx, Wz: [%.2f, %.2f]", vx_prev, wz_prev);
        
    }
    void update_params(){

        ROS_INFO_COND(debug,PRINTF_CYAN "Before Weight-corrected lin_acc,lin_decc,ang_acc: [%.2f, %.2f, %.2f] ", lin_acc, lin_decc, ang_acc);
        ROS_INFO_COND(debug,PRINTF_CYAN" Current weight: [%.2f] ",weight.data);
        
        if(weight.data > w_limit-10){
            lin_acc = exp_factor(lin_acc_or,w_limit-10);
            lin_decc = exp_factor(lin_decc_or,w_limit-10);
            ang_acc =  exp_factor(ang_acc_or,w_limit-10);;
        }else{
            lin_acc = exp_factor(lin_acc_or, weight.data);//fabs(lin_acc_or*(1-exp(-exp_ct*(w_limit-weight.data))));
            lin_decc = exp_factor(lin_decc_or,weight.data); //fabs(lin_decc_or*(1-exp(-exp_ct*(w_limit-weight.data))));
            ang_acc = exp_factor(ang_acc_or, weight.data); //fabs(ang_acc_or*(1-exp(-exp_ct*(w_limit-weight.data))));
        }
    
        ROS_INFO_COND(debug,PRINTF_CYAN "Weight-corrected lin_acc,lin_decc,ang_acc: [%.2f, %.2f, %.2f]", lin_acc, lin_decc, ang_acc);
        lin_decc/=rate;
        lin_acc/=rate;
        ang_acc/=rate;
    }
    inline float exp_factor(float var, float w){
        return var*(1-exp(-exp_ct*(w_limit-w)));
    }
    void cap_speed(){
        //x
        if(!(ret.linear.x < lin_max_vel && ret.linear.x > lin_min_vel)){
            if(ret.linear.x < 0){
                ret.linear.x = lin_min_vel;
            }else{
                ret.linear.x = lin_max_vel;
            }
        }
        
        //z
        if(fabs(ret.angular.z) > ang_max_speed){
            if(ret.angular.z < 0){
                ret.angular.z = -ang_max_speed;
            }else{
                ret.angular.z = ang_max_speed;
            }
        }
    }
    void emergency_stop(geometry_msgs::Twist vel){
        geometry_msgs::Twist ret;

        if(vel.linear.x != 0 ){
            if(vel.linear.x < 0.05){
                ret.linear.x = 0;
            }else{
               step = vel.linear.x/(sec_step*rate);
               step/=2;
               if(vel.linear.x<0){
                   ret.linear.x = vel.linear.x + step;
               }else{
                   ret.linear.x = vel.linear.x -step;
               }
            }
        }
        
        if(vel.angular.z !=0  ){
            if(vel.angular.z < 0.05){
                ret.angular.z = 0;
            }else{
               step = vel.angular.z/(sec_step*rate);
               step/=2;
               if(vel.angular.z<0){
                   ret.angular.z = vel.angular.z + step;
               }else{
                   ret.angular.z = vel.angular.z -step;
               }
            }
        }
        vx_prev = ret.linear.x;
        wz_prev = ret.angular.z;
        wheelsMb_pub.publish(ret);
    }


    /*   Variables  */

        /*  Input Parameters  */
    bool weightEnabled, debug;
    float exp_ct, w_limit;

    float lin_max_vel,lin_min_vel,lin_acc,lin_decc,ang_max_speed,ang_acc;
    //Original values are stored in _or variables
    //TODO: Make them const 
    float lin_acc_or,lin_decc_or,ang_acc_or;
    float sec_step;
    float rate;

    std::string input_twist_topic,output_wheelsMb_topic;

        /*  ROS objects */
    ros::Subscriber sim_weight_sub; //Subscriber to get weight (number) used to increase smooth
    std_msgs::Float32 weight; // TODO: Make it dynamically reconfigurable :)

    //Twist messages publisher and subscriber
    ros::Publisher wheelsMb_pub;
    geometry_msgs::Twist ret;

    ros::Subscriber twist_pre_smooth_sub;

    idmind_motorsboard::WheelsMB wheels_msg;
    float r_vel,l_vel;
    float base_width;
    //Variables used by the smoother
    std::vector<float> min_speed, max_speed;
    
    float vx_prev, wz_prev;
    float step;
};

int main(int argc, char** argv){

    ros::init(argc, argv, "smoother_node");
    ros::NodeHandle n;

   
    ros::Rate rate(20);

    Smoother smoother(&n,20);

    while(ros::ok()){
        ros::spinOnce();

        smoother.run();

        rate.sleep();
    }

    return 0;
}