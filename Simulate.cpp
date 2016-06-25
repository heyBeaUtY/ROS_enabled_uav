//Simulate

# include <ros/ros.h>
# include <geometry_msgs/Twist.h>
# include < geometry_msgs / PoseWithCovarianceStamped .h>
float goal [6][3]={{0 ,0 ,0} ,{1 ,1 ,0} ,{ -1 ,1 ,0} ,{ -1 

, -1 ,0} ,{1 , -1 ,0} ,{1 ,1 ,0}};
float tolerance = 0.05;

double kp = 0.5;
double ki = 0.0002;
double kd = 0.00005;

float w;

float error_x = 0;
float error_y = 0;
float error_z = 0;
float error_w = 0;
float prev_error_x = 0;
float prev_error_y = 0;
float prev_error_z = 0;
float prev_error_w = 0;
float rise = 1;
float nonstop = true ;
float proportional_x = 0;
float proportional_y = 0;
float proportional_z = 0;
float proportional_w = 0;
float integral_x = 0;
float integral_y = 0;
float integral_z = 0;
float integral_w = 0;
float derivative_x = 0;
float derivative_y = 0;
float derivative_z = 0;
float derivative_w = 0;
float action_x = 0;
float action_y = 0;
float action_z = 0;
float action_w = 0;

geometry_msgs :: Point real ;
geometry_msgs :: Twist twist ;

bool must_exit = false ;
int waypoint_number = 0;

void odoCallback ( const geometry_msgs :: 

PoseWithCovarianceStamped :: ConstPtr & msg )
{
  real .x=msg -> pose . pose . position .x;
  real .y=msg -> pose . pose . position .y;
  real .z=msg -> pose . pose . position .z;
  w=msg -> pose . pose . orientation .z;
  prev_error_x = error_x ;
  prev_error_y = error_y ;
  prev_error_z = error_z ;
  prev_error_w = error_w ;
  error_x = goal [ waypoint_number ][0] - real .x;
  error_y = goal [ waypoint_number ][1] - real .y;
  error_z = ( goal [ waypoint_number ][2] + 0.001 * rise ) 

- real .z;
  error_w = 0 - w;
 
  proportional_x = kp * error_x ;
  proportional_y = kp * error_y ;
  proportional_z = kp * error_z ;
  proportional_w = kp * error_w ;
  integral_x += ki * error_x ;
  integral_y += ki * error_y ;
  integral_z += ki * error_z ;
  integral_w += ki * error_w ;
  derivative_x = kd * ( error_x - prev_error_x );
  derivative_y = kd * ( error_y - prev_error_y );
  derivative_z = kd * ( error_z - prev_error_z );
  derivative_w = kd * ( error_w - prev_error_w );
  action_x = proportional_x + integral_x + derivative_x ;
  action_y = proportional_y + integral_y + derivative_y ;
  action_z = proportional_z + integral_z + derivative_z ;
  action_w = 10 * proportional_w + integral_w + 

derivative_w ;
 
  twist . linear .x = action_x ;
  twist . linear .y = action_y ;
  twist . linear .z = action_z ;
  twist . angular .z = action_w ;
  ROS_INFO (" Error X: %0.2 f \n", error_x );
  ROS_INFO (" Error Y: %0.2 f \n", error_y );
  ROS_INFO (" Error Z: %0.2 f \n", error_y );
  ROS_INFO ("W: %0.2 f \n", w);
  ROS_INFO (" Action X: %0.2 f \n", action_x );
  ROS_INFO (" Action Y: %0.2 f \n", action_y );
  ROS_INFO (" Action Z: %0.2 f \n", action_y );
  ROS_INFO (" Action W: %0.2 f \n", action_y );
  ROS_INFO ("Voy hacie el objetivo %d \n", waypoint_number 

+1) ;
 
  if (( fabs ( error_x ) < tolerance ) && ( fabs ( error_y 

) < tolerance )) {
    if ( must_exit == true )
    {
    twist . linear .x = 0;
    twist . linear .y = 0;
    twist . linear .z = 0;
    twist . angular .z = 0;
    exit (0) ;
    }
   else
    {
     waypoint_number += 1;
     rise += 1;
    }
  }

if ( waypoint_number == ( sizeof ( goal )/ sizeof ( goal 

[0]) ))
   {
    if ( nonstop )
    {
     waypoint_number = 1;
    }
    else
     {
      waypoint_number = 0;
      must_exit = true ;
     }
    }
}

int main ( int argc , char ** argv )
{
  ros :: init (argc , argv , " test_viapoint ");
  ros :: NodeHandle np;
  ros :: NodeHandle nh;
  ros :: Publisher pub_vel = nh. advertise < geometry_msgs 

:: Twist >(" cmd_vel ", 1);
  ros :: Subscriber sub = np. subscribe (" poseupdate ", 

1000 , odoCallback );
  ros :: Rate loop_rate (10) ;
  int count = 0;
 
  while ( ros :: ok ())
    {
     pub_vel.publish ( twist );
     ros :: spinOnce ();
     loop_rate.sleep ();
     ++ count ;
}
 }
