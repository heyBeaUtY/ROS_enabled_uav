//Simulation with real input
# include <ros / ros .h>
# include " roscopter /RC.h"
# include <roscopter / Attitude .h>
# include < sensor_msgs / Imu .h>
# include < geometry_msgs / Quaternion .h>
# include < geometry_msgs / Twist .h>
# include < geometry_msgs / PoseWithCovarianceStamped .h>
# include <signal .h>
# include <termios .h>
# include <stdio .h>
# include " boost / thread / mutex . hpp "
# include " boost / thread / thread . hpp"

# define PI 3.14159
# define KEYCODE_SPACE 0 x20

struct euler {
float roll ;
float pitch ;
float yaw ;
 } ;
geometry_msgs :: Twist vel ;
geometry_msgs :: Quaternion virtual_quaternion ;

euler real_angles ;
euler virtual_angles ;
float kp_p = 0.09;
float kp_y = 0.09;
float kp_r = 0.09;
float ki_p = 0.00004;
float ki_y = 0.00004;
float ki_r = 0.00004;
float kd_p = 0.00001;
float kd_y = 0.00001;
float kd_r = 0.00001;

float yaw_prev_error = 0;
float pitch_prev_error = 0;
float roll_prev_error = 0;
float yaw_error = 0;
float pitch_error = 0;
float roll_error = 0;

float yaw_action = 0;
float pitch_action = 0;
float roll_action = 0;

float yaw_proportional = 0;
float pitch_proportional = 0;
float roll_proportional = 0;

float yaw_integral = 0;
float pitch_integral = 0;
float roll_integral = 0;

float yaw_derivative = 0;
float pitch_derivative = 0;
float roll_derivative = 0;

float toDegrees ( float radians )
{
return radians *180/ PI;
}

float toRads ( float degrees )
{
return degrees *PI /180;
}

int diffAngle ( float target , float source )
{
int a;
target = (int) target ;
source = (int) source ;
a = target - source ;
a = (a + 180) % 360 - 180;
return a;
}

euler QuaternionToRoll ( float x, float y, float z, float w)
{
float test = x * y + z * w;
float roll , pitch , yaw ;
euler solution ;
if ( test > 0.499) { // singularity at north pole
    pitch = (2 * atan2 (x, w));
    yaw = (PI / 2);
    roll = 0;
    solution . roll = toDegrees ( roll );
    solution . pitch = toDegrees ( pitch );
    solution . yaw = toDegrees ( yaw );
    return solution ;
}
if ( test < -0.499) { // singularity at south pole
    pitch = (-2 * atan2 (x, w));
    yaw = (-PI / 2);
    roll = 0;
    solution . roll = toDegrees ( roll );
    solution . pitch = toDegrees ( pitch );
    solution . yaw = toDegrees ( yaw );
    return solution ;
    }

float sqx = x * x;
float sqy = y * y;
float sqz = z * z;
pitch = atan2 (2 * y * w - 2 * x * z, 1 - 2 * sqy - 2 * sqz);
yaw = asin (2 * test );
roll = atan2 (2 * x * w - 2 * y * z, 1 - 2 * sqx - 2 * sqz );
solution . roll = toDegrees ( roll );
solution . pitch = toDegrees ( pitch );
solution . yaw = toDegrees ( yaw);
return solution ;
}

void odoCallback ( const roscopter :: Attitude & msg ) {
	real_angles . roll = toDegrees ( msg . roll );
    real_angles . pitch = toDegrees ( msg. pitch );
    real_angles . yaw = toDegrees ( msg . yaw );
    yaw_prev_error = yaw_error ;
    yaw_error = diffAngle ( real_angles .yaw , virtual_angles . yaw );
    yaw_proportional = kp_y * yaw_error ;
    yaw_integral += ki_y * yaw_error
    yaw_derivative = kd_y * ( yaw_error - yaw_prev_error )
    yaw_action = yaw_proportional + yaw_integral + yaw_derivative
    pitch_prev_error = pitch_error ;
    pitch_error = real_angles . pitch - virtual_angles . pitch ;
    pitch_proportional = kp_p * pitch_error ;
    pitch_integral += ki_p * pitch_error
    pitch_derivative = kd_p * ( pitch_error - pitch_prev_error )
    pitch_action = pitch_proportional + pitch_integral + pitch_derivative
    roll_prev_error = roll_error ;
    roll_error = real_angles . roll - virtual_angles . roll ;
    roll_proportional = kp_r * roll_error ;
    roll_integral += ki_r * roll_error
    roll_derivative = kd_r * ( roll_error - roll_prev_error )
    roll_action = roll_proportional + roll_integral + roll_derivative
    vel . angular .z = yaw_action ;
    vel . linear .x = pitch_action ;
    vel . linear .y = roll_action ;
    printf (" Action : %0.2 f \n", yaw_action );
    printf (" Error : %0.2 f \n", yaw_error );
    printf (" Roll : %0.2 f \n", real_angles .yaw );
    printf (" Virtual roll : %0.2 f \n", virtual_angles . yaw );
}
 
void odoCallback2 ( const geometry_msgs :: PoseWithCovarianceStamped :: ConstPtr & msg ){
     virtual_quaternion .x=msg -> pose . pose . orientation .x;
     virtual_quaternion .y=msg -> pose . pose . orientation .y;
     virtual_quaternion .z=msg -> pose . pose . orientation .z;
     virtual_quaternion .w=msg -> pose . pose . orientation .w;
     virtual_angles = QuaternionToRoll ( virtual_quaternion .x, virtual_quaternion .y,
     virtual_quaternion .z, virtual_quaternion .w);
}
 
int main ( int argc , char ** argv )
{
  ros :: init (argc , argv , " simulate ");
  ros :: NodeHandle n;
  ros :: NodeHandle np;
  ros :: NodeHandle nh;
  ros :: Publisher pub_vel = n. advertise < geometry_msgs :: Twist >(" cmd_vel ", 1000) ;
  ros :: Subscriber attitude_sub = np. subscribe (" attitude ", 1000 , odoCallback );
  ros :: Subscriber imu_virtual_sub = nh. subscribe (" poseupdate ", 1000 , odoCallback2 );
 
  ros :: Rate loop_rate (10) ;
  int count = 0;
  while ( ros :: ok ())
   {  
     pub_vel . publish ( vel );
     ros :: spinOnce ();
     loop_rate . sleep ();
     ++ count ;
     }
return 0;
}
