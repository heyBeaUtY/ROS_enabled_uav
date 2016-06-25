#!/ usr / bin / env python
from __future__ import print_function
import roslib ; roslib . load_manifest ('roscopter ')
import rospy
from std_msgs . msg import String
from sensor_msgs . msg import NavSatFix , NavSatStatus , Imu
import roscopter . msg
import sys , struct ,time ,os
import time
from optparse import OptionParser
parser = OptionParser ("go.py [ options ]")

parser . add_option ("-m"," --mode ", dest =" mode ",
help =" Working mode : waypoints , goup , rotation ", default =" waypoints ")
parser . add_option ("-w"," -- waypoints ", dest =" goal_waypoints ", type ='float ',
help =" Array with the waypoints [x1 ,y1 ,z1 ,x2 ,y2 ,z2 ...] ", default=[0 ,0 ,0])

parser . add_option ("-a"," -- altitude ", dest =" altitude ", type ='float ',
help =" Altitude goal ", default =1.0)
parser . add_option ("-r"," --angle ", dest =" goal_deg ", type ='int ',
help =" yaw angle goal ", default =180)

(opts , args ) = parser . parse_args ()

waypoint_number = 0

def rc( data ):
global security
global throttle
security = data . channel [5]
throttle = data . channel [2]

def landing ( rate ):
if rate < 1450 :
for x in range (rate , 1300 , -1):
for x2 in range (x -5,x+5):
channel = [0,0,x2 ,0 ,0 ,0 ,0 ,0]
pub_rc . publish ( channel )
time . sleep (0.05)
else :
for x in range (1450 , 1300 , -1):
for x2 in range (x -5,x+5):
channel = [0,0,x2 ,0 ,0 ,0 ,0 ,0]
pub_rc . publish ( channel )
time . sleep (0.05)
channel = [0 ,0 ,1000 ,0 ,0 ,0 ,0 ,0]
pub_rc . publish ( channel )

def security_vel ( channel ):

channel_sec = channel

if ( security < 1200) :
	print (" Changing to secure mode ")
    channel = [0 ,0 ,0 ,0 ,0 ,0 ,0 ,0]
    pub_rc . publish ( channel )
  elif (1200 < security < 1700) :
for i in range ( channel [2] -5 , channel [2]+5) :
    channel_sec [2] = i
    pub_rc . publish ( channel_sec )
  elif ( security > 1700) :
landing ( throttle )
     os. _exit (1)
  else :
for i in range (990 , 1010) :
         channel = [0,0 ,i ,0 ,0 ,0 ,0 ,0]
         pub_rc . publish ( channel )
         
if opts . mode == " waypoints ":
     def gps ( data ):
   global error_x
   global error_y
   global error_z
   global Integral_x
   global Integral_y
   global Integral_z

          x = data .x
          y = data .y
          z = data .z

   prev_error_x = error_x
   prev_error_y = error_y
   prev_error_z = error_z
   error_x = opts . goal_waypoints [3* waypoint_number +0] - x
   error_y = opts . goal_waypoints [3* waypoint_number +1] - y
   error_z = opts . goal_waypoints [3* waypoint_number +2] - z
   action_x = kp * error_x
   action_y = kp * error_y
   action_z = kp * error_z
   position = {'x':x, 'y':y, 'z':z}
   goal = {'gx ': opts . goal_waypoints [3*i+0] , 'gy ': opts . goal_waypoints [3* i+1] , 'gz ':
    opts . goal_waypoints [3* i +2]}
   error = {'ex ': error_x , 'ey ': error_y , 'ez ': error_z }
   action = {'ax ': action_x , 'ay ': action_y , 'az ': action_z }

   print (" Position : %(x) .0.4f, %(y) .0.4f, %(z) .0.4 f" % position )
   print (" Goal : %( gx) .0.4f, %( gy) .0.4f, %( gz) .0.4 f" % goal )
   print (" Error X, Y, Z: %( ex) .0.4f, %( ey) .0.4f, %( ez) .0.4 f" % error )
   print (" Actions X, Y, Z: %( ax) .0.4f, %( ay) .0.4f, %( az) .0.4 f" % action )
   
Proportional_x = kp * error_x
Proportional_y = kp * error_y
Proportional_z = kp * error_z

Integral_x += ki * error_x
Integral_y += ki * error_y
Integral_z += ki * error_z

Derivative_x = kd * ( error_x - prev_error_x )
Derivative_y = kd * ( error_y - prev_error_y )
Derivative_z = kd * ( error_z - prev_error_z )

action_x = Proportional_x + Integral_x + Derivative_x
action_y = Proportional_y + Integral_y + Derivative_y
action_z = Proportional_z + Integral_z + Derivative_z
       rc_action_roll = 1500 + action_y
       rc_action_pitch = 1500 + action_x
       rc_action_throttle = 1000 + action_z
       channel = [ rc_action_roll , rc_action_pitch , rc_action_throttle ,0 ,0 ,0 ,0 ,0]
 
       security_vel ( channel )

      global waypoint_number
      if error_x < tolerance and error_y < tolerance and error_z < tolerance :
         if must_exit == 1:
        landing ( throttle )
    os. _exit (1)
      else
          waypoint_number += 1

if waypoint_number == ( len ( opts . goal_waypoints ) / 3):
   waypoint_number = 0
   must_exit = 1
 
   kp = 2
   ki = 0.1
   kd = 0.07
   error = 0
   Integral = 0
   tolerance = 0.1
   must_exit = 0
   gps_sub = rospy . Subscriber ("gps", NavSatFix , gps )
elif opts . mode == " goup ":
    def vfr_hud ( data ):
  global error
  global Integral
        global security
        
        altitude = data .alt
        
  prev_error = error
       error = goal - altitude
       action = kp * error
  Proportional = kp * error
  Integral += ki * error
  Derivative = kd * ( error - prev_error )
  
  action = Proportional + Integral + Derivative
  
        rc_action = 1000 + action
        channel = [0,0, rc_action ,0 ,0 ,0 ,0 ,0]

  security_vel ( channel )
 
    kp = 2
    ki = 0.1
    kd = 0.07
    error = 0
    Integral = 0
    goal = opts . altitude
    vfr_hud_sub = rospy . Subscriber (" vfr_hud ", roscopter .msg. VFR_HUD , vfr_hud )
  
  elif opts . mode == " rotation ":
      def attitude ( data ):
      	          global error
                  global Integral
                  global throttle
   
                  yaw = data . yaw * 180 / 3.14
  
                  prev_error = error
                  error = goal - yaw
                  error = ( error + 180) % 360 - 180 # only in yaw
                  Proportional = kp * error
                  print (" Error : %0.4 f"% error )
                  print (" Action : %0.4 f"% Proportional )
                  print (" Yaw : %0.4 f"% yaw )
                  print (" Security : %0.4 f"% security )
                  
                  Integral += ki * error
                  Derivative = kd * ( error - prev_error )
 
                  action = Proportional + Integral + Derivative
 
            rc_action = 1480 + action
                  channel = [0 ,0 ,1300 , rc_action ,0 ,0 ,0 ,0]
                  channel = [0,0, security , rc_action ,0 ,0 ,0 ,0]
  
            security_vel ( channel )
 
                  f = open ('yaw_data ','a')
                  ticks = time . time ()
                  to_file = {'time ': ticks , 'yaw ': yaw , 'goal ': goal , 'error ': error , 'prev_error '
                                    : prev_error , ' Proportional ': Proportional , 'Integral ': Integral , 'Derivative ': Derivative }
                  f. write ("%( time )d %( yaw) 0.4 f %( goal ) 0.4 f %( error ) 0.4 f %( prev_error ) 0.4 f %(
                      Proportional )0.4f %( Integral ) 0.4 f %( Derivative ) 0.4f \n"% to_file )
              kp = 2 # 1.75
              ki = 0.1
              kd = 0.07
              error = 0
              Integral = 0
              goal = opts . goal_deg #in rads
              attitude_sub = rospy . Subscriber (" attitude ", roscopter . msg . Attitude , attitude )
              
security = 0
pub_rc = rospy . Publisher ('send_rc ', roscopter .msg .RC)
rc_sub = rospy . Subscriber ("rc", roscopter . msg .RC , rc)
              
 def mainloop ():
    
	pub = rospy . Publisher (' rolling_demo ', String )
    rospy . init_node (' rolling_demo ')
  
    while not rospy . is_shutdown ():
        rospy . sleep (0.001)
if __name__ == '__main__ ':
    try :
         mainloop ()
    except rospy . ROSInterruptException : pass
