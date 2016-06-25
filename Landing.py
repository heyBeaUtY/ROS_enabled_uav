//Landing
#!/ usr / bin / env python
from __future__ import print_function
import roslib ; roslib . load_manifest ('roscopter ')
import rospy
from std_msgs . msg import String
from sensor_msgs . msg import NavSatFix , NavSatStatus , Imu
import roscopter . msg
import sys , struct ,time ,os
import time

def vfr_hud ( data ):

    w = data . heading
    altitude = data .alt
    
    error = 0 - altitude
    error_w = 0 - w
    action = kp * error
    action_w = kp * error_w
 
   Proportional = kp * error
     Proportional_w = kp * error_w
   action = Proportional
     action_w = 10 * Proportional_w
 
     rc_action = 1000 + action
     rc_action_w = 1500 + action_w
     channel = [0,0 , rc_action , rc_action_w ,0 ,0 ,0 ,0]
 
   security_vel ( channel )
 
kp = 0.5
vfr_hud_sub = rospy . Subscriber (" vfr_hud ", roscopter .msg. VFR_HUD , vfr_hud )

security = 0
pub_rc = rospy . Publisher ('send_rc ', roscopter .msg.RC)
rc_sub = rospy . Subscriber ("rc", roscopter . msg .RC , rc)

def mainloop ():
	
    pub = rospy . Publisher ('landing ', String )
    rospy . init_node ('landing ')

    while not rospy . is_shutdown ():
       rospy . sleep (0.001)
if __name__ == '__main__ ':
    try :
         mainloop ()
    except rospy . ROSInterruptException : pass
