from interbotix_xs_modules.turret import InterbotixTurretXS

# This script commands arbitrary positions to the PhantomX XL430 Turret when using Velocity-Based-Profile for its Drive Mode
#
# To get started, open a terminal and type 'roslaunch interbotix_xsturret_control xsturret_control.launch robot_model:=pxxls'
# Then change to this directory and type 'python turret_velocity_profile_control.py'
# Note that in general, each 'profile_velocity' unit is 0.229 rev/min and each 'profile_acceleraton' unit is 214.577 rev/min^2

def main():
    # By default, the 'pan' and 'tilt' motors are intialized with the 'time' profile_type. So change to 'velocity' for this demo
    # set the pan servo's max velocity about pi rad/s (equivelant to 131), and the max acceleration about 5.6 rad/s^2 (equivelant to 15)
    # set the tilt servo's max velocity about pi/2 rad/s and the max acceleration about 1.87 rad/s^2
    robot = InterbotixTurretXS(
        robot_model="pxxls",
        pan_profile_type="velocity",
        tilt_profile_type="velocity",
        pan_profile_velocity=131,
        pan_profile_acceleration=131,
        tilt_profile_velocity=65,
        tilt_profile_acceleration=5)

    # move the pan servo to 1.0 radians
    # then arbitrarily wait 1 second until the motion finishes to return control to the user
    robot.turret.pan(
        position=1.0,
        profile_velocity=131,
        profile_acceleration=15,
        delay=1.0)

    # move the tilt servo to 1.5 radians 
    # then arbitrarily wait 2 seconds to return control to the user
    robot.turret.tilt(
        position=1.5,
        profile_velocity=65,
        profile_acceleration=5,
        delay=2.0)

    # move the turret to [0.8, -1] radians
    # make the pan peak-velocity about 0.75 pi rad/s and the tilt peak-velocity about 0.25 pi rad/s
    # make the pan peak-acceleration about 1.87 rad/s^2 and tilt peak-acceleration 'infinite' (accelerate as fast as physically possible)
    # then arbitrarily wait 3 seconds to return control to the user
    robot.turret.pan_tilt_move(
        pan_position=-0.8,
        tilt_position=-1,
        pan_profile_velocity=98,
        pan_profile_acceleration=5,
        tilt_profile_velocity=33,
        tilt_profile_acceleration=0,
        delay=3.0)

    # move the turret back home (0 rad for both pan and tilt)
    # use the last 'profile_velocity' and 'profile_acceleration' settings by default (ones from the line above)
    robot.turret.pan_tilt_go_home()

if __name__=='__main__':
    main()
