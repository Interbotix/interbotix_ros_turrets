from interbotix_xs_modules.turret import InterbotixTurretXS

# This script commands arbitrary positions to the PhantomX XL430 Turret when using both Time & Velocity-Based-Profiles for its Drive Mode
#
# To get started, open a terminal and type 'roslaunch interbotix_xsturret_control xsturret_control.launch robot_model:=pxxls'
# Then change to this directory and type 'python turret_mixed_profile_control.py'
# Note that in general, each 'profile_velocity' unit is 0.229 rev/min and each 'profile_acceleraton' unit is 214.577 rev/min^2 when using the Velocity-based-Profile

def main():
    # By default, the 'pan' and 'tilt' motors are intialized with the 'time' profile_type. So change the pan servo to 'velocity' for this demo
    # The movement of the pan servo should take 1 second and spend half a second accelerating and decelerating
    # The max velocity of the tilt servo should be about pi/2 rad/s and the max acceleration about 1.87 rad/s^2
    robot = InterbotixTurretXS(
        robot_model="pxxls",
        pan_profile_type="time",
        tilt_profile_type="velocity",
        pan_profile_velocity=1.0,
        pan_profile_acceleration=0.5,
        tilt_profile_velocity=65,
        tilt_profile_acceleration=5)

    # move the pan servo to 1.0 radians
    # then wait until the motion finishes to return control to the user
    robot.turret.pan(
        position=1.0,
        blocking=True)

    # move the tilt servo to 1.5 radians
    # then arbitrarily wait 2 seconds to return control to the user
    robot.turret.tilt(
        position=1.5,
        delay=2.0)

    # move the turret to [0.8, -1] radians
    # make the pan peak-velocity about 0.75 pi rad/s and the tilt peak-velocity about 0.25 pi rad/s
    # make the pan peak-acceleration about 1.87 rad/s^2 and tilt peak-acceleration 'infinite' (accelerate as fast as physically possible)
    # then arbitrarily wait 3 seconds to return control to the user
    robot.turret.pan_tilt_move(
        pan_position=-0.8,
        tilt_position=-1,
        pan_profile_velocity=1.0,
        pan_profile_acceleration=0.4,
        tilt_profile_velocity=33,
        tilt_profile_acceleration=0,
        delay=3.0)

    # move the turret back home (0 rad for both pan and tilt)
    # use the last 'profile_velocity' and 'profile_acceleration' settings by default (ones from the line above)
    robot.turret.pan_tilt_go_home()

if __name__=='__main__':
    main()
