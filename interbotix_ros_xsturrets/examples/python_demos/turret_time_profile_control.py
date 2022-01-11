from interbotix_xs_modules.turret import InterbotixTurretXS

# This script commands arbitrary positions to the PhantomX XL430 Turret when using Time-Based-Profile for its Drive Mode
# When operating a robot in 'position' control mode, Time-Based-Profile allows you to easily set the duration of a particular movement
#
# To get started, open a terminal and type 'roslaunch interbotix_xsturret_control xsturret_control.launch robot_model:=pxxls'
# Then change to this directory and type 'python turret_time_profile_control.py'

def main():
    robot = InterbotixTurretXS(
        robot_model="pxxls",
        pan_profile_type="time",
        tilt_profile_type="time")

    # move the pan servo to 1.0 radians; make the movement take 1 second and spend half a second accelerating and decelerating;
    # then wait until the motion finishes to return control to the user
    robot.turret.pan(
        position=1.0,
        profile_velocity=1.0,
        profile_acceleration=0.5,
        blocking=True)

    # move the tilt servo to 1.5 radians; make the movement take 1.5 seconds, and spend 300ms accelerating and decelerating;
    # then wait until the motion finishes to return control to the user
    robot.turret.tilt(
        position=1.5,
        profile_velocity=1.5,
        profile_acceleration=0.3,
        blocking=True)

    # move the turret to [0.8, -1] radians
    # make the pan motion take 1 second - spending 0.4 seconds accelerating to and decelerating from peak velocity
    # make the tilt motion take 2 seconds - spending 1 second accelerating to and decelerating from peak velocity
    robot.turret.pan_tilt_move(
        pan_position=-0.8,
        tilt_position=-1,
        pan_profile_velocity=1.0,
        pan_profile_acceleration=0.4,
        tilt_profile_velocity=2.0,
        tilt_profile_acceleration=1.0)

    # move the turret back home (0 rad for both pan and tilt)
    # use the last 'profile_velocity' and 'profile_acceleration' settings by default (ones from the line above)
    robot.turret.pan_tilt_go_home()

if __name__=='__main__':
    main()
