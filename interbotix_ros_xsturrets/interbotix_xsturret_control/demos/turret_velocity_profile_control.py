#!/usr/bin/env python3

# Copyright 2024 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from interbotix_xs_modules.xs_robot.turret import InterbotixTurretXS
"""
This script commands arbitrary positions to the PhantomX XL430 Turret when using
Velocity-Based-Profile for its Drive Mode

To get started, open a terminal and type:

    ros2 launch interbotix_xsturret_control xsturret_control.launch robot_model:=pxxls

Then change to this directory and type:

    python3 turret_velocity_profile_control.py

Note that in general, each 'profile_velocity' unit is 0.229 rev/min and each 'profile_acceleration'
unit is 214.577 rev/min^2
"""


def main():
    # By default, the 'pan' and 'tilt' motors are initialized with the 'time' profile_type. So
    # change to 'velocity' for this demo
    #
    # Set the pan servo's max velocity about pi rad/s (equivalent to 131), and the max acceleration
    # about 5.6 rad/s^2 (equivalent to 15)
    #
    # Set the tilt servo's max velocity about pi/2 rad/s and the max acceleration about 1.87
    # rad/s^2
    robot = InterbotixTurretXS(
        robot_model="pxxls",
        pan_profile_type="velocity",
        tilt_profile_type="velocity",
        pan_profile_velocity=131,
        pan_profile_acceleration=131,
        tilt_profile_velocity=65,
        tilt_profile_acceleration=5
    )

    # Move the pan servo to 1.0 radians
    #
    # Then arbitrarily wait 1 second until the motion finishes to return control to the user
    robot.turret.pan(
        position=1.0,
        profile_velocity=131,
        profile_acceleration=15,
        delay=1.0
    )

    # Move the tilt servo to 1.5 radians
    #
    # Then arbitrarily wait 2 seconds to return control to the user
    robot.turret.tilt(
        position=1.5,
        profile_velocity=65,
        profile_acceleration=5,
        delay=2.0
    )

    # Move the turret to [0.8, -1] radians
    #
    # Make the pan peak-velocity about 0.75 pi rad/s and the tilt peak-velocity about 0.25 pi rad/s
    #
    # Make the pan peak-acceleration about 1.87 rad/s^2 and tilt peak-acceleration 'infinite'
    # (accelerate as fast as physically possible)
    #
    # Then arbitrarily wait 3 seconds to return control to the user
    robot.turret.pan_tilt_move(
        pan_position=-0.8,
        tilt_position=-1,
        pan_profile_velocity=98,
        pan_profile_acceleration=5,
        tilt_profile_velocity=33,
        tilt_profile_acceleration=0,
        delay=3.0
    )

    # Move the turret back home (0 rad for both pan and tilt)
    #
    # Use the last 'profile_velocity' and 'profile_acceleration' settings by default (ones from the
    # line above)
    robot.turret.pan_tilt_go_home()

    # End all background processes
    robot.shutdown()


if __name__ == '__main__':
    main()
