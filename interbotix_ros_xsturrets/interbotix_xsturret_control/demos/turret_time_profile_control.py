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
This script commands arbitrary positions to the PhantomX XL430 Turret when using Time-Based-Profile
for its Drive Mode

When operating a robot in 'position' control mode, Time-Based-Profile allows you to easily set the
duration of a particular movement

To get started, open a terminal and type:

    ros2 launch interbotix_xsturret_control xsturret_control.launch.py robot_model:=pxxls

Then change to this directory and type:

    python3 turret_time_profile_control.py
"""


def main():
    robot = InterbotixTurretXS(
        robot_model="pxxls",
        pan_profile_type="time",
        tilt_profile_type="time"
    )

    # Move the pan servo to 1.0 radians; make the movement take 1 second and spend half a second
    # accelerating and decelerating;
    #
    # Then wait until the motion finishes to return control to the user
    robot.turret.pan(
        position=1.0,
        profile_velocity=1.0,
        profile_acceleration=0.5,
        blocking=True
    )

    # Move the tilt servo to 1.5 radians; make the movement take 1.5 seconds, and spend 300ms
    # accelerating and decelerating;
    #
    # Then wait until the motion finishes to return control to the user
    robot.turret.tilt(
        position=1.5,
        profile_velocity=1.5,
        profile_acceleration=0.3,
        blocking=True
    )

    # Move the turret to [0.8, -1] radians
    #
    # Make the pan motion take 1 second - spending 0.4 seconds accelerating to and decelerating
    # from peak velocity
    #
    # Make the tilt motion take 2 seconds - spending 1 second accelerating to and decelerating from
    # peak velocity
    robot.turret.pan_tilt_move(
        pan_position=-0.8,
        tilt_position=-1,
        pan_profile_velocity=1.0,
        pan_profile_acceleration=0.4,
        tilt_profile_velocity=2.0,
        tilt_profile_acceleration=1.0
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
