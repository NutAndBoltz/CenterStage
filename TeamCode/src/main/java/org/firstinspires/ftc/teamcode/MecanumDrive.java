/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="Mecanum Drive", group="Robot")
public class MecanumDrive extends LinearOpMode {

    // Define drive speed
    public static final double DRIVE_SPEED = 0.8;


    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    @Override
    public void runOpMode() {
        double vertical = 0;
        double horizontal = 0;
        double turn = 0;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Mecanum wheels - calculate power from gamepad
            vertical = DRIVE_SPEED * (-gamepad1.left_stick_y); //move forward, backward
            horizontal = DRIVE_SPEED * (gamepad1.left_stick_x); //strafe left, right
            turn = DRIVE_SPEED * (-gamepad1.right_stick_x); //rotate left, right

            boolean release = gamepad1.b; // releases drone
            boolean clamp = gamepad1.right_bumper; //clamps pixel
            boolean unclamp = gamepad1.left_bumper; //unclamps pixel
            boolean up = gamepad1.dpad_up; //wrist moves up
            boolean down = gamepad1.dpad_down; //wrist moves down
            double lift = (.5 * gamepad1.right_trigger); // lifts arm
            double drop = (.5 * -gamepad1.left_trigger); // drops arm

            //Mecanum wheels - run motors
            robot.leftFront.setPower(vertical + horizontal - turn);
            robot.rightFront.setPower(vertical - horizontal + turn);
            robot.leftRear.setPower(vertical - horizontal - turn);
            robot.rightRear.setPower(vertical + horizontal + turn);
            robot.beanStalk.setPower(lift + drop);

            if (release) {
                robot.droneServo.setPosition(0.5); //releases drone
                telemetry.addData("CURRENT ACTION:", "clamp pressed");
                telemetry.update();
            }
            if (clamp) {
                robot.claw.setPosition(robot.claw.getPosition()+.01); //clamp
                telemetry.addData("CURRENT ACTION:", "clamp");
                telemetry.update();
            }
            if (unclamp) {
                robot.claw.setPosition(robot.claw.getPosition()-.01); //unclamp
                telemetry.addData("CURRENT ACTION:", "unclamp");
                telemetry.update();
            }
            if (up) {
                robot.wrist.setPosition(robot.wrist.getPosition()+.01); //wrist up
                telemetry.addData("CURRENT ACTION:", "wrist up");
                telemetry.update();
            }
            if (down) {
                robot.wrist.setPosition(robot.wrist.getPosition()-.01); //wrist down
                telemetry.addData("CURRENT ACTION:", "wrist down");
                telemetry.update();
            }
        }
    }
            RobotHardware robot = new RobotHardware(this);
}

