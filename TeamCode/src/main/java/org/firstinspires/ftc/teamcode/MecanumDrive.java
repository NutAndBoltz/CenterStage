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
    public static final double DRIVE_SPEED   =  0.8 ;


    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double vertical      = 0;
        double horizontal    = 0;
        double turn          = 0;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Mecanum wheels - calculate power from gamepad
            vertical = DRIVE_SPEED*(-gamepad1.left_stick_y); //move forward, backward
            horizontal = DRIVE_SPEED*(gamepad1.left_stick_x); //strafe left, right
            turn = DRIVE_SPEED*(-gamepad1.right_stick_x); //rotate left, right

            boolean release = gamepad1.right_bumper; // releases drone
            boolean lift = gamepad1.b; // lifts arm
            boolean drop = gamepad1.left_bumper; // drops arm
            boolean openClaw = gamepad1.dpad_left; // opens claw
            boolean closeClaw = gamepad1.dpad_right; // closes claw
            boolean wristUp = gamepad1.dpad_right; // closes claw
            boolean wristDown = gamepad1.dpad_right; // closes claw


            //Mecanum wheels - run motors
            robot.leftFront.setPower(vertical + horizontal - turn);
            robot.rightFront.setPower(vertical - horizontal + turn);
            robot.leftRear.setPower(vertical - horizontal - turn);
            robot.rightRear.setPower(vertical + horizontal + turn);

            if (release) {
                robot.droneServo.setPosition(1); //releases drone
                telemetry.addData("CURRENT ACTION:", "drone launched");
                telemetry.update();
            }
            if (lift) {
                robot.elbow.setPosition(0.3); //raise arm
                telemetry.addData("CURRENT ACTION:", "lifting arm");
                telemetry.update();
            }
            if (drop) {
                robot.elbow.setPosition(0.8); //lowers arm
                telemetry.addData("CURRENT ACTION:", "lowering arm");
                telemetry.update();
            }
            if (openClaw) {
                robot.claw.setPosition(0.8); //lowers arm
                telemetry.addData("CURRENT ACTION:", "lowering arm");
                telemetry.update();
            }
            if (closeClaw) {
                robot.claw.setPosition(0.2); //lowers arm
                telemetry.addData("CURRENT ACTION:", "lowering arm");
                telemetry.update();
            }
            if (wristUp) {
                robot.wrist.setPosition(0.8); //lowers arm
                telemetry.addData("CURRENT ACTION:", "wrist arm");
                telemetry.update();
            }
            if (wristDown) {
                robot.wrist.setPosition(0.2); //lowers arm
                telemetry.addData("CURRENT ACTION:", "wrist arm");
                telemetry.update();
            }
        }
    }
}