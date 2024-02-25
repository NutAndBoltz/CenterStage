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
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mecanum Drive Two Drivers", group="Robot")
public class MecanumDriveTwoDrivers extends LinearOpMode {

    // Define drive speed
    public static final double DRIVE_SPEED = 0.8;

    double clawOffset = 0;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    double wristOffset = 0;
    public static final double WRIST_SPEED  = 0.001 ;                 // sets rate to move servo
    public static final double WINCH_SPEED  = 0.5 ;                 // sets rate to move winch DC motor
    boolean pixelPlacerUp = false; // keep track of pixel placer position for toggle control
    boolean previouslyPressed = false; // keep track of pixel placer button press for toggle control

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
            boolean togglePixelPlacer = gamepad1.a; // moves pixelPlacer servo up and down
            boolean hookPlacerUp = gamepad2.y; // moves hookPlacer servo up and down
            boolean hookPlacerDown = gamepad2.x; // moves hookPlacer servo up and down
            boolean clamp = gamepad2.right_bumper; //clamps pixel
            boolean unclamp = gamepad2.left_bumper; //unclamps pixel
            boolean up = gamepad2.dpad_up; //wrist moves up
            boolean down = gamepad2.dpad_down; //wrist moves down
            boolean left = gamepad2.dpad_left; //retract winch
            boolean right = gamepad2.dpad_right; //extend winch
            double lift = (.5 * gamepad2.right_trigger); // lifts arm
            double drop = (.5 * -gamepad2.left_trigger); // drops arm

            //Mecanum wheels - run motors
            robot.leftFront.setPower(vertical + horizontal - turn);
            robot.rightFront.setPower(vertical - horizontal + turn);
            robot.leftRear.setPower(vertical - horizontal - turn);
            robot.rightRear.setPower(vertical + horizontal + turn);
            robot.beanStalk.setPower(lift + drop);

            //launch drone
            if (release) {
                robot.droneServo.setPosition(1); //releases drone
                telemetry.addData("CURRENT ACTION:", "launch!");
                telemetry.update();
            }

            //move claw open and closed
            if (clamp) {
                robot.claw.setPosition(0);
                telemetry.addData("CURRENT ACTION:", "clamp");
                telemetry.update();
            }
            else if (unclamp) {
                robot.claw.setPosition(0.5);
                telemetry.addData("CURRENT ACTION:", "unclamp");
                telemetry.update();
            }

            //calculate wrist new target position
            if (up) {
                wristOffset += WRIST_SPEED;
                telemetry.addData("Wrist up:", wristOffset);
                telemetry.update();
            }
            else if (down) {
                //robot.wrist.setPosition(0);
                wristOffset -= WRIST_SPEED;
                telemetry.addData("Wrist down:", wristOffset);
                telemetry.update();
            }

            //move wrist to new target position
            wristOffset = Range.clip(wristOffset, 0, 1);
            robot.wrist.setPosition(wristOffset);

            //winch control
            if(right){
                robot.winch.setPower(WINCH_SPEED); //retract winch
            }
            else if(left){
                robot.winch.setPower(-WINCH_SPEED); //extend winch
            }
            else {
                robot.winch.setPower(0); //don't move
            }

            //hook placer two-button control
            if(hookPlacerUp){
                robot.hookPlacer.setPosition(0.1); //raise hookPlacer servo when button is pressed
            } else if (hookPlacerDown){
                robot.hookPlacer.setPosition(0.4); //lower hookPlacer servo when button is pressed and it's up
            }

            //pixel placer toggle control
            if (togglePixelPlacer){
                if (!previouslyPressed) {
                    if (pixelPlacerUp) {
                        robot.pixelPlacer.setPosition(0.03); // Lower
                    }
                    else {
                        robot.pixelPlacer.setPosition(.2); // Raise
                    }
                    pixelPlacerUp = !pixelPlacerUp;
                }
                previouslyPressed = true;
            } else { previouslyPressed = false; }

        }
    }
            RobotHardware robot = new RobotHardware(this);
}

