package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class MoveTheRobot extends LinearOpMode {
    public PrevRobotInit robot = new PrevRobotInit();
    ElapsedTime runtime = new ElapsedTime();
    //    public class releaseLeftClaw implements Runnable {
//        public void run(){
////
//        }
//    }
//    public class clampLeftClaw implements Runnable {
//        public void run(){
//
//        }
//    }
//    public class releaseRightClaw implements Runnable {
//        public void run(){
////
//        }
//    }
//    public class startRightClaw implements Runnable {
//        public void run(){
//
//        }
//    }
    public void runOpMode() {
        robot.init(hardwareMap);
        boolean openToggle = false;

        waitForStart();

        while (opModeIsActive()) {
            //VARIABLES INITIALIZATION
            //WHEELS
            double vertical = 0.80*(-gamepad2.left_stick_y); //move forward, backward
            double horizontal = 0.80*(gamepad2.left_stick_x); //move left, right
            double turn = 0.80*(-gamepad2.right_stick_x); //turn left, right


            //SLIDER
//            boolean buttonY = gamepad1.y;
//            boolean buttonA = gamepad1.a;

//            double armDown = gamepad1.left_trigger; // brings linear slides down
//            double armUp = -gamepad1.right_trigger; // brings linear slides up
//
//            //SPINNER
//            double turntable = (-gamepad1.right_stick_x); // turning on the turntable
//            robot.waiter.setPower(turntable);
//
//            //CLAW
//            boolean clamp = gamepad1.right_bumper; // clamps the closer servo
//            boolean release = gamepad1.left_bumper; // release the closer servo
//
//            //turning is same, triggers raise/lower, bumpers open/close,
//
//
//            //SLIDER
//            robot.armLiftLeft.setPower(armDown + armUp);
//            robot.armLiftRight.setPower(armDown + armUp);
//            telemetry.addData("LEFT: ",robot.armLiftLeft.getCurrentPosition());
//            telemetry.addData("RIGHT: ",robot.armLiftRight.getCurrentPosition());

           // telemetry.update();

            //mecnum wheels driving
            robot.motorFL.setPower(vertical + horizontal - turn);
            robot.motorFR.setPower(vertical - horizontal + turn);
            robot.motorBL.setPower(vertical - horizontal - turn);
            robot.motorBR.setPower(vertical + horizontal + turn);


            //clamp and release cone with closer servo PROBLEM WITH SERVOS
//            if (clamp) {
//                robot.closerL.setPosition(0); //Rotates clockwise
//                // robot.closerR.setPosition(0.6); //Rotates clockwise
////                robot.closerR.setPosition(0); //Rotates counterclockwise
//                telemetry.addData("CURRENT ACTION:", "clamp pressed");
//                telemetry.update();
//            }
//            if (release) {
//                robot.closerL.setPosition(0.5); //release cone with closer servo
//                // robot.closerR.setPosition(0); //release cone with closer servo
////                robot.closerR.setPosition(.5); //release cone with closer servo
//                telemetry.addData("CURRENT ACTION", "Release pressed");
//                telemetry.update();
//            }
        }
    }

//MINIT AUTNOMOUS

    //            if (buttonY) {
//               raiseTop();
//            }
//            if (buttonA) {
//                lowerZero();
//            }

//    public void raise(int numberOfJunction) {
//
//        int[][] positionSet;
//        int HIGH_JUNCTION_LEFT = -1;
//        int HIGH_JUNCTION_RIGHT = -1;
//
//        int MEDIUM_JUNCTION_LEFT = -1;
//        int MEDIUM_JUNCTION_RIGHT = -1;
//
//        int LOW_JUNCTION_LEFT = -1;
//        int LOW_JUNCTION_RIGHT = -1;
//
//        int GROUND_JUNCTION_LEFT = 0;
//        int GROUND_JUNCTION_RIGHT = 0;
//
//
//        // allocates memory for 10 integers
//        positionSet = new int[][]{
//                {GROUND_JUNCTION_LEFT, LOW_JUNCTION_LEFT, MEDIUM_JUNCTION_LEFT, HIGH_JUNCTION_LEFT},
//                {GROUND_JUNCTION_RIGHT, LOW_JUNCTION_RIGHT,MEDIUM_JUNCTION_RIGHT, HIGH_JUNCTION_RIGHT},
//        };
//
//        int newArmLiftTargetRight;
//        int newArmLiftTargetLeft;
//
//
//        // Determine new target position, and pass to motor controller
//        newArmLiftTargetRight = robot.armLiftLeft.getCurrentPosition() + positionSet[numberOfJunction][numberOfJunction];
//        newArmLiftTargetLeft =   robot.armLiftRight.getCurrentPosition() + positionSet[numberOfJunction][numberOfJunction];
//
//        robot.armLiftLeft.setTargetPosition(newArmLiftTargetLeft);
//        robot.armLiftRight.setTargetPosition(newArmLiftTargetRight);
//
//        // Turn On RUN_TO_POSITION
//        robot.armLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.armLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.armLiftLeft.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.armLiftRight.setPower(Math.abs(robot.DRIVE_SPEED));
//
//      while (opModeIsActive() && robot.armLiftLeft.isBusy() && robot.armLiftRight.isBusy()) {
//            // Display it for the driver.
//            telemetry.addData("CURRENT ACTION: ",  "RAISING TO THE TOP");
//            telemetry.update();
//        }
//    }
//
//
//    public void lowerZero() {
//
//        robot.armLiftLeft.setTargetPosition(0);
//        robot.armLiftRight.setTargetPosition(0);
//
//        // Turn On RUN_TO_POSITION
//        robot.armLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.armLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.armLiftLeft.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.armLiftRight.setPower(Math.abs(robot.DRIVE_SPEED));
//    }

}