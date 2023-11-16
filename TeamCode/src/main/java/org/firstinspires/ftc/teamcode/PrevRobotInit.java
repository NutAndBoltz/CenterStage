//package org.firstinspires.ftc.teamcode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;
//
//
////@Disabled
//public class PrevRobotInit {
//
//    /* Public OpMode members. */
//    //creating objects
//    public DcMotor motorFL;
//    public DcMotor motorFR;
//    public DcMotor motorBL;
//    public DcMotor motorBR;
////    public DcMotor armLiftLeft;
////    public DcMotor armLiftRight;
//    //public DcMotor waiter;
////    public DcMotor armLift; //arm lifting mechanism
//
//
////  Servos intilization (Claw left and right part)
////    Keep cone in place
//
////    public Servo closerL;
////    //public Servo closerR;
////    public CRServo waiter;
//
//
//    //from Encoder Sample
//    double     COUNTS_PER_MOTOR_REV    = 537.7;
//    double     WHEEL_DIAMETER_INCHES   = 4.0 ;  // For figuring circumference
//    double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI); //42.789 counts per inch
//    double     DRIVE_SPEED             = 0.65;
//    double     ARM_SPEED_RISING        = 0.42;
//    double     ARM_SPEED_LOWER         = 0.35;
//    double     ARM_SPEED               = 0.45;
//    double     teleOP_FORWARD_SPEED    = 1;
//
//    /* local OpMode members */
//    HardwareMap hardwareMap ;
//
//    // instantiate = to create an instance of = constructor
//    /* Constructor */
//    public PrevRobotInit(){
//
//    }
//
//
//    /* Initialize standard Hardware interfaces */
//    public void init(HardwareMap ahwMap) {
//        // Save reference to Hardware map
//        hardwareMap = ahwMap;
//
//
//        // Define and Initialize Motors
//        motorFL = hardwareMap.get(DcMotor.class, "motor_fl");
//        motorFR = hardwareMap.get(DcMotor.class, "motor_fr");
//        motorBL = hardwareMap.get(DcMotor.class, "motor_bl");
//        motorBR = hardwareMap.get(DcMotor.class, "motor_br");
////        armLiftLeft = hardwareMap.get(DcMotor.class, "armLiftLeft");
////        armLiftRight = hardwareMap.get(DcMotor.class, "armLiftRight");
//        //waiter = hardwareMap.get(DcMotor.class, "waiter");
//
////        spinner = hardwareMap.get(DcMotor.class, "spinner");
//
//        // Set the direction of the DC motors
//        motorFL.setDirection(DcMotor.Direction.REVERSE);
//        motorFR.setDirection(DcMotor.Direction.FORWARD);
//        motorBL.setDirection(DcMotor.Direction.REVERSE);
//        motorBR.setDirection(DcMotor.Direction.FORWARD);
////        armLiftLeft.setDirection(DcMotor.Direction.REVERSE); //Not sure which direction
////        armLiftRight.setDirection(DcMotor.Direction.FORWARD); //Not sure which direction
//        //waiter.setDirection(DcMotor.Direction.REVERSE); //Not sure which direction
//
//
//        // Set all DC motors to zero power
//        motorBL.setPower(0);
//        motorBR.setPower(0);
//        motorFR.setPower(0);
//        motorFL.setPower(0);
////        armLiftLeft.setPower(0);
////        armLiftRight.setPower(0);
//        //waiter.setPower(0);
//
//
//        // Set all motors to run without encoders.
//        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        //waiter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Not sure
////        armLiftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Not sure
////        armLiftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Not sure
//
//
//        // Define and initialize ALL installed servos.
////        closerL = hardwareMap.get(Servo.class, "closerL");
////      //closerR = hardwareMap.get(Servo.class, "closerR");
////        waiter = hardwareMap.get(CRServo.class, "waiter");
//
//
//    }
//}