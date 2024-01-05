/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name="Red Right Auto", group="Robot")
public class RedRightAuto extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    // RobotHardware robot       = new RobotHardware(this); //commented out because using SampleMecanumDrive for drive motors
    private ElapsedTime     runtime = new ElapsedTime();
    static OpenCvWebcam webcam;
    CenterStagePipeline pipeline; //pipeline = series of img coming through camera to process
    CenterStagePipeline.PropPosition snapshotAnalysis = CenterStagePipeline.PropPosition.CENTER; // default

    @Override
    public void runOpMode() {

        // initialize all the hardware except drive motors, using the hardware class.
        //robot.init();

        // initialize the drive motors for odometry
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        pipeline = new CenterStagePipeline();
        webcam.setPipeline(pipeline);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        // ===========
        // Trajectories
        // ===========

        //left spike mark and park trajectory sequence
        TrajectorySequence leftTrajSeq = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(20)
                .lineToLinearHeading(new Pose2d(27, 8, Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)))
                .strafeRight(33)
                .build();

        //center spike mark and park trajectory sequence
        TrajectorySequence centerTrajSeq = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(32)
                .back(12)
                .strafeRight(33)
                .build();

        //right spike mark and park trajectory sequence
        TrajectorySequence rightTrajSeq = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(20)
                .lineToLinearHeading(new Pose2d(27, -8, Math.toRadians(-45)))
                .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)))
                .strafeRight(33)
                .build();


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        snapshotAnalysis = pipeline.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        switch (snapshotAnalysis)
        {
            case LEFT:
            {
                /* Your autonomous code */
                // Drive to left spike mark and then park
                drive.followTrajectorySequence(leftTrajSeq);

                break;
            }

            case CENTER:
            {
                /* Your autonomous code */
                // Drive to center spike mark and then park
                drive.followTrajectorySequence(centerTrajSeq);

                break;
            }

            case RIGHT:
            {
                /* Your autonomous code*/
                // Drive to right spike mark and then park
                drive.followTrajectorySequence(rightTrajSeq);

                break;
            }
        }

        telemetry.addLine("Waiting for start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            // Telemetry for testing barcode detection
            telemetry.addData("LeftAnalysis", pipeline.getLeftAnalysis());
            telemetry.addData("CenterAnalysis", pipeline.getCenterAnalysis());
            telemetry.addData("RightAnalysis", pipeline.getRightAnalysis());
            telemetry.addData("Position", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(1000);
        }
    }

    public static class CenterStagePipeline extends OpenCvPipeline {
        boolean viewportPaused;

        public enum PropPosition {
            LEFT,
            CENTER,
            RIGHT
        }

        // Some color constants
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        // The core values which define the location and size of the sample regions
        static final Point BOX1_TOPLEFT_ANCHOR_POINT = new Point(23, 120);
        static final Point BOX2_TOPLEFT_ANCHOR_POINT = new Point(143, 95);
        static final Point BOX3_TOPLEFT_ANCHOR_POINT = new Point(253, 125);

        static final int REGION_WIDTH = 55;
        static final int REGION_HEIGHT = 55;
        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (75,75) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                BOX1_TOPLEFT_ANCHOR_POINT.x,
                BOX1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                BOX1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                BOX1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Point region2_pointA = new Point(
                BOX2_TOPLEFT_ANCHOR_POINT.x,
                BOX2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                BOX2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                BOX2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Point region3_pointA = new Point(
                BOX3_TOPLEFT_ANCHOR_POINT.x,
                BOX3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                BOX3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                BOX3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        // Creating variables
        Mat YCrCb = new Mat();
        Mat Cr = new Mat();

        //Box1
        Mat region1_Cr;
        int avg1;

        //Box2
        Mat region2_Cr;
        int avg2;

        //Box3
        Mat region3_Cr;
        int avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile PropPosition position = PropPosition.LEFT;

        // This function takes the RGB frame, converts to YCrCb, and extracts the Cr channel to the 'Cr' variable
        void inputToCr(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cr, 1); // 1 selects the 2nd color channel which is Cr (Chroma red) [the index counting starts at 0, 1, 2...]
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCr(firstFrame);

            //Figure out how much blue is in each region
            region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cr = Cr.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cr = Cr.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because this op mode is designed to detect an
             * all-blue team prop.
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over strike mark. The brightest blue of the 3 regions
             * is where we assume the team prop to be.
             *
             * In order for this whole process to work correctly, each region
             * should be positioned where the team prop will be located, and
             * be small enough such that only the team prop is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCr(input);

            avg1 = (int) Core.mean(region1_Cr).val[0];
            avg2 = (int) Core.mean(region2_Cr).val[0];
            avg3 = (int) Core.mean(region3_Cr).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(max == avg1) // Was it from region 1?
            {
                position = PropPosition.LEFT; // Record our analysis

                /*
                 * Draw a  green rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        4); // Negative thickness means solid fill
            }
            else if(max == avg2) // Was it from region 2?
            {
                position = PropPosition.CENTER; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        4); // Negative thickness means solid fill
            }
            else if(max == avg3) // Was it from region 3?
            {
                position = PropPosition.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        4); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        public PropPosition getAnalysis() {
            return position;
        }
        public int getLeftAnalysis() {
            return avg1;
        }

        public int getCenterAnalysis() {
            return avg2;
        }

        public int getRightAnalysis() {
            return avg3;
        }

        @Override
        public void onViewportTapped() {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}
