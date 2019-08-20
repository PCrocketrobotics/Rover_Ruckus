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

/* Imports for General Op Modes */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

/* Imports for Rev Expansion Hub IMU */
/* Imports for Vuforia and Tensorflow for Camera Functions */


/**
 * General Autonomous Mode Program
 * Rocket Robotics - Rover Ruckus 2018 - 2019 Season
 * Programming Team - Kyle Clawson, Daniel Wood, Chad Willard
 *
 */

@Autonomous(name="Auto Red Depot", group="Pushbot")
public class Auto_Red_Depot extends LinearOpMode {

    /* Declare RR_Harware which is class that details our Expansion Hub Hardware used. */
    RR_Hardware         robot   = new RR_Hardware();
    /* Create a Timer to use to make sure we stay under 30 seconds to Autonomous Mode */
    private ElapsedTime     runtime = new ElapsedTime();
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.05;     // Larger is more responsive, but also less stable
    // Drive Motors are a 40:1 Gear Ration NeverRest Motor
    // Encoder has 1120 Counts per revolution.
    // Encoder resolution is 1/ 1120 * 360 degrees = 0.32142 degrees.
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // Neverest Encoder 40:1
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // 32 Tooth at Motor, 32 Tooth at Wheels 32/32 = 1
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // 4" Omni Wheels
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1;
    static final double     ACCURACY_SPEED          = 0.5;
    static final double     TURN_SPEED              = 1;
    static final double     ARM_SPEED               = 1;
    // Arm Motor is a 40:1 Gear Ratio NeverReset Motor
    static final double ARM_COUNTS_PER_MOTOR_REV   =  1120;
    // AS TESTED; ONE REVOLUTION OF THE MOTOR = 0.42in
    static final double INCHES_PER_REVOLUTION  = .42;
    static final double ARM_REVOLUTIONS_PER_INCH = .618/INCHES_PER_REVOLUTION;
    static  final double ARM_COUNTS_PER_INCH = (ARM_COUNTS_PER_MOTOR_REV* ARM_REVOLUTIONS_PER_INCH);
    //static final int INCHES_7_IN_TICKS = 16000;
    static final int INCHES_7_IN_TICKS = 17700;
    public String trackableInView = "Nothing in View";

    boolean FRESH = TRUE;
    public  static  Boolean FoundGold = FALSE;
    public  static String GoldLocation = "NULL";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AeAU0DH/////AAABmQY+Lfrn2EV4pnO0QZ5jAzUhybrHQZp5ifZxMY39xl7oKMB8FJPmGRodb6CKTsc/UdCBivixczM/h5Zj+W6MgyKOEHFxuVsWQe4RxjnZXh7pP7uYmo4TswS8XPOh7DjcEpJEPBvLM+F6+tmfjcO3ktUEbucTTX+sTT9LNzneIRN0EApshnbs7mwjljT7CqjjPG1V9YgIjAGaTc+WM7cgFhB5QyMlEHlfGqYrfoQw1TUwFx1CqhJHknERgh+bDCtxuk88VJJzPfg+uKn5zyzZvt3P+Kr+yUkzcbRXn5wKkfccTMl2zkGfXzszfJAZ/e68kJ77duH9m74kzKLunf8xQqHRbrnUaddEUXTzYXo7iQeK";

    // The IMU sensor object
    //BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // Everything below (before wait for start, happens when you press INIT on Driver Station

        // Initialize RR_Hardware map for our used hardware.
        robot.init(hardwareMap);
        //ImageCamera.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        // Reset current Encodeer Positions to 0
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.vert_act.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.vert_act.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.leftDrive.getCurrentPosition(),
                          robot.rightDrive.getCurrentPosition());
                          robot.vert_act.getCurrentPosition();
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Init Vuforia");
        telemetry.update();
        initVuforia();   // Init the camera system on the phone while we wait to start the game.
        telemetry.addLine("Init Vuforia ... Done");
        telemetry.update();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
            tfod.activate();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.clearAll();

        waitForStart();
        while (opModeIsActive() && FRESH) {
// Drive by Gyro
            //gyroDrive(DRIVE_SPEED, -160, 0);
            //Drop roboto 4"
            encoderDrive(DRIVE_SPEED, 0, 0, INCHES_7_IN_TICKS, 100);
            // Turn Robot to break away from crater. Drop Linear Act back to home position.
            //encoderDrive(DRIVE_SPEED, -7.09040008623, 7.09040008623, 0, 100); 4.05691657876    8.11383315751
            encoderDrive(ACCURACY_SPEED, -7.11383315751, 7.11383315751, 0, 100);

            // Move Forward 2 Inches
            encoderDrive(DRIVE_SPEED, -2, -2, 0, 100);
            // Move Robot back to Orig Orientation to start finding Gold
            // encoderDrive(ACCURACY_SPEED, 7.09040008623 ,-7.09040008623 , 0, 100);

            // See if Gold is in the middle
            if (tfod != null) {
                //tfod.activate();
                sleep(1000);
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel() == "Gold Mineral") {
                        GoldLocation = "RIGHT";
                        FoundGold = TRUE;
                        break;
                        // We found it in the middle.
                    }
                    else {
                       // tfod.deactivate();
                    }
                }
            }
            if (FoundGold == FALSE){
                encoderDrive(ACCURACY_SPEED, 5.0 ,-5.0 , 0, 100);
                if (tfod != null) {
                   // tfod.activate();
                    sleep(1000);
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel() == "Gold Mineral") {
                            GoldLocation = "MIDDLE";
                            FoundGold = TRUE;
                            break;
                            // We found it in the middle.`112
                        }
                        else {
                           // tfod.deactivate();
                        }
                    }
                }
            }
            if (FoundGold == FALSE){
                //encoderDrive(DRIVE_SPEED, -3.09040008623 ,3.09040008623 , 0, 100);
                //encoderDrive(DRIVE_SPEED, -3.09040008623 ,3.09040008623 , 0, 100);
                encoderDrive(ACCURACY_SPEED, 5.0 ,-5.0 , 0, 100);
                sleep(1000);
                if (tfod != null) {
                    //tfod.activate();
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel() == "Gold Mineral") {
                            GoldLocation = "LEFT";
                            FoundGold = TRUE;
                            break;
                            // We found it in the middle.
                        }
                        else {
                          //  tfod.deactivate();
                        }
                    }
                }
            }
            // Move Forward and Knock it off.
            if (GoldLocation == "MIDDLE"){
                encoderDrive(DRIVE_SPEED, -50, -50, 0, 100);
                encoderDrive(DRIVE_SPEED, -12, 12, 0, 100);

                //Drop Marker
                robot.marker.scaleRange(-1,1);
                robot.marker.setDirection(Servo.Direction.REVERSE);
                robot.marker.setPosition(1);
                sleep(1500);     // pause for servos to move
                robot.marker.setDirection(Servo.Direction.FORWARD);
                robot.marker.setPosition(1);
                sleep(1500);     // pause for servos to move

                //encoderDrive(DRIVE_SPEED, -84, -84, 0, 100);
                gyroDrive(ACCURACY_SPEED, -77, 0);
                encoderDrive(ACCURACY_SPEED, 6, -6, 0, 100);
                gyroDrive(ACCURACY_SPEED, -12, 0);
            }
            else if (GoldLocation == "LEFT"){
                encoderDrive(DRIVE_SPEED, -50, -50, 0, 100);
            }
            else if (GoldLocation == "RIGHT"){
                encoderDrive(DRIVE_SPEED, -44, -44, 0, 100);
                encoderDrive(DRIVE_SPEED, -13, 13, 0, 100);
                gyroDrive(DRIVE_SPEED, 40, 0);

                //Drop Marker
                robot.marker.scaleRange(-1,1);
                robot.marker.setDirection(Servo.Direction.REVERSE);
                robot.marker.setPosition(1);
                sleep(1500);     // pause for servos to move
                robot.marker.setDirection(Servo.Direction.FORWARD);
                robot.marker.setPosition(1);
                sleep(1500);     // pause for servos to move

                //encoderDrive(DRIVE_SPEED, -84, -84, 0, 100);
                gyroDrive(ACCURACY_SPEED, -77, 0);
                encoderDrive(ACCURACY_SPEED, 6, -6, 0, 100);
                gyroDrive(ACCURACY_SPEED, -12, 0);

            }
            else if (FoundGold == FALSE){
                // Could not find it. Make a guess where block is.
                encoderDrive(DRIVE_SPEED, -2, -2, 0, 100);
            }

            /*
            //Drop marker
            robot.marker.scaleRange(-1,1);
            robot.marker.setDirection(Servo.Direction.REVERSE);
            robot.marker.setPosition(1);
            sleep(1500);     // pause for servos to move
            robot.marker.setDirection(Servo.Direction.FORWARD);
            robot.marker.setPosition(1);
            sleep(1500);     // pause for servos to move
            */

            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.vert_act.setPower(0);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            FRESH = FALSE;
        }
        // Stop all motion;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.vert_act.setPower(0);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches, double armTicks,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newArmTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            //newArmTarget = robot.arm.getCurrentPosition() + (int)(armInches *ARM_COUNTS_PER_INCH);
            newArmTarget = robot.vert_act.getCurrentPosition() + (int) (armTicks);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.vert_act.setTargetPosition((newArmTarget));

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.vert_act.setMode((DcMotor.RunMode.RUN_TO_POSITION));

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));
            robot.vert_act.setPower(Math.abs(ARM_SPEED));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftDrive.isBusy() || robot.vert_act.isBusy() || robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.leftDrive.getCurrentPosition(),
                                            robot.rightDrive.getCurrentPosition());
                telemetry.addData("ArmPath", "Running at %7d", newArmTarget);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.vert_act.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.vert_act.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
        // Stop all motion;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.vert_act.setPower(0);
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = angles.firstAngle;
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), -1, 1);
            robot.leftDrive.setPower(speed);
            robot.rightDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed + steer;
                rightSpeed = speed - steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftDrive.setPower(leftSpeed);
                robot.rightDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.addData("Heading", angles.firstAngle);
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}

