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
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

/* Imports for Rev Expansion Hub IMU */

@Autonomous(name="Reset Actuator", group="Pushbot")
public class ResetActuator extends LinearOpMode {

    /* Declare RR_Harware which is class that details our Expansion Hub Hardware used. */
    RR_Hardware         robot   = new RR_Hardware();
    /* Create a Timer to use to make sure we stay under 30 seconds to Autonomous Mode */
    private ElapsedTime     runtime = new ElapsedTime();
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

    boolean FRESH = TRUE;


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



        waitForStart();
        while (opModeIsActive() && FRESH) {
            //Drop roboto 4"
            encoderDrive(DRIVE_SPEED, 0, 0, -INCHES_7_IN_TICKS, 100);

            FRESH = FALSE;
        }
        // Stop all motion;
        robot.vert_act.setPower(0);
    }

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

}

