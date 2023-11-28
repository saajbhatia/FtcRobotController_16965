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

package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Driver Controlled Code", group="TeleOp")
public class DriverControl extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor arm = null;
    //private DcMotor arm = null;
    public IMU gyro;
    public double heading;
    private boolean isAutonomous;
    private static final boolean USE_WEBCAM = false;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean isDrivingToBoard = false;
    private double x;
    private double y;
    private AnalogInput ultrasonicSensor;
    private Servo claw;
    private Servo pivot;
    private Servo airPlane;

    private double driveToX;
    private double driveToY;
    private final double boardX = 50;
    private final double boardY = 100;
    private double clawPosition;
    private double wantedArmPosition = 0.0;
    private double slowMultiplier = 1.0;
    private int armPosition = 0;
    private int cooldownTicksAButton = 0;
    private int cooldownTicksYButton = 0;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        initAprilTag();

        isAutonomous = false;

        telemetry.addData("Status", "Initialized");



        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backLeft  = hardwareMap.get(DcMotor.class, "BACKLEFT");
        frontRight = hardwareMap.get(DcMotor.class, "FRONTRIGHT");
        frontLeft  = hardwareMap.get(DcMotor.class, "FRONTLEFT");
        backRight = hardwareMap.get(DcMotor.class, "BACKRIGHT");
        arm = hardwareMap.get(DcMotor.class, "ARM");
        claw = hardwareMap.get(Servo.class, "CLAW");
        pivot = hardwareMap.get(Servo.class, "PIVOT");
        airPlane = hardwareMap.get(Servo.class, "AIRPLANE");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition((int)wantedArmPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        gyro = hardwareMap.get(IMU.class, "imu");
        gyro.resetYaw();
        heading = 0;

        ultrasonicSensor = hardwareMap.get(AnalogInput.class, "ULTRASONIC");
        //claw.resetDeviceConfigurationForOpMode();
        //claw.scaleRange(claw.getPosition(), claw.getPosition() + 0.6);
        clawPosition = claw.getPosition();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);
        pivot.setDirection(Servo.Direction.FORWARD);
        airPlane.setDirection(Servo.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        airPlane.setPosition(0.0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }

    public double getAngleImu() {
        //We choose to normalize the IMU's angle reading but you don't need to.
        return normalize(
                gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    public static double normalize(double degrees) {
        double normalizedAngle = degrees;
        while (normalizedAngle > 180) normalizedAngle -= 360;
        while (normalizedAngle <= -180) normalizedAngle += 360;
        return normalizedAngle;
    }

    public double calculateP(double power, double deadband) {
        double currentHeading = getAngleImu();
        double wantedHeading = heading;
        double headingCorrection = currentHeading - wantedHeading;
        while (headingCorrection > 180) headingCorrection -= 360;
        while (headingCorrection <= -180) headingCorrection += 360;

        if (Math.abs(headingCorrection) > deadband) {
            telemetry.addData("Deadband activated", "WARNING deadband was activated, heading correction exceeded limit of " + deadband + " degrees.");
            return 0.0;
        }
        return headingCorrection * power;
    }

    public void driveTo(double toX, double toY) {
        driveToX = toX;
        driveToY = toY;
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double backLeftPower;
        double backRightPower;
        double frontLeftPower;
        double frontRightPower;
        double forward = 0.0;
        double pivotPower = 0.0;

        //telemetryAprilTag();

        //List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        //for (AprilTagDetection detection : currentDetections) {
        //    if (detection.metadata != null) {
        //        telemetry.addData("detection id", "Id: " + detection.id);
        //        telemetry.addData("detection pitch", "Pitch: " + detection.ftcPose.pitch);
        //        telemetry.addData("detection pitch", "Yaw: " + detection.ftcPose.yaw);
        //        telemetry.addData("detection x", "X: " + detection.ftcPose.x);
        //        telemetry.addData("detection y", "Y: " + detection.ftcPose.y);
        //        telemetry.addData("detection z", "Z: " + detection.ftcPose.z);
        //        if (detection.id == 2) {
        //            heading += detection.ftcPose.yaw/7;
        //            forward -= (detection.ftcPose.y - 20)/100;
        //        }
        //    }
        //}

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double turn = 0.0;
        double strafe = 0.0;
        double realHeading = getAngleImu() + 180;
        double airPlanePower = 0.0;


        //if (gamepad1.dpad_down) {
        //    visionPortal.stopStreaming();
        //} else if (gamepad1.dpad_up) {
        //    visionPortal.resumeStreaming();
        //}
        // just change wanted heading with turn on game controller, and let the automatic system do the rest.
        double before_correction_turn = gamepad1.right_stick_x;
        //small value to account for slight stick drift and things like that.
        //change if necesarry - not tested
        if (Math.abs(before_correction_turn) > 0.02) {
            telemetry.addData("test correction", "heading values updated");
            heading = realHeading;
            heading = normalize(heading);
        }
        double correction_turn = calculateP(0.004, 30);
        telemetry.addData("Turn Correction", "Turn correction: " + correction_turn);
        turn = before_correction_turn + correction_turn;
        strafe = -gamepad1.left_stick_x * slowMultiplier;
        forward = -gamepad1.left_stick_y * slowMultiplier;
        if (gamepad1.dpad_down) {
            armPosition = 0;
        } else if (gamepad1.dpad_up) {
            armPosition = 1950;
        } else if (gamepad1.right_bumper) {
            armPosition = 1500;
        } else if (gamepad1.left_bumper) {
            armPosition = 2040;
        }
        if (gamepad1.x) {
            clawPosition = 1.0;
        } else if (gamepad1.b) {
            clawPosition = 0.0;
        }
        if (gamepad1.dpad_left) {
            pivotPower = -0.05;
            telemetry.addData("testpivot", "Pivot went, dpad left pressed");
        } else if (gamepad1.dpad_right) {
            pivotPower = 0.05;
        }
        if (gamepad1.y) {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armPosition = 0;
        }
        armPosition -= gamepad1.right_trigger * 30;
        armPosition += gamepad1.left_trigger * 30;
        if (gamepad2.x) {
            airPlane.setPosition(0.0);
        } else if (gamepad2.y) {
            airPlane.setPosition(0.75);
        }

        if (cooldownTicksAButton <= 0) {
            if (gamepad1.a) {
                if (slowMultiplier == 1.0) {
                    slowMultiplier = 3.0/4.0;
                } else {
                    slowMultiplier = 1.0;
                }
                cooldownTicksAButton = 20;
            }
        } else {
            cooldownTicksAButton--;
        }


        x += (Math.cos(realHeading) * forward)/100;
        y += (Math.sin(realHeading) * forward)/100;

        backLeftPower = Range.clip(((forward + turn - strafe) * slowMultiplier) / 3, -1.0, 1.0);
        backRightPower = Range.clip(((forward - turn + strafe) * slowMultiplier) / 3, -1.0, 1.0);
        frontLeftPower = Range.clip(((forward + turn + strafe) * slowMultiplier) / 3, -1.0, 1.0);
        frontRightPower = Range.clip(((forward - turn - strafe) * slowMultiplier) / 3, -1.0, 1.0);
        // Send calculated power to wheels
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        pivot.setPosition(pivot.getPosition() + pivotPower);
        claw.setPosition(clawPosition);
        arm.setTargetPosition(armPosition);
        airPlane.setPosition(airPlane.getPosition() + airPlanePower);


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", backLeftPower, backRightPower);
        telemetry.addData("Angle", "Heading: " + getAngleImu());
        telemetry.addData("TurnCorrection", "Turn Correction: " + turn);
        telemetry.addData("WantedHeading", "Wanted Heading: " + heading);
        telemetry.addData("Forward", "Forward: " + forward);
        telemetry.addData("Ultrasonic", "Ultrasonic value (cm): " + (157 * ultrasonicSensor.getVoltage()));
        telemetry.addData("ServoTest", "Servo pos, and then stuff: " + claw.getPosition());
        telemetry.addData("x and y", "X, Y: " + x + ", " + y);
        telemetry.addData("arm position", "Arm Position: " + arm.getCurrentPosition());
        telemetry.addData("slow amount", "Slow amount: " + slowMultiplier);
        telemetry.addData("a cooldown", "A cooldown: " + cooldownTicksAButton);
        telemetry.addData("armposition wanted", armPosition + " wanted arm pos");
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}