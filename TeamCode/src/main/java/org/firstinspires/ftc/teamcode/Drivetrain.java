package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
public class Drivetrain {
    //declare motors
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor testWheel;

    public static String testWheelHardwareMapName = "frontright";
    public IMU imu;

    //botHeading used for field centric drive
    private double botHeading;

    //constants used for tuning auto alignment
    public static double AUTO_ALIGN_MAX_SPEED = 0.8; //auto alignment speed is clipped to minimum negative this and maximum positive this (bilateral tolerance)
    public static double AUTO_ALIGN_GAIN_LIMELIGHT = 0.0175; //converts tx from limelight to power

    public static double AUTO_ALIGN_DRIVE_POWER_MULTIPLIER_MIDPOINT = 0.45; //half of max power
    public Gamepad currentGamepad = new Gamepad();
    public Gamepad previousGamepad = new Gamepad();
    private OpMode opMode;


    //constructor which acts as an initialization function for whenever an object of the class is created
    public Drivetrain(OpMode linearopmode)
    {
        this.opMode = linearopmode;
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "frontleft");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "backleft");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "frontright");
        backRight = opMode.hardwareMap.get(DcMotor.class, "backright");

        testWheel = opMode.hardwareMap.get(DcMotor.class, testWheelHardwareMapName);

        imu = opMode.hardwareMap.get(IMU.class, "imu");

        // TODO: adjust the orientation parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // actually set the parameters
        imu.initialize(parameters);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    public void updateIMU() {
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    public void drive(Gamepad gamepad)
    {
        //get gamepad inputs
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading); //always offset the robot's heading to only move in the four cardinal directions
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1; //counter imperfect strafing

        //speed control with triggers
        double maxSpeed = 0.5;
        double maxSpeedMultiplier;
        maxSpeedMultiplier = maxSpeed + ((-gamepad.right_trigger * (maxSpeed * 0.5)) + (gamepad.left_trigger * maxSpeed));

        //solve for power
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        //multiply calculated power with the speed control to obtain final power for the motors
        frontLeft.setPower(frontLeftPower * maxSpeedMultiplier);
        backLeft.setPower(backLeftPower * maxSpeedMultiplier);
        frontRight.setPower(frontRightPower * maxSpeedMultiplier);
        backRight.setPower(backRightPower * maxSpeedMultiplier);
    }
    public void driveAutoAlign(Gamepad gamepad, double rotation)
    {

        double maxSpeed = AUTO_ALIGN_DRIVE_POWER_MULTIPLIER_MIDPOINT;
        double maxSpeedMultiplier;
        maxSpeedMultiplier = maxSpeed + ((-gamepad.right_trigger * (maxSpeed * 0.5)) + (gamepad.left_trigger * maxSpeed));

        //get gamepad inputs
        double ypower = -gamepad.left_stick_y * maxSpeedMultiplier; // lower the power when auto aligning
        double xpower = gamepad.left_stick_x * (maxSpeedMultiplier + 0.01); //multiplied by an extra 0.1 to counter imperfect strafing
        double rotationpower = rotation;

        double rotX = xpower * Math.cos(-botHeading) - ypower * Math.sin(-botHeading); //always offset the robot's heading to only move in the four cardinal directions
        double rotY = xpower * Math.sin(-botHeading) + ypower * Math.cos(-botHeading);
        rotX = rotX * 1.1; //counter imperfect strafing

        //solve for power
        double denominator = Math.max(Math.abs(ypower) + Math.abs(xpower) + Math.abs(rotationpower), 1);
        double frontLeftPower = (rotY + rotX + rotationpower) / denominator;
        double backLeftPower = (rotY - rotX + rotationpower) / denominator;
        double frontRightPower = (rotY - rotX - rotationpower) / denominator;
        double backRightPower = (rotY + rotX - rotationpower) / denominator;

        //multiply calculated power with the speed control to obtain final power for the motors
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public double calculateAutoAlignPowerLimelight(double bearing) {
        return Range.clip(bearing * AUTO_ALIGN_GAIN_LIMELIGHT, -AUTO_ALIGN_MAX_SPEED, AUTO_ALIGN_MAX_SPEED);
    }
}


