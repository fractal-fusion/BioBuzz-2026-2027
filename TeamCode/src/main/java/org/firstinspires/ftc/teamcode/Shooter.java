package org.firstinspires.ftc.teamcode;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Shooter {
    private OpMode opMode;
    private DcMotor shooterLeft;
    private DcMotor shooterRight;

    public static double staticRPM = 1935;
    private Servo gate;
    private Servo hood;
    private boolean isOn = false;

    //RPM CONTROL
    public boolean passedRPMThreshold = false; //boolean for once the shooter reaches velocity

    public static double RPM_TO_TICKS_PER_SECOND = 28.0 / 60.0; //divide rpm by 60 to get rotations per second, then multiply by 28 since that's the ticks per revolution
    public static double TICKS_PER_SECOND_TO_RPM = 60.0 / 28.0; //inverse of above

    public double currentTargetRPMTicksPerSecond = REGRESSION_RESTING_RPM * RPM_TO_TICKS_PER_SECOND;
    public static double TARGET_RPM_TOLERANCE = 10;
    public static double LOWER_THRESHOLD_RPM_OFFSET = 80;

    //.setVelocity PIDF values
    public static double P = 100;
    public static double I = 0.15;
    public static double D = 0;
    public static double F = 15;

    //gate variables
    public static double GATE_OPEN_POSITION = 0.01;
    public static double GATE_CLOSED_POSITION = 0.13;

    //hood variables
    public static double HOOD_OFFSET = 0.00; //offset so the hood is in the right place
    public static double HOOD_MAX_POSITION = 0.36;

    //regression variables
    public static double REGRESSION_RPM_OFFSET = -25;
    public static double REGRESSION_HOOD_OFFSET = -0.05;
    public static double REGRESSION_RESTING_RPM = 1500; //lower rpm to keep the shooter running at

    //shooter position offset for autoalign
    public static double POLLEN_X_OFFSET = 0;
    public static double POLLEN_Y_OFFSET = 3;



    //Gamepads for rising edge detector
    private Gamepad currentGamepad = new Gamepad();
    private Gamepad previousGamepad = new Gamepad();

    public Shooter(LinearOpMode opMode)
    {
        shooterLeft = opMode.hardwareMap.get(DcMotor.class, "shooterleft");
        shooterRight = opMode.hardwareMap.get(DcMotor.class, "shooterright");

        gate = opMode.hardwareMap.get(Servo.class, "gate");
        hood = opMode.hardwareMap.get(Servo.class, "hood");

        //reverse hood so increase in position increases hood
        hood.setDirection(Servo.Direction.REVERSE);
        //reset hood down to 0
        setHoodPosition(0);

        //open gate
        gate.setPosition(GATE_OPEN_POSITION);

        //reset right motor so both motors spin forward with a power of 1
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //reset encoders on initialization
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ((DcMotorEx) shooterLeft).setVelocityPIDFCoefficients(P, I, D, F);
        ((DcMotorEx) shooterRight).setVelocityPIDFCoefficients(P, I, D, F);
    }
    public void updateShooterVelocity(){
        ((DcMotorEx) shooterRight).setVelocity(currentTargetRPMTicksPerSecond);
        ((DcMotorEx) shooterLeft).setVelocity(currentTargetRPMTicksPerSecond);
    }
    public void turnOff(){
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ((DcMotorEx) shooterRight).setVelocity(0);
        ((DcMotorEx) shooterLeft).setVelocity(0);
    }
    public void turnOffRestingRpm(){
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ((DcMotorEx) shooterRight).setVelocity(REGRESSION_RESTING_RPM * RPM_TO_TICKS_PER_SECOND);
        ((DcMotorEx) shooterLeft).setVelocity(REGRESSION_RESTING_RPM * RPM_TO_TICKS_PER_SECOND);
    }

    public void setTargetRPM(double RPM) {
        currentTargetRPMTicksPerSecond = RPM * RPM_TO_TICKS_PER_SECOND;
    }
    public boolean atTargetVelocity() {
        return ((DcMotorEx) shooterLeft).getVelocity() >= currentTargetRPMTicksPerSecond - (TARGET_RPM_TOLERANCE * RPM_TO_TICKS_PER_SECOND)
                && ((DcMotorEx) shooterRight).getVelocity() >= currentTargetRPMTicksPerSecond - (TARGET_RPM_TOLERANCE * RPM_TO_TICKS_PER_SECOND);
    }
    public boolean atLowerThresholdVelocity() {
        return ((DcMotorEx) shooterLeft).getVelocity() <= (currentTargetRPMTicksPerSecond - (LOWER_THRESHOLD_RPM_OFFSET * RPM_TO_TICKS_PER_SECOND))
                && ((DcMotorEx) shooterRight).getVelocity() <= (currentTargetRPMTicksPerSecond - (LOWER_THRESHOLD_RPM_OFFSET * RPM_TO_TICKS_PER_SECOND));
    }

    public void updateShooterThreshold(){
//        opMode.telemetry.addData("passed threshold", passedThreshold);
        if (atTargetVelocity()) {
            passedRPMThreshold = true;
        } else if (atLowerThresholdVelocity()) {
            passedRPMThreshold = false;
        }
    }

    public void turnOnShooterPower()
    {
        shooterLeft.setPower(0.75);
        shooterRight.setPower(0.75);
    }
    public void turnOffShootingPower()
    {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    //GATE METHODS
    public void setGatePosition(double position){
        gate.setPosition(position);
    }
    public void controlGate(Gamepad gamepad){
        if (isOn && passedRPMThreshold){
            gate.setPosition(GATE_OPEN_POSITION);
            gamepad.rumbleBlips(1);
        }
    }

    //HOOD METHODS

    public void setHoodPosition(double position){
        hood.setPosition(position + HOOD_OFFSET);
    }
    public void updateGamepad(Gamepad gamepad){
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad);
    }

    //regression calculation methods
    public double calculateRPMRegression(double distance){
        return Range.clip(0.0000423963 * Math.pow(distance, 4) - 0.0069645 * Math.pow(distance, 3) + 0.387821 * Math.pow(distance, 2) - 5.55246 * distance + 1502.84812, 1520, 1620);
    }

    public double calculateHoodRegression(double distance){
        return Range.clip(0.000000605274 * Math.pow(distance, 4) - 0.000105842 * Math.pow(distance, 3) + 0.0065738 * Math.pow(distance, 2) - 0.166008 * distance + 1.6127, 0.00, 0.31);
    }

    //variable getters
    public boolean isOn(){
        return isOn;
    }

    public double shooterLeftGetVelocity() {
        return (((DcMotorEx) shooterLeft).getVelocity() * Shooter.TICKS_PER_SECOND_TO_RPM);
    }

    public double shooterRightGetVelocity() {
        return (((DcMotorEx) shooterRight).getVelocity() * Shooter.TICKS_PER_SECOND_TO_RPM);
    }

    //                         |
    // Shooter Motor controls  |
    //                        \ /

    public void toggleShooter(Gamepad gamepad)
    {
        if (currentGamepad.x && !previousGamepad.x){
            isOn = !isOn;
        }
    }

    //AUTO COMMANDS

    public Command setRPM(double target) {
        return sequential(
                instant(() -> setTargetRPM(target)),
                instant(this::updateShooterVelocity)
        );
    }

    public Command setHood(double target){
        return instant(() -> setHoodPosition(target));
    }

    public Command openGate(){
        return instant(() -> gate.setPosition(GATE_OPEN_POSITION));
    }

    public Command closeGate(){
        return instant(() -> gate.setPosition(GATE_CLOSED_POSITION));
    }

}
