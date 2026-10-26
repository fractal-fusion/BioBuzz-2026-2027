package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Shooter {
    private OpMode opMode;
    private DcMotor shooterLeft;
    private DcMotor shooterRight;
    private double rpm = 3000;
    private boolean isOn = false;

    //RPM CONTROL
    public boolean passedRPMThreshold = false; //boolean for once the shooter reaches velocity

    public static double RPM_TO_TICKS_PER_SECOND = 28.0 / 60.0; //divide rpm by 60 to get rotations per second, then multiply by 28 since that's the ticks per revolution
    public double currentTargetRPMTicksPerSecond = REGRESSION_RESTING_RPM * RPM_TO_TICKS_PER_SECOND;
    public static double TARGET_RPM_TOLERANCE = 50;
    public static double LOWER_THRESHOLD_RPM_OFFSET = 50;
    public static double REGRESSION_RPM_ODOMETRY = 0;
    public static double REGRESSION_RESTING_RPM = 2800;

    //.setVelocity PIDF values
    //TODO: tune pidf values
    public static double P = 220;
    public static double I = 0.21;
    public static double D = 0;
    public static double F = 12.20004;

    //
    private Gamepad currentGamepad = new Gamepad();
    private Gamepad previousGamepad = new Gamepad();

    public Shooter(LinearOpMode opMode)
    {
        shooterLeft = opMode.hardwareMap.get(DcMotor.class, "leftshooter");
        shooterRight = opMode.hardwareMap.get(DcMotor.class, "rightshooter");

        //reset right motor so both motors spin forward with a power of 1
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //reset encoders on initialization
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        ((DcMotorEx) shooterLeft).setVelocityPIDFCoefficients(P, I, D, F);
//        ((DcMotorEx) shooterRight).setVelocityPIDFCoefficients(P, I, D, F);
    }

    public void turnOn(){
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
    public void updateShooterVelocity(){
        ((DcMotorEx) shooterRight).setVelocity(currentTargetRPMTicksPerSecond);
        ((DcMotorEx) shooterLeft).setVelocity(currentTargetRPMTicksPerSecond);
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

    public void updateGamepad(Gamepad gamepad){
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad);
    }

    //variable getters
    public boolean isOn(){
        return isOn;
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

}
