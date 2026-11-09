
package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="shooterVelocityTuner", group="Robot")
public class ShooterVelocityTuner extends LinearOpMode {
    Shooter shooter;
    Intake intake;
    DcMotorEx shooterRight;
    DcMotorEx shooterLeft;
    double currentVelocity;
    public static double PID_OFFSET = 9.002; //right motor is always slower
    public static double P = 50;
    public static double I = 0.16;
    public static double D = 0;
    public static double F = 16;
    public static double TARGET_RPM = 1500;


    @Override
    public void runOpMode() {

        shooter = new Shooter(this);
        intake = new Intake(this);

        Telemetry t = telemetry;
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterright");
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoders on initialization
        shooterRight.setVelocityPIDFCoefficients(P, I, D, F);

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterleft");
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoders on initialization
        shooterLeft.setVelocityPIDFCoefficients(P, I, D, F);

        shooterLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
                intake.intakePollen();
            }
            else{
                intake.turnOffPollen();
            }

            shooterRight.setVelocity(TARGET_RPM * Shooter.RPM_TO_TICKS_PER_SECOND);
            shooterLeft.setVelocity(TARGET_RPM * Shooter.RPM_TO_TICKS_PER_SECOND);

            currentVelocity = shooterLeft.getVelocity();

            panelsTelemetry.addData("shooter velocity", shooter.shooterLeftGetVelocity());
            panelsTelemetry.addData("target velocity", TARGET_RPM);
            panelsTelemetry.update(telemetry);
            t.update();
        }
    }
}
