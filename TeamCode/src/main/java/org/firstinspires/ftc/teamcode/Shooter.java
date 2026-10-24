package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
public class Shooter {
    private OpMode opMode;
    private DcMotor pollenShooter;
    private DcMotor nectarShooter;
    private double rpm = 3000;

    public Shooter(LinearOpMode opMode)
    {
        pollenShooter = opMode.hardwareMap.get(DcMotor.class, "pollenShooter");
        nectarShooter = opMode.hardwareMap.get(DcMotor.class, "nectarShooter");
        nectarShooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void turnOnShooter()
    {
        pollenShooter.setPower(0.75);
        nectarShooter.setPower(0.75);
    }
    public void turnOffShooting()
    {
        pollenShooter.setPower(0);
        nectarShooter.setPower(0);
    }

}
