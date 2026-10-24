package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
@Configurable
@TeleOp(name = "testOpmode")
public class testOpmode extends LinearOpMode {

    public static String testServo1Name = "testservo1";
    public static String testServo2Name = "testservo2";
    public static String testMotor1Name = "testmotor1";
    public static String testMotor2Name = "testmotor2";



    CRServo testServo1;
    CRServo testServo2;
    DcMotor testMotor1;
    DcMotor testMotor2;

    @Override
    public void runOpMode() throws InterruptedException {

        //Intake intake = new Intake(this);

        Shooter shooter = new Shooter(this);
        Drivetrain drivetrain = new Drivetrain( this);

        testServo1 = hardwareMap.get(CRServo.class, testServo1Name);
        testServo2 = hardwareMap.get(CRServo.class, testServo2Name);

        testMotor1 = hardwareMap.get(DcMotor.class, testMotor1Name);
        testMotor2 = hardwareMap.get(DcMotor.class, testMotor2Name);



        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){

            drivetrain.drive(gamepad1);

            if (gamepad1.a){
//                testServo1.setPower(1);
//                testServo2.setPower(1);

                testMotor1.setPower(-1);
                testMotor2.setPower(-1);
            }
            else if (gamepad1.b){
//                testServo1.setPower(-1);
//                testServo2.setPower(-1);
//
                testMotor1.setPower(1);
                testMotor2.setPower(1);
            }
            else
            {
                testMotor1.setPower(0);
                testMotor2.setPower(0);
            }

            if (gamepad1.x)
            {
                shooter.turnOnShooter();
            }
            else if(gamepad1.y)
            {
                shooter.turnOffShooting();
            }
            telemetry.addLine("hello world");
            telemetry.update();

        }
    }
}
