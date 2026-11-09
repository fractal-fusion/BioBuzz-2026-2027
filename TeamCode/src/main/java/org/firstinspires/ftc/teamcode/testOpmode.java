package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "testOpmode")
public class testOpmode extends LinearOpMode {

    public static String testServo1Name = "testservo1";
    public static String testServo2Name = "testservo2";
    public static String testMotor1Name = "testmotor1";
    public static String testMotor2Name = "testmotor2";


    public static String testServoName = "intakeramp";

    public double testPosition = 0;
    public double increment = 0.01;
    boolean pressed = false;



    CRServo testServo1;
    CRServo testServo2;
    DcMotor testMotor1;
    DcMotor testMotor2;
    Servo testServo;

    @Override
    public void runOpMode() throws InterruptedException {

        //Intake intake = new Intake(this);

        Shooter shooter = new Shooter(this);
        Drivetrain drivetrain = new Drivetrain( this);
        Intake intake = new Intake(this);

//        testServo1 = hardwareMap.get(CRServo.class, testServo1Name);
//        testServo2 = hardwareMap.get(CRServo.class, testServo2Nam
//        e);
//
//        testMotor1 = hardwareMap.get(DcMotor.class, testMotor1Name);
//        testMotor2 = hardwareMap.get(DcMotor.class, testMotor2Name);

        testServo = hardwareMap.get(Servo.class, testServoName);

//        testServo.setDirection(Servo.Direction.REVERSE);


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){



//            drivetrain.drive(gamepad1);

            if (gamepad1.a){
                intake.intakePollen();
            }
            else if (gamepad1.b) {
                intake.outtakePollen();
            }
            else
            {
                intake.turnOffPollen();
            }
//
//            if (gamepad1.x)
//            {
//                shooter.turnOnShooter();
//            }
//            else if(gamepad1.y)
//            {
//                shooter.turnOffShooting();
//            }


            if(gamepad1.right_bumper && !gamepad1.left_bumper && !pressed){
                testPosition += increment;
                pressed = true;
            }
            else if (gamepad1.left_bumper && !gamepad1.right_bumper && !pressed){
                testPosition -= increment;
                pressed = true;
            }
            else if (!gamepad1.right_bumper && !gamepad1.left_bumper && pressed){
                pressed = false;
            }

            testPosition = Math.max(0, Math.min(testPosition, 1));

            testServo.setPosition(testPosition);

            telemetry.addData("servopos:", testPosition);
            telemetry.addData("shooter regression velocity:", shooter.calculateRPMRegression(drivetrain.calculateOdoGoalDistance(new Pose(55.72, 133.25), drivetrain.getCurrentGoalPose())) + Shooter.REGRESSION_RPM_OFFSET);
            telemetry.addData("shooter regression hood:", shooter.calculateHoodRegression(drivetrain.calculateOdoGoalDistance(new Pose(55.72, 133.25), drivetrain.getCurrentGoalPose())));
            telemetry.update();

        }
    }
}
