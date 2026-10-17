package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "testOpmode")
public class testOpmode extends LinearOpMode {

    public static String testServo1Name = "";
    public static String testServo2Name = "";
    public static String testMotor1Name = "";
    public static String testMotor2Name = "";

    CRServo testServo1;


    @Override
    public void runOpMode() throws InterruptedException {

        testServo1 = hardwareMap.get(CRServo.class, testServo1Name);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            if (gamepad1.a){
                testServo1.setPower(0.25);
            }
            else if (gamepad1.b){
                testServo1.setPower(-0.25);
            }

        }
    }
}
