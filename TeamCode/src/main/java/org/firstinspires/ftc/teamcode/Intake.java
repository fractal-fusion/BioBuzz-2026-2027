
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


public class Intake {
    // front is relative to the direction that the intake is facing

    //ignoring nectar for meet 1
//    private DcMotor nectarTransferFront;
//    private DcMotor nectarTransferTop;
    private DcMotor pollenIntake;
    private DcMotor pollenTransfer;
    private Servo ramp;

    //constant variables for ramp
    public static double RAMP_UP_POSITION = 0.37;
    public static double RAMP_DOWN_POSITION = 0.07;

    //gamepad variables for toggling ramp mode
    private Gamepad currentGamepad = new Gamepad();
    private Gamepad previousGamepad = new Gamepad();
    private boolean rampMode = false;

    public static final double INTAKE_SPEED = 1; // change this to speed up or slow down intakes

    private OpMode opMode;

    public Intake(OpMode opmode)
    {
        opMode = opmode;
//        nectarTransferFront = opMode.hardwareMap.get(DcMotor.class, "nectarFront");
//        nectarTransferTop = opMode.hardwareMap.get(DcMotor.class, "nectarTop");
        pollenIntake = opMode.hardwareMap.get(DcMotor.class, "pollenintake");
        pollenTransfer = opMode.hardwareMap.get(DcMotor.class, "pollentransfer");

        ramp = opMode.hardwareMap.get(Servo.class, "intakeramp");
        ramp.setPosition(RAMP_UP_POSITION);

        pollenIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        pollenTransfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intakePollen()
    {
            pollenIntake.setPower(INTAKE_SPEED);
            pollenTransfer.setPower(INTAKE_SPEED);
    }

    public void outtakePollen()
    {
        pollenIntake.setPower(-INTAKE_SPEED);
        pollenTransfer.setPower(-INTAKE_SPEED);
    }

    public void turnOffPollen()
    {
        pollenIntake.setPower(0);
        pollenTransfer.setPower(0);
    }

    public void updateGamepad(Gamepad gamepad){
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad);
    }

    public void toggleRampMode(Gamepad gamepad){
        if (currentGamepad.dpad_down && !previousGamepad.dpad_down){ //change this to change toggle button
            rampMode = !rampMode;
        }

        if (rampMode){
            ramp.setPosition(RAMP_DOWN_POSITION);
        }
        else{
            ramp.setPosition(RAMP_UP_POSITION);
        }
    }
    public boolean isRampModeOn(){
        return rampMode;
    }

//    public void intakeNectar()
//    {
//        nectarTransferFront.setPower(INTAKE_SPEED);
//        nectarTransferTop.setPower(INTAKE_SPEED);
//    }
//
//    public void outtakeNectar()
//    {
//        nectarTransferFront.setPower(-INTAKE_SPEED);
//        nectarTransferTop.setPower(-INTAKE_SPEED);
//    }
//
//    public void turnOffNectar()
//    {
//       nectarTransferFront.setPower(0);
//       nectarTransferTop.setPower(0);
//    }
}
