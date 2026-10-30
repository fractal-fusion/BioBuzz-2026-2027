
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class Intake {
    // front is relative to the direction that the intake is facing

    //ignoring nectar for meet 1
//    private DcMotor nectarTransferFront;
//    private DcMotor nectarTransferTop;
    private DcMotor pollenIntake;
    private DcMotor pollenTransfer;

    public static final double INTAKE_SPEED = 0.5; // change this to speed up or slow down intakes

    private OpMode opMode;

    public Intake(OpMode opmode)
    {
        opMode = opmode;
//        nectarTransferFront = opMode.hardwareMap.get(DcMotor.class, "nectarFront");
//        nectarTransferTop = opMode.hardwareMap.get(DcMotor.class, "nectarTop");
        pollenIntake = opMode.hardwareMap.get(DcMotor.class, "pollenintake");
        pollenTransfer = opMode.hardwareMap.get(DcMotor.class, "pollentransfer");

        //todo: (Vicente) check if you need to reverse a motor after you connect it.

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
