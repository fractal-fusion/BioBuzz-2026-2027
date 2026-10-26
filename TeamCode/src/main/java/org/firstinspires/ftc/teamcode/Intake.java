
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Intake {
    // front is relative to the direction that the intake is facing
    private DcMotor nectarTransferFront;
    private DcMotor nectarTransferTop;
    private DcMotor pollenTransferFront;
    private DcMotor pollenTransferTop;

    public static final double INTAKE_SPEED = 0.5; // change this to speed up or slow down intakes

    private OpMode opMode;

    public Intake(OpMode opmode)
    {
        opMode = opmode;
        nectarTransferFront = opMode.hardwareMap.get(DcMotor.class, "nectarFront");
        nectarTransferTop = opMode.hardwareMap.get(DcMotor.class, "nectarTop");
        pollenTransferFront = opMode.hardwareMap.get(DcMotor.class, "pollenFront");
        pollenTransferTop = opMode.hardwareMap.get(DcMotor.class, "pollenTop");

        //todo: (Vicente) check if you need to reverse a motor after you connect it.
    }

    public void intakePollen()
    {
            pollenTransferFront.setPower(INTAKE_SPEED);
            pollenTransferTop.setPower(INTAKE_SPEED);
    }

    public void outtakePollen()
    {
        pollenTransferFront.setPower(-INTAKE_SPEED);
        pollenTransferTop.setPower(-INTAKE_SPEED);
    }

    public void turnOffPollen()
    {
        pollenTransferFront.setPower(0);
        pollenTransferTop.setPower(0);
    }

    public void intakeNectar()
    {
        nectarTransferFront.setPower(INTAKE_SPEED);
        nectarTransferTop.setPower(INTAKE_SPEED);
    }

    public void outtakeNectar()
    {
        nectarTransferFront.setPower(-INTAKE_SPEED);
        nectarTransferTop.setPower(-INTAKE_SPEED);
    }

    public void turnOffNectar()
    {
       nectarTransferFront.setPower(0);
       nectarTransferTop.setPower(0);
    }
}
