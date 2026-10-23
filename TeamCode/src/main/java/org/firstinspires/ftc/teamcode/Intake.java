
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Intake {
    // front is relative to the direction that the intake is facing
    private DcMotor nectarFront;
    private DcMotor nectarTop;
    private DcMotor pollenFront;
    private DcMotor pollenTop;
    public static final double INTAKE_SPEED = 0.5; // change this to speed up or slow down intakes

    private OpMode opMode;

    public Intake(OpMode opmode)
    {
        opMode = opmode;
        nectarFront = opMode.hardwareMap.get(DcMotor.class, "nectarFront");
        nectarTop = opMode.hardwareMap.get(DcMotor.class, "nectarTop");
        pollenFront = opMode.hardwareMap.get(DcMotor.class, "pollenFront");
        pollenTop = opMode.hardwareMap.get(DcMotor.class, "pollenTop");

        //todo: (Vicente) check if you need to reverse a motor after you connect it.
    }

    public void forwardPollen()
    {
            pollenFront.setPower(INTAKE_SPEED);
            pollenTop.setPower(INTAKE_SPEED);
    }

    public void reversePollen()
    {
        pollenFront.setPower(-INTAKE_SPEED);
        pollenTop.setPower(-INTAKE_SPEED);
    }

    public void turnOffPollen()
    {
        pollenFront.setPower(0);
        pollenTop.setPower(0);
    }

    public void forwardNectar()
    {
        nectarFront.setPower(INTAKE_SPEED);
        nectarTop.setPower(INTAKE_SPEED);
    }

    public void reverseNectar()
    {
        nectarFront.setPower(-INTAKE_SPEED);
        nectarTop.setPower(-INTAKE_SPEED);
    }

    public void turnOffNectar()
    {
       nectarFront.setPower(0);
       nectarTop.setPower(0);
    }
}
