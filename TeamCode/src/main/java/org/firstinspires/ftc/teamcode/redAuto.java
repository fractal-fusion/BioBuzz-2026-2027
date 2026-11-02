package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

@Configurable
@Autonomous(name = "\uD83D\uDD34 redAuto")
public class redAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        //Since the scheduler is static, we need to reset it before each OpMode
        //so commands don't carry over from one OpMode to the next
        Scheduler.reset();
        // Initialize hardware
        Shooter shooter = new Shooter(this);
        Intake intake = new Intake(this);

        // Compose: raise the arm, wait 200ms, then open the claw

        Command autoSequence = sequential(
            shooter.turnOnShooter(3000)
        );
        waitForStart();

        // Schedule the sequence when the OpMode starts
        schedule(autoSequence);
        while (opModeIsActive()) {
            // Run the scheduler each loop
            Scheduler.execute();
        }
    }
}
