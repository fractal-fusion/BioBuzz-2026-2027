package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Configurable
@TeleOp(name = "productionOpmode")
public class productionOpmode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //initialize subsystems
        Intake intake = new Intake(this);
        Shooter shooter = new Shooter(this);
        Drivetrain drivetrain = new Drivetrain( this);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){

            //GAMEPAD 1 CONTROLS
            if (gamepad1.a){
                //autoalign
            }
            else if (gamepad1.x){
                drivetrain.resetIMU();
            }
            else{
                drivetrain.drive(gamepad1);
            }

            //GAMEPAD 2 CONTROLS

            shooter.toggleShooter(gamepad2);
            if (gamepad2.a){
                if (!shooter.isOn()) {
                    shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
                }
                intake.intakePollen();
            }
            else if (gamepad2.b){
                intake.outtakePollen();
            }
            else{
                intake.turnOffPollen();
            }

            //SHOOTER VELOCITY LOGIC
            if (shooter.isOn()){
                //shooter.setTargetRPM(calculate regression rpm method);
                //shooter.setHoodPosition(calclate ramp regression method)
                shooter.updateShooterVelocity();
            }
            else {
                shooter.turnOff();
                //or keep shooter on at a lower rpm
//                shooter.turnOffRestingRpm();
            }

            //update everything
            drivetrain.updateIMU();

            //automatically opens gate when shooter passes threshold
            shooter.controlGate();

            shooter.updateGamepad(gamepad2);
            shooter.updateShooterThreshold();

            telemetry.update();

        }
    }
}
