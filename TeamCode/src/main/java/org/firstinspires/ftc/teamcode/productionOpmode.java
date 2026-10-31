package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "productionOpmode")
public class productionOpmode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize subsystems
        Intake intake = new Intake(this);
        Shooter shooter = new Shooter(this);
        Drivetrain drivetrain = new Drivetrain( this);

        //initialize follower TODO: tune pedro so follower is not null
//        Follower follower = Constants.create(hardwareMap);
//        follower.setPose(TeleopPoseManager.currentPose);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){

            //update follower
//            follower.update();

            //GAMEPAD 1 CONTROLS
            if (gamepad1.a){
                //autoalign
//                drivetrain.autoAlignTwoStageProportionalController(drivetrain.calculateOdoGoalBearing(follower.pose(), drivetrain.getCurrentGoalPose()));
            }
            else if (gamepad1.x){
                drivetrain.resetIMU();
            }
            else{
                drivetrain.drive(gamepad1);
            }

            //GAMEPAD 2 CONTROLS
            intake.toggleRampMode(gamepad2);
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
//                shooter.setTargetRPM(shooter.calculateRPMRegression(drivetrain.calculateOdoGoalDistance(follower.pose(), drivetrain.getCurrentGoalPose())));
//                shooter.setHoodPosition(shooter.calculateHoodRegression(drivetrain.calculateOdoGoalDistance(follower.pose(), drivetrain.getCurrentGoalPose())));
                shooter.updateShooterVelocity();
            }
            else {
                shooter.turnOff();
                //or keep shooter on at a lower rpm
//                shooter.turnOffRestingRpm();
            }

            //update everything
            drivetrain.updateIMU();


            //shooter updates
            //automatically opens gate when shooter passes threshold
            shooter.controlGate();

            shooter.updateGamepad(gamepad2);
            shooter.updateShooterThreshold();

            //intake updates
            intake.updateGamepad(gamepad2);

            //drivetrain updates
//            drivetrain.updateCurrentGoalPose(follower.pose());

            telemetry.update();

        }
    }
}
