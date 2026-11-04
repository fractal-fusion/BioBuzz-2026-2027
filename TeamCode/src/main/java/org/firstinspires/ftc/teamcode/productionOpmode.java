package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Configurable
@TeleOp(name = "productionOpmode")
public class productionOpmode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //manual hood variables //TODO: REMOVE
        double increment = 0.01;
        double testPosition = 0;
        boolean pressed = false;



        //initialize subsystems
        Intake intake = new Intake(this);
        Shooter shooter = new Shooter(this);
        Drivetrain drivetrain = new Drivetrain( this);

        //initialize follower
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(TeleopPoseManager.currentPose);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){

            //update follower
            follower.update();

            //GAMEPAD 1 CONTROLS
            if (gamepad1.a){
                //autoalign
                drivetrain.driveAutoAlign(gamepad1, drivetrain.autoAlignTwoStageProportionalController(-drivetrain.calculateOdoGoalBearing(drivetrain.calculatePollenShooterPose(follower.pose(), Shooter.POLLEN_X_OFFSET, Shooter.POLLEN_Y_OFFSET), drivetrain.getCurrentGoalPose())));
            }
            else if (gamepad1.x){
                drivetrain.resetIMU();
            }
            else{
                drivetrain.drive(gamepad1);
            }

            //relocalization logic
            if (gamepad1.dpad_down){
                follower.setPose(TeleopPoseManager.RED_BOTTOM_LEFT_POSE);
            }

            //GAMEPAD 2 CONTROLS
            intake.toggleRampMode(gamepad2);
            shooter.toggleShooter(gamepad2);
            if (gamepad2.a){
                if (!shooter.isOn()) {
                    shooter.setGatePosition(Shooter.GATE_CLOSED_POSITION);
                    intake.intakePollen();
                }
                else {
                    intake.intakePollenSlow();
                }
            }
            else if (gamepad2.b){
                intake.outtakePollen();
            }
            else{
                intake.turnOffPollen();
            }

            //SHOOTER VELOCITY LOGIC
            if (shooter.isOn()){
//                shooter.setTargetRPM(Shooter.staticRPM);
                shooter.setTargetRPM(shooter.calculateRPMRegression(drivetrain.calculateOdoGoalDistance(follower.pose(), drivetrain.getCurrentGoalPose())));
                shooter.setHoodPosition(shooter.calculateHoodRegression(drivetrain.calculateOdoGoalDistance(follower.pose(), drivetrain.getCurrentGoalPose())));
                shooter.updateShooterVelocity();
            }
            else {
//                shooter.turnOff();
                //or keep shooter on at a lower rpm
                shooter.turnOffRestingRpm();
                shooter.setHoodPosition(0);
            }

            //manual hood control TODO:REMOVE
//            if(gamepad2.right_bumper && !gamepad2.left_bumper && !pressed){
//                testPosition += increment;
//                pressed = true;
//            }
//            else if (gamepad2.left_bumper && !gamepad2.right_bumper && !pressed){
//                testPosition -= increment;
//                pressed = true;
//            }
//            else if (!gamepad2.right_bumper && !gamepad2.left_bumper && pressed){
//                pressed = false;
//            }
//
//            testPosition = Math.max(0, Math.min(testPosition, 0.36));
//
//            shooter.setHoodPosition(testPosition);


            //telemetry
            telemetry.addData("servopos:", testPosition);
            telemetry.addData("shooter velocity:", shooter.shooterLeftGetVelocity());
            telemetry.addData("current pose:", follower.pose());

            telemetry.addData("current distance:", drivetrain.calculateOdoGoalDistance(follower.pose(), drivetrain.getCurrentGoalPose()));
            telemetry.addData("currentGoal", drivetrain.getCurrentGoalPose());


            //update everything

            //update all telemetry
            telemetry.update();

            //update drivetrain imu for field centric
            drivetrain.updateIMU();


            //shooter updates
            //automatically opens gate when shooter passes threshold
            shooter.controlGate();

            shooter.updateGamepad(gamepad2);
            shooter.updateShooterThreshold();

            //intake updates
            intake.updateGamepad(gamepad2);

            //drivetrain updates
            drivetrain.updateCurrentGoalPose(follower.pose());

            //update follower on panels
            Drawing.drawDebug(follower);

            telemetry.update();

        }
    }
}
