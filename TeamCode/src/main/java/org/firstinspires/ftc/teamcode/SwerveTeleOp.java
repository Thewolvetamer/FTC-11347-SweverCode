// ***********************************************************************
// SwerveTeleOp
// ***********************************************************************
// The tele-op mode for swerve robot operations

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// ***********************************************************************
// Definitions from Qualcomm code for OpMode recognition
// ***********************************************************************
@TeleOp(name="Swerve: 2-TeleOp 1.1", group="Swerve")
//@Disabled
public class SwerveTeleOp extends SwerveCore {
    // Note when we are approaching the end of the game
    Boolean inEndGame;

    enum autoScoring {
        DRIVE_FORWARD,
        EXTEND,
        INTAKE,
        TURN_LEFT,
        TURN_RIGHT,
        LANDER
    }
    private autoScoring curScoreState;

    // ***********************************************************************
    // SwerveTeleOp
    // ***********************************************************************
    // Constructs the class.
    // The system calls this member when the class is instantiated.
    public SwerveTeleOp() {

    }

    // ***********************************************************************
    // Init
    // ***********************************************************************
    // Performs any actions that are necessary when the OpMode is enabled.
    // The system calls this member once when the OpMode is enabled.
    @Override
    public void init() {
        swerveDebug(500, "SwerveTeleOp::init", "START");

        // Run initialization of other parts of the class
        // Note that the class will connect to all of our motors and servos
        super.init();



        // We are just starting, so not in the end game yet...
        inEndGame = Boolean.FALSE;

        swerveDebug(500, "SwerveTeleOp::init", "DONE");
    }


    // ***********************************************************************
    // start
    // ***********************************************************************
    // Do first actions when the start command is given.
    // Called once when the OpMode is started.
    @Override
    public void start() {
        swerveDebug(500, "SwerveTeleOp::start", "START");

        // Call the super/base class start method.
        super.start();

        ourSwerve.curSwerveMode = SwerveDrive.swerveModes.SWERVE_DRIVER;

        swerveDebug(500, "SwerveTeleOp::start", "DONE");
    }


    // ***********************************************************************
    // loop
    // ***********************************************************************
    // State machine for autonomous robot control
    // Called continuously while OpMode is running
    @Override
    public void loop() {
//        double totalPower;
//        int endGameTime;

        swerveDebug(2000, "SwerveTeleOp::loop", "START");

        // set swerve drive orientation automation level based on driver request
        if (gamepad1.a) {
            ourSwerve.setSwerveMode(SwerveDrive.swerveModes.SWERVE_AUTO);
        }
        if (gamepad1.b) {
            ourSwerve.setSwerveMode(SwerveDrive.swerveModes.SWERVE_DRIVER);
        }
        if (gamepad1.x) {
            ourSwerve.setSwerveMode(SwerveDrive.swerveModes.SWERVE_DRIVE_ORIENT);
        }
        if (gamepad1.y) {
            ourSwerve.setSwerveMode(SwerveDrive.swerveModes.SWERVE_DRIVE_TURN);
        }

        // Move the robot, flipping y since the joysticks are upside down
        ourSwerve.driveRobot(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, gamepad1.right_stick_y);

        // *** use buttons to trigger other actions ***

        strafeR();

        strafeL();

        dropTeamIcon();

        climb();

//        wrist();

        hSlide();

        vSlide();

        vSlide.setPower(0);

//        autoScore(gamepad2.a, gamepad2.b, gamepad2.dpad_left, gamepad2.dpad_right);

        ourSwerve.distance(heightL.getDistance(DistanceUnit.CM), heightR.getDistance(DistanceUnit.CM));


        // Any loop background updates happen now....
        loopEndReporting();

        swerveDebug(500, "SwerveTeleOp::loop", "DONE");
    }


    // ***********************************************************************
    // stop
    // ***********************************************************************
    // Performs any actions that are necessary when the OpMode is disabled.
    // The system calls this member once when the OpMode is disabled.
    @Override
    public void stop() {
        swerveDebug(500, "SwerveTeleOp::stop", "START");

        // Call the super/base class stop method
        super.stop();

        swerveDebug(500, "SwerveTeleOp::stop", "DONE");
    }

    //lifrArm,dropTeamIcon,ballshooter created and tested on 10/8/18
    private void dropTeamIcon() {

        if (gamepad2.b) {
            // move to 180 degrees
            gameMarkDrop.setPosition(1);
        } else {
            gameMarkDrop.setPosition(0);
        }

        //telemetry.addData("Servo Position", gameMarkDrop.getPosition());
        //telemetry.addData("Status", "Running");
        //telemetry.update();
    }



    // makes it easier to go directly sideways
    private void strafeR() {

        if (gamepad1.dpad_left) {
            swerveLeftFront.updateWheel(1, -0.50);
            swerveRightFront.updateWheel(1, -0.50);
            swerveLeftRear.updateWheel(1, -0.50);
            swerveRightRear.updateWheel(1, -0.50);
        }
    }

    private void strafeL() {

        if (gamepad1.dpad_right) {
            swerveLeftFront.updateWheel(1, 0.50);
            swerveRightFront.updateWheel(1, 0.50);
            swerveLeftRear.updateWheel(1, 0.50);
            swerveRightRear.updateWheel(1, 0.50);
        }
    }

    private void climb() {
        if(gamepad2.dpad_up ) {
            climber.setPower(.6);
        }
        else if(gamepad2.dpad_left) {
            while(heightL.getDistance(DistanceUnit.CM) < 25) {
                climber.setPower(1);
            }
        }
        else if(gamepad2.dpad_down) {
            climber.setPower(-1);
        }
        else {
            climber.setPower(0);
        }
    }

    private void vSlide() {
        if(gamepad2.a) {
            vSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            distance/circumference of spool   * tpr
            vSlide.setTargetPosition(1);

            vSlide.setPower(1);
        }
        else if(vSlide.getTargetPosition() == vSlide.getCurrentPosition()) {
            dump.setPosition(1);
            double t = getRuntime();
            if(getRuntime() == t + 1000) {
                dump.setPosition(0);
                vSlide.setTargetPosition(0);
                vSlide.setPower(-1);
            }
        }
        else {
            vSlide.setPower(0);
        }
    }

    private void hSlide() {
        hSlide.setPower(gamepad2.right_stick_y);
    }

    private void wrist() {
        if(intake.getPosition() == 1) {
            wristR.setPosition(1);
            wristL.setPosition(1);
        }
        if(intake.getPosition() == 0){
            swerveSleep(750);
            if(intake.getPosition() == 0) {
                wristL.setPosition(0);
                wristR.setPosition(0);
            }
        }
    }


//    public void autoScore(boolean button, boolean back, boolean left, boolean right) {
//        hSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        if(ourSwerve.curSwerveMode == SwerveDrive.swerveModes.SWERVE_AUTO) {
//            switch(curScoreState) {
//                case DRIVE_FORWARD:
//                    if(button) {
//                        curScoreState = autoScoring.EXTEND;
//                    }
//                    else if(back) {
//                        curScoreState = autoScoring.LANDER;
//                    }
//                    else{
//                        ourSwerve.driveRobot(1, 0, 0, 0);
//                    }
//                    break;
//
//                case EXTEND:
//                    if(button){
//                        curScoreState = autoScoring.INTAKE;
//                    }
//                    else if(back) {
//                        curScoreState = autoScoring.DRIVE_FORWARD;
//                    }
//                    else {
//                        hSlide.setPower(.7);
//                    }
//                    break;
//
//                case INTAKE:
//                    intake.setPosition(1);
//                    if(left) {
//                        ourSwerve.driveRobot(0,0, -1, 0);
//                    }
//                    else if(right) {
//                        ourSwerve.driveRobot(0,0, 1, 0);
//                    }
//                    else if(button) {
//                        curScoreState = autoScoring.LANDER;
//                    }
//                    else if(back) {
//                        curScoreState = autoScoring.EXTEND;
//                    }
//                    break;
//
//                case LANDER:
//                    if(button) {
//                        vSlide();
//                        swerveSleep(3000);
//                        curScoreState = autoScoring.DRIVE_FORWARD;
//                    }
//                    else if(back) {
//                        curScoreState = autoScoring.INTAKE;
//                    }
//                    break;
//            }
//        }
//    }
}