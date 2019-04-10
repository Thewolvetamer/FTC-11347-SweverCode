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
    //    int wristSpeed;
    private int minToggle = 0;
    private boolean toggleVar = false;
    private boolean toggle2 = false;
    double togglePos;

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
        // Initialize base classes.
        // All via self-construction.

        // Initialize class members.
        // All via self-construction.
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

        // start without using gradual drive changes
        ourSwerve.setUseGradual(Boolean.FALSE);

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
        if (gamepad1.dpad_down) {
            ourSwerve.setSwerveMode(SwerveDrive.swerveModes.SWERVE_DEMO);
        }

        // Move the robot, flipping y since the joysticks are upside down
        ourSwerve.driveRobot(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, gamepad1.right_stick_y);

        // *** use buttons to trigger other actions ***

        strafeR();

        strafel();

        dropTeamIcon();

        climb();

        wrist();

        hSlide();

        vSlide();

        ourSwerve.distance(height.getDistance(DistanceUnit.CM));


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

    private void strafel() {

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
        else if(height.getDistance(DistanceUnit.CM) < 100 && gamepad2.dpad_left) {
            climber.setPower(1);
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
            swerveSleep(1000);
            dump.setPosition(0);
            vSlide.setTargetPosition(0);
            vSlide.setPower(-1);
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


//    public void autoScore(boolean button) {
//        if(curSwerveMode == swerveModes.SWERVE_AUTO) {
//            switch(curScoreState) {
//                case DRIVE_FORWARD:
//                    if(button) {
//
//                    }
//            }
//        }
//    }

}
//telemetry.update()


