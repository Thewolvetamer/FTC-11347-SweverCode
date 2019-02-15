// ***********************************************************************
// SwerveTeleOp
// ***********************************************************************
// The tele-op mode for swerve robot operations

package org.firstinspires.ftc.teamcode;

// ***********************************************************************
// Definitions from Qualcomm code for OpMode recognition
// ***********************************************************************

public class SwerveTeleOpDepot extends SwerveCore {
    // Note when we are approaching the end of the game
    Boolean inEndGame;
    //    int wristSpeed;
    private int minToggle = 0;
    private boolean toggleVar=false;
    private boolean toggle2 = false;
    double togglePos;
    boolean crater = false;

    // ***********************************************************************
    // SwerveTeleOp
    // ***********************************************************************
    // Constructs the class.
    // The system calls this member when the class is instantiated.
    public SwerveTeleOpDepot() {
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

        flapL.setPosition(1);
        flapR.setPosition(0);

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

        // set swerve drive oritation automation level based on driver request
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
        ourSwerve.headlessDriveRobot(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, gamepad1.right_stick_y, crater);

        // *** use buttons to trigger other actions ***

        strafeR();

        strafel();

        dropTeamIcon();

        liftRobot();

        wrist();

        extend();

        intake();

        flaps();


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
            gameMarkDrop.setPosition(0.4);
        }

        //telemetry.addData("Servo Position", gameMarkDrop.getPosition());
        //telemetry.addData("Status", "Running");
        //telemetry.update();
    }
    private void allFlaps(double position){
        flapR.setPosition(position);
        flapL.setPosition(position);
    }
    private void liftRobot() {
        //Robot climber control

        // x for down, y for up
        if (gamepad2.dpad_down) {
            climber.setPower(-1);
        } else if (gamepad2.dpad_up) {

            climber.setPower(1);
        } else {

            climber.setPower(0);
        }
    }


    private void strafeR() {

        if (gamepad1.dpad_right) {
            swerveLeftFront.updateWheel(1, -0.50);
            swerveRightFront.updateWheel(1, -0.50);
            swerveLeftRear.updateWheel(1, -0.50);
            swerveRightRear.updateWheel(1, -0.50);
        }
    }

    private void strafel() {

        if (gamepad1.dpad_left) {
            swerveLeftFront.updateWheel(1, 0.50);
            swerveRightFront.updateWheel(1, 0.50);
            swerveLeftRear.updateWheel(1, 0.50);
            swerveRightRear.updateWheel(1, 0.50);
        }
    }

    void extend() {
        extension.setPower(gamepad2.right_stick_y);
    }


    void wrist() {
        wrist.setPower(-gamepad2.left_stick_y);
    }

    void intake() {
//        intake
        if (gamepad2.left_bumper) {
            intake.setPower(.6);
        }
//        outtake
        else if (gamepad2.right_bumper) {
            intake.setPower(-1);
        }
//        zero power
        else {
            intake.setPower(0.0);
        }
    }

    void flaps() {
//        How do you even set a toggle button?
        if (gamepad2.a&&!toggleVar) {
            // remmeber button is pressed
            toggleVar=true;

            if ( toggle2 ) {
                // flip to other when button pressed again
                toggle2 = false;
                allFlaps(.5);

                togglePos=.5;
            } else {
                toggle2 = true;
                allFlaps(1);
                togglePos=1;
            }
        }
        if (toggleVar&&!gamepad2.a){
            // button released
            toggleVar = false;
        }






        if(gamepad2.right_trigger >= 0.2) {
            allFlaps(-1);
        }
        else {

            allFlaps(togglePos);
        }


//        } else if(gamepad2.left_trigger >= 0.2) {
//            flapR.setPosition(.5);
//            flapL.setPosition(.5);
//        } else {
//            flapR.setPosition(1);
//            flapL.setPosition(1);
    }
}


//telemetry.update()


