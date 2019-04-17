// ***********************************************************************
// SwerveTeleOp
// ***********************************************************************
// The tele-op mode for swerve robot operations

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import static org.firstinspires.ftc.teamcode.SwerveCore.autoScoring.*;
// ***********************************************************************
// Definitions from Qualcomm code for OpMode recognition
// ***********************************************************************
@TeleOp(name="Swerve: 2-TeleOp 1.1", group="Swerve")

public class SwerveTeleOp extends SwerveCore {
    // Note when we are approaching the end of the game
    private ButtonRebounce buttonToggle=new ButtonRebounce();
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
//        if (gamepad1.a) {
//            ourSwerve.setSwerveMode(SwerveDrive.swerveModes.SWERVE_AUTO);
//        }
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
        ourSwerve.driveRobot(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, gamepad1.right_stick_y);

        // *** use buttons to trigger other actions ***

        strafe();

        climb();

        hSlide();

        dump();

        vSlide();

        intake();

        autoScore();

        clear();

        ourSwerve.distance(heightL.getDistance(DistanceUnit.CM));


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

    // makes it easier to go directly sideways

    private void strafe() {
        if (gamepad1.dpad_left) {
            swerveLeftFront.updateWheel(1, -0.50);
            swerveRightFront.updateWheel(1, -0.50);
            swerveLeftRear.updateWheel(1, -0.50);
            swerveRightRear.updateWheel(1, -0.50);
        }
        else if (gamepad1.dpad_right) {
            swerveLeftFront.updateWheel(1, 0.50);
            swerveRightFront.updateWheel(1, 0.50);
            swerveLeftRear.updateWheel(1, 0.50);
            swerveRightRear.updateWheel(1, 0.50);
        }
    }
    private void dump(){
        if(gamepad2.y){
            dump.setPosition(-1);
        }
        else{
            dump.setPosition(0);
        }
    }
    private void climb() {
        if(gamepad1.right_bumper) {
            climber.setTargetPosition(4500);
            climber.setPower(-1);
            if(gamepad1.start) {
                climber.setTargetPosition(7250);
                climber.setPower(1);
            }
            else if(climber.getCurrentPosition() == climber.getTargetPosition()) {
//                double check height
                while(heightL.getDistance(DistanceUnit.CM) < 30) {
                    climber.setPower(-1);
                }
            }
        }
        else if(gamepad1.dpad_down) {
            climber.setPower(-.7);
        }
        else if(gamepad1.dpad_up) {
            climber.setPower(.7);
        }
        else {
            climber.setPower(0);
        }
    }

    private void vSlide() {
        if(ourSwerve.curSwerveMode == SwerveDrive.swerveModes.SWERVE_AUTO) {
            if(gamepad2.x) {
                double time = getRuntime();
                vSlide.setTargetPosition(10);
                vSlide.setPower(-1);
            }
            else if(vSlide.getCurrentPosition() == vSlide.getTargetPosition() && getRuntime() - time < 750) {
                wristL.setPosition(-1);
                wristR.setPosition(-1);
            }
            else if(wristR.getPosition() == -1) {
                //            distance/circumference of spool   * tpr
                vSlide.setTargetPosition(3500);
                vSlide.setPower(1);
                wristR.setPosition(-.9);
                wristL.setPosition(-.9);
            }
            else if(vSlide.getTargetPosition() == vSlide.getCurrentPosition() && getRuntime()- time > 750) {
                dump.setPosition(1);
                final double t = getRuntime();
                if (getRuntime() == t + 1000) {
                    dump.setPosition(0);
                    vSlide.setTargetPosition(0);
                    vSlide.setPower(-1);
                }
            } else {
                vSlide.setPower(0);
            }
        }
        else {
            vSlide.setPower(-gamepad2.right_stick_y);
        }
    }

    private void hSlide() {
        hSlide.setPower(-gamepad2.left_stick_y);
    }

    private void intake() {
        if (gamepad2.right_trigger > .2) {
            wristR.setPosition(1);
            wristL.setPosition(1);
            intake.setPower(1);
        } else if (gamepad2.left_trigger > .2) {
            wristL.setPosition(-.8);
            wristR.setPosition(-.8);
            intake.setPower(1);

        } else {
            wristL.setPosition(-.8);
            wristR.setPosition(-.8);
            intake.setPower(0);
        }
    }
    private void clear(){
        if (gamepad2.back) {
            vSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            vSlide.setTargetPosition(2175);
            vSlide.setPower(1);
        }
    }

    public void autoScore() {
            hSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (ourSwerve.curSwerveMode == SwerveDrive.swerveModes.SWERVE_AUTO) {
                switch (curScoreState) {

                    case DRIVE_FORWARD:
                        if (buttonToggle.status(gamepad2.a) == ButtonRebounce.Status.COMPLETE) {
                            curScoreState = EXTEND;
                            buttonToggle.reset_status();
                        } else if (buttonToggle.status(gamepad2.b) == ButtonRebounce.Status.COMPLETE) {
                            curScoreState = LANDER;
                            buttonToggle.reset_status();
                        }
                        else {
                            ourSwerve.driveRobot(1, 0, 0, 0);
                        }
                        break;
                    case EXTEND:
                        if (buttonToggle.status(gamepad2.a) == ButtonRebounce.Status.COMPLETE) {
                            curScoreState = INTAKE;
                            hSlide.setPower(0);
                            buttonToggle.reset_status();
                        } else if (buttonToggle.status(gamepad2.b) == ButtonRebounce.Status.COMPLETE) {
                            curScoreState = DRIVE_FORWARD;
                            hSlide.setPower(0);
                            buttonToggle.reset_status();
                        }
                        else {
                            hSlide.setPower(.8);
                        }
                        break;
                    case INTAKE:
                        if(gamepad2.dpad_left) {
                            ourSwerve.driveRobot(0, 0, -1, 0);
                        }
                        else if(gamepad2.dpad_right) {
                            ourSwerve.driveRobot(0, 0, 1, 0);
                        }
                        else if (buttonToggle.status(gamepad2.a) == ButtonRebounce.Status.COMPLETE) {
                            curScoreState = LANDER;
                            intake.setPower(0);
                            buttonToggle.reset_status();
                        } else if (buttonToggle.status(gamepad2.b) == ButtonRebounce.Status.COMPLETE) {
                            curScoreState = EXTEND;
                            intake.setPower(0);
                            buttonToggle.reset_status();
                        }
                        else {
                            intake.setPower(1);
                        }
                        break;
                    case LANDER:
                        hSlide.setTargetPosition(10);
                        hSlide.setPower(-1);
                        if (buttonToggle.status(gamepad2.a) == ButtonRebounce.Status.COMPLETE) {
                            curScoreState = EXTEND;
                            buttonToggle.reset_status();
                        } else if (buttonToggle.status(gamepad2.b) == ButtonRebounce.Status.COMPLETE) {
                            curScoreState = LANDER;
                            buttonToggle.reset_status();
                        }
                        else {
                            ourSwerve.driveRobot(-1, 0, 0, 0);
                        }
                        break;
                    case DUMP:
                        if (buttonToggle.status(gamepad2.a) == ButtonRebounce.Status.COMPLETE) {
                            curScoreState = DRIVE_FORWARD;
                            buttonToggle.reset_status();
                            vSlide();
                            final double t = getRuntime();
                            if(t + 3000 == getRuntime()) {
                                curScoreState = autoScoring.DRIVE_FORWARD;
                            }
                        } else if (buttonToggle.status(gamepad2.b) == ButtonRebounce.Status.COMPLETE) {
                            curScoreState = INTAKE;
                            buttonToggle.reset_status();
                        }
                        break;
                }
            }
        }


}