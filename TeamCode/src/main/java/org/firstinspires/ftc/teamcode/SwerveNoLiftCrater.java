// We have constant problems with our lift.
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="Swerve: OnGroundCrater", group="Swerve")

public class SwerveNoLiftCrater extends SwerveAuto {

    // ***********************************************************************
    // SwerveAutoGold
    // ***********************************************************************
    // Constructs the class.
    // The system calls this member when the class is instantiated.


    // ***********************************************************************
    // Init
    // ***********************************************************************
    // Set needed values for Red alliance position 1
    @Override
    public void init() {
        swerveDebug(500, "SwerveAutoCrater::init", "STARTing init for Crater");

        // Run initialization of other parts of the class
        // Note that the class will connect to all of our motors and servos
        super.init();
        noLift = true;
        crater = Boolean.TRUE;

        // Robot and autonomous settings are read in from files in the core class init()
        // Report the autonomous settings
        showAutonomousGoals();

        swerveDebug(500, "SwerveAutoCrater::init", "DONE");
    }


    // ***********************************************************************
    // start
    // ***********************************************************************
    // Do first actions when the start command is given.
    // Called once when the OpMode is started.
    @Override
    public void start() {
        swerveDebug(500, "SwerveAutoGold::start", "START");

        // Call the super/base class start method.
        super.start();

        swerveDebug( 500, "SwerveAutoGold::start", "DONE");
    }

}
