// ***********************************************************************
// SwerveAutoGold
// ***********************************************************************
// The autonomous mode for swerve operations for Red team position 1

//
// This is just a setup for the general autonomous code.
//

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// ***********************************************************************
// Definitions from Qualcomm code for OpMode recognition
// ***********************************************************************
@Autonomous(name="Swerve: 2-AutoGold 1.1ALT", group="Swerve")
//@Disabled
public class SwerveAutoGold2 extends SwerveAuto {

    // ***********************************************************************
    // SwerveAutoGold
    // ***********************************************************************
    // Constructs the class.
    // The system calls this member when the class is instantiated.
    public SwerveAutoGold2() {
        // Initialize base classes.
        // All via self-construction.

        // Initialize class members.
        // All via self-construction.
    }

    // ***********************************************************************
    // Init
    // ***********************************************************************
    // Set needed values for Red alliance position 1
    @Override
    public void init() {
        swerveDebug(500, "SwerveAutoGold::init", "STARTing init for Gold");

        // Run initialization of other parts of the class
        // Note that the class will connect to all of our motors and servos
        super.init();
        start90= Boolean.TRUE;
        targetSilver = Boolean.FALSE;

        // Robot and autonomous settings are read in from files in the core class init()
        // Report the autonomous settings
        showAutonomousGoals();

        swerveDebug(500, "SwerveAutoGold::init", "DONE");
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