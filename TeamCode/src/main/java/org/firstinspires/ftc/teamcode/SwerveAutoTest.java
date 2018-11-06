// ***********************************************************************
// SwerveAutoTEST
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
@Autonomous(name="Swerve: 9-AutoTEST 1.0", group="Swerve")
//@Disabled
public class SwerveAutoTest extends SwerveAuto {

    // ***********************************************************************
    // SwerveAutoTEST
    // ***********************************************************************
    // Constructs the class.
    // The system calls this member when the class is instantiated.
    public SwerveAutoTest() {
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
        swerveDebug(500, "SwerveAutoTEST::init", "STARTing init for TETS");

        // Run initialization of other parts of the class
        // Note that the class will connect to all of our motors and servos
        super.init();

        targetSilver = Boolean.TRUE;

        // Robot and autonomous settings are read in from files in the core class init()
        // Report the autonomous settings
        showAutonomousGoals();

        swerveLog( "X S6", ourSwerve.getOrientLog());

        swerveDebug(500, "SwerveAutoTEST::init", "DONE");
    }


    // ***********************************************************************
    // start
    // ***********************************************************************
    // Do first actions when the start command is given.
    // Called once when the OpMode is started.
    @Override
    public void start() {
        swerveDebug(500, "SwerveAutoTEST::start", "START");

        // Call the super/base class start method.
        super.start();


        // *************************************************
        // *************************************************
        // ****** set debugging on or off and options ******
        // *************************************************
        // *************************************************
        debugActive = Boolean.TRUE;
        // always add one for the state set in init below
        debugStates = 4 + 1;
        debugStartState = autoStates.SWERVE_TEST_MOVE2_ROBOT;
        // *************************************************
        // *************************************************
        

        swerveDebug( 500, "SwerveAutoTEST::start", "DONE");
    }
}