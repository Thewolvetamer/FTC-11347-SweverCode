// ***********************************************************************
// SwerveAutoSet
// ***********************************************************************
// The autonomous setup code for swerve operations

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// ***********************************************************************
// Definitions from Qualcomm code for OpMode recognition
// ***********************************************************************
@Autonomous(name="Swerve: 9-Auto SETUP 0.4", group="Swerve")
//@Disabled

public class SwerveAutoSet extends SwerveCore {

    // ***********************************************************************
    // SwerveAuto
    // ***********************************************************************
    // Constructs the class.
    // The system calls this member when the class is instantiated.
    public SwerveAutoSet()
    {
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
    @Override public void init ()
    {
        swerveDebug(500, "SwerveAutoSet::init", "START");

        // Run initialization of other parts of the class
        // Note that the class will connect to all of our motors and servos
        super.init();

        // Robot and autonomous settings are read in from files in the core class init()

        // Report the initial goals
        showAutonomousGoals();

        swerveDebug( 500, "SwerveAutoSet::init", "DONE" );
    }


    // ***********************************************************************
    // start
    // ***********************************************************************
    // Do first actions when the start command is given.
    // Called once when the OpMode is started.
    @Override
    public void start()
    {
        // Call the super/base class start method.
        super.start();

        settingsDone = Boolean.TRUE;
    }


    // ***********************************************************************
    // loop
    // ***********************************************************************
    // Read settings from the controller - our menu system!
    // Displays current settings in the telemetry area of the driver controller screen
    @Override
    public void loop()
    {
        // Use controllers to set values
        readSettings();

        // Report the current settings
        showAutonomousGoals();

        // Wait, so we do not read settings too fast
        swerveSleep( 250 );
    }


    // ***********************************************************************
    // stop
    // ***********************************************************************
    // Shut down the readings and save
    // Called once when the OpMode is stopped
    @Override
    public void stop()
    {
        // Write the autonomous settings
        if ( swerveWriteAutoSettings() < 0 ) {
            // File not written - but no real way to help....
            swerveLog("*ERROR*", "Write of autonomous settings FAILED");
        } else {
            swerveDebug(100, "revTankAutoSet::stop", "Write of autonomous settings *DONE*");
        }

        // Call the super/base class start method.
        super.stop();
    }

    // ***********************************************************************
    // readSettings
    // ***********************************************************************
    // Read joystick buttons to change settings
    //
    // We need to choose the red side or the blue side
    //      x is blue -- b is red
    // We need to choose position 1 or position 3
    //      left joystick:negative x is 1 -- positive x is 3
    // We need to choose to shoot particles or not
    //      a is no shooting -- y means shoot
    // Delay time to wait for partner to move first
    //      left joystick y neg is more -- pos is less (y is flipped)
    // Done changing settings says move on (write to file)
    //      right joystick pressed down
    void readSettings() {
        swerveDebug( 500, "SwerveAutoSet::readSettings", "START");

            autoDelay = 6;

        // Check for red or blue
        if (gamepad1.x) {
            settingsDone = Boolean.FALSE;

            crater = Boolean.FALSE;
        }
        if (gamepad1.b) {
            settingsDone = Boolean.FALSE;

            crater = Boolean.TRUE;
        }

        // Check for waiting for partner
        if (gamepad1.left_stick_y > minJoystickMove) {
            settingsDone = Boolean.FALSE;

            autoDelay += 1;
            if (autoDelay > delayMAX) {
                autoDelay = delayMAX;
            }
        }
        if (gamepad1.left_stick_y < -minJoystickMove) {
            settingsDone = Boolean.FALSE;


        }

        // Check for done changing settings
        if (settingsDone) {

            // Write the autonomous settings
            if ( swerveWriteAutoSettings() < 0 ) {
                // File not written - but no real way to help....
                swerveLog( "*ERROR*", "Write of autonomous settings FAILED" );
            } else {
                swerveDebug( 100, "revTankAutoSet::readSettings", "Write of autonomous settings *DONE*" );
            }

            // Settings have been saved
            settingsDone = Boolean.TRUE;
        }

        swerveDebug( 500, "SwerveAutoSet::readSettings", "DONE" );
    }
}
