// ***********************************************************************
// SwerveTest
// ***********************************************************************
// The test code for swerve operations
//
// This allows us to run a test against each individual motor/servo

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// ***********************************************************************
// Definitions from Qualcomm code for OpMode recognition
// ***********************************************************************
@TeleOp(name="Swerve: 9-TEST 1.0", group="Swerve")

public class SwerveTest extends SwerveCore {

    // Grouping for the values we are testing
    private int     testGroup;
    private static final int TEST_G_MOTORS = 1;
    private static final int TEST_G_SERVOS = 2;

    // which item in the group is being tested
    private int     testItem;
    private int     maxItems;

    // motors and servos we are testing
    private TestMotor motorList[];
    private TestServo servoList[];

    // Telemetry data to display the test information
    private String  testName;
    private String  testValue;


    // ***********************************************************************
    // SwerveTest
    // ***********************************************************************
    // Constructs the class.
    // The system calls this member when the class is instantiated.
    public SwerveTest()
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
        // Run initialization of other parts of the class
        // Note that the class will connect to all of our motors and servos
        super.init();

        // build out the motors to test
        motorList = new TestMotor[4];
        motorList[ 0 ] = new TestMotor( motorLeftFront, "Swerve Motor - Left Front" );
        motorList[ 1 ] = new TestMotor( motorLeftRear, "Swerve Motor - Left Rear" );
        motorList[ 2 ] = new TestMotor( motorRightRear, "Swerve Motor - Right Rear" );
        motorList[ 3 ] = new TestMotor( motorRightFront, "Swerve Motor - Right Front" );

        // build out the servos to test
        servoList = new TestServo[4];
        servoList[ 0 ] = new TestServo( servoLeftFront, Boolean.FALSE, "Swerve Servo - Left Front");
        servoList[ 1 ] = new TestServo( servoLeftRear, Boolean.FALSE, "Swerve Servo - Left Rear");
        servoList[ 2 ] = new TestServo( servoRightRear, Boolean.FALSE, "Swerve Servo - Right Rear");
        servoList[ 3 ] = new TestServo( servoRightFront, Boolean.FALSE, "Swerve Servo - Right Front");
    }


    // ***********************************************************************
    // start
    // ***********************************************************************
    // Do first actions when the start command is given.
    // Called once when the OpMode is started.
    @Override
    public void start()
    {
        // no testing yet
        testGroup = -1;

        // start with motors
        changeTest( TEST_G_MOTORS, 0 );
    }

    // ***********************************************************************
    // loop
    // ***********************************************************************
    // State machine for autonomous robot control
    // Called continuously while OpMode is running
    @Override
    public void loop()
    {
        double      nextValue;              // Next value to target
        int         newItem;                // Nest item to test

        // Give the driver time to change
        swerveSleep( 250 );

        // Check buttons to see which component to test next

        // Press A to test wheels
        if ( gamepad1.a && (testGroup != TEST_G_MOTORS)) {
            changeTest( TEST_G_MOTORS, 0);
        }

        // Press B to test arm components
        if ( gamepad1.b && (testGroup != TEST_G_SERVOS)) {
            changeTest( TEST_G_SERVOS, 0);
        }

        // Use the right stick x values to pick a new target
        if (gamepad1.right_stick_x > minJoystickMove) {
            newItem = testItem + 1;
            if ( newItem > maxItems) {
                newItem = 0;
            }
            // Start the new test
            changeTest( testGroup, newItem );

        } else if (gamepad1.right_stick_x < -minJoystickMove) {
            newItem = testItem - 1;
            if ( newItem < 0 ) {
                newItem = maxItems;
            }
            // Start the new test
            changeTest( testGroup, newItem );
        }

        // Now check for movement for the component
        if (( gamepad1.left_stick_x < minJoystickMove) && ( gamepad1.left_stick_x > -minJoystickMove)) {
            nextValue = 0.0;
        } else {
            nextValue = gamepad1.left_stick_x;
        }

        // Run our current state
        testRun( nextValue );

        // Add telemetry data for the test to the screen
        swerveLog("Name  ", testName);
        swerveLog("Value ", testValue);

        // Any loop background updates happen now....
        loopEndReporting();
    }


    // ***********************************************************************
    // changeTest
    // ***********************************************************************
    // Prepare for a new test (state)
    //
    // This does any initialization that may be needed for a state change
    // In all cases, it sets the test name (target) for telemetry display
    public void changeTest( int newGroup, int newItem )
    {
        // shut down old test item
        testValue = "Power OFF";
        switch ( testGroup ) {
            // Stop motor
            case TEST_G_MOTORS:
                motorList[ testItem ].stopMoving();
                break;

            // Stop servo
            case TEST_G_SERVOS:
                servoList[ testItem ].stopMoving();
                break;
        }

        // more changes for changing groups
        if ( newGroup != testGroup ) {
            switch ( newGroup ) {
                // note motor cout
                case TEST_G_MOTORS:
                    maxItems = motorList.length - 1;
                    break;

                // note servo count
                case TEST_G_SERVOS:
                    maxItems = servoList.length - 1;
                    break;
            }

            // switch to target group
            testGroup = newGroup;
        }

        // switch to target item
        testItem = newItem;

        switch ( testGroup ) {
            // test a motor
            case TEST_G_MOTORS:
                testName = motorList[ testItem ].getName();
                motorList[ testItem ].stopMoving();
                break;

            // test a servoe
            case TEST_G_SERVOS:
                testName = servoList[ testItem ].getName();
                servoList[ testItem ].stopMoving();
                break;

            default:
                testName = "UNKNOWN!";
        }
    }


    // ***********************************************************************
    // testRun
    // ***********************************************************************
    // Make the requested change for the current state
    //
    // Note that the name of the state was already set when the test started
    public void testRun( double myValue )
    {
        // By default, just report the value asked for....
        testValue = "Set to " + swerveNumberFormat.format(myValue);

        switch ( testGroup ) {
            // motors
            case TEST_G_MOTORS:
                motorList[ testItem ].setSpeed( myValue );
                break;

            // Servo settings
            case TEST_G_SERVOS:
                servoList[ testItem ].setPosition( myValue );
                break;
        }
    }
}
