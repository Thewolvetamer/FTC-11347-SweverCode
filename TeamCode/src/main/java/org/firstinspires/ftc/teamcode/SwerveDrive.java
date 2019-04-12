// ***********************************************************************
// SwerveDrive
// ***********************************************************************
// Controls a 4 wheel swerve drive on a robot.
//
// This drive code works with the IMU in the Rev Robotics expansion hub. The IMU provides a heading
// angle relative to magnetic north. Driving modes are:
//      SWERVE_DRIVER
//          The driver does all the work.
//          Robot left/right and forward/back are based on the robot position.
//          Robot turn left/right rotate the robot, while turn forward/back are not used.
//      SWERVE_DRIVE_TURN
//          Driving based on heading but not rotation.
//          Robot left/right and forward/back are derived with the heading.
//          Robot turn left/right rotate the robot, which turn forward/backward are not used.
//      SWERVE_DRIVE_ORIENT
//          Driving based on rotation but not heading.
//          Robot left/right and forward/back are based on the robot position.
//          Robot turn left/right and forward/backward are relative to field position.
//      SWERVE_AUTO
//          Driving is based on heading for direction and rotation.
//          Robot left/right and forward/back are derived with the heading.
//          Robot turn left/right and forward/backward are relative to field position.
//
// It is expected that autonomous action will help the driver to move the robot more easily.
// To use the heading automation, the init code needs to pass in a reference heading. This is the
// heading of the robot away from the wall where it starts the match. All movement will be
// adjusted based on the orientation relative to that heading (as desired 'north'/away.
//
// Wheel order is counterclockwise:
//     Wheel 1 is rightFront
//     Wheel 2 is leftFront
//     Wheel 3 is leftBack
//     Wheel 4 is rightBack
//
// *** DERIVED FROM ***
// GREAT data on swerve drive design found here: https://www.chiefdelphi.com/media/papers/2426
// Posted by Ether starting in 2011. The simple calculations were clear. The spreadsheet model
// was an amazing help for visualizing what we were building before driving it.
//
// We also found some useful code ideas posted on GitHub by FTC team 1251 (references to
// team1251.org not working as of April, 2018)
// Part of the swerve drive code from https://github.com/bob80333/swerve-drive
// That code was created by Eric Engelhart on 3/20/2017


// TODO: add separate task to manage the wheels to use the gradual speed (and position) change

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.FastMath;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.FileInputStream;

import java.io.FileOutputStream;

// ***********************************************************************
// SwerveDrive
// ***********************************************************************
// Class definitions.

public class SwerveDrive {

    // drive operation mode
    enum swerveModes {
        SWERVE_DRIVER,
        SWERVE_DRIVE_TURN,
        SWERVE_DRIVE_ORIENT,
        SWERVE_AUTO,
        SWERVE_DEMO
    }
    // mode we are operating in
    public swerveModes curSwerveMode;

    // Our 4 swerve drive wheels (servo and motor)
    private SwerveWheel[] swerveWheels;
    // Position for each drive wheel servo
    private double[] positions;
    // Speeds for each drive wheel motor
    private double[] speeds;
    // Max speed is useful for several reasons
    private double maxSpeed;


    // auto drive speed
    private double autoSpeed;
    private double autoAngle;
    private double autoOrient;
    private double autoDistance;
    private Boolean autoDone;
    private double autoWheelMove[];
    private double autoWheelLast[];
    private double deltaGravity=0;
    //Breaking test
    private double decrementSpeed;

    // distance is in cm, we have 288 encoder ticks per rotation and a 6 inch wheel diameter
    // encoder ticks per cm of lateral distance
    private double wheelcm2encoder = ( 125 / ( 6 * 2.54 * Math.PI ) );


    // ratio value between length and width for the drive base to adjust turns
    private double frontRatio;
    private double sideRatio;

    // logging data available for use
    private String modeLog;
    private String moveLog;
    private String moveAdjustLog;
    private String speedLog;
    private String angleLog;
    private String orientationLog;
    private String autoDriveLog;
    private String heightLog;

    // IMU for heading when using movement automation
    private BNO055IMU imu;
    // and the angles from that IMU
    private Orientation angles;
    private Acceleration gravAngles;


    // TODO: BE SURE we are using degrees for the angle, or change the file read/write code
    // saved base orientation of the robot from the first wall
    private double baseOrientationAngle;
    // file to save the base orientation angle in
    private String swerveAngleFile;
    // saved current (adjusted) orientation/heading
    double curHeading;
    // when to run the next orientation
    private double nextOrientationTime;
    // measure time for the next update
    private ElapsedTime swerveTime;
    // how long to wait between changes
    private float minOrientationWait;

    // scale the output based on servo angle capability
    // -- swerve base code plans +/- 90, servo is +/- 135, so scale back 50%
    // ***** TODO **** testing at 1:1 scale for now
//    private double SWERVE_SCALE = ( 0.66667 / Math.PI );
    private double SWERVE_SCALE = ( 1.0 / Math.PI );
    public double DEG2BASE = ( Math.PI / 180 );



    // flag for using gradual motor changes rather than abrupt changes
    private Boolean useGradual;

    // string format for doubles in messages
    private String dblFormat = "%6.3f";


    // ***********************************************************************
    // SwerveDrive - create a new drive based on wheels and IMU
    // ***********************************************************************
    // The 4 swerve drive wheels (motor and servo) are passed in as units.
    // The wheelbase and trackwidth are used to calculate the impact of any rotation angles.
    //  -- These approximate an Ackermann turn around a fixed point on the field
    //  -- The goal is that all wheels move using planned direction/power without slipping.
    //  -- https://en.wikipedia.org/wiki/Ackermann_steering_geometry
    // The inertial management unit (IMU) built into the Rev hub gives magnetic robot orientation.
    public SwerveDrive(SwerveWheel rightFront, SwerveWheel leftFront, SwerveWheel leftBack, SwerveWheel rightBack,
                       double wheelBase, double trackWidth, BNO055IMU newImu ){
        // diagonal length of the drive train
        double driveTrainDiagonal;

        // Set the file for reading/saving the base orientation (robot to field)
        //  Note that the directory matches the other configs for the robot
        swerveAngleFile = AppUtil.FIRST_FOLDER + "SwerveAngle.dat";

        swerveWheels = new SwerveWheel[4];
        speeds = new double[4];
        positions = new double[4];

        // save the wheels for driving
        swerveWheels[0] = rightFront;
        swerveWheels[1] = leftFront;
        swerveWheels[2] = leftBack;
        swerveWheels[3] = rightBack;

        // prepare a timer
        swerveTime = new ElapsedTime();
        swerveTime.reset();
        // any movement needs to reorient
        nextOrientationTime = 0;
        // wait at least 30 milliseconds before adjusting settings
        minOrientationWait = 30;

        // Read the saved robot to field orientation value
        swerveReadAngle();

        // build relative movement for turning
        driveTrainDiagonal = FastMath.sqrt( wheelBase*wheelBase + trackWidth*trackWidth );
        frontRatio = wheelBase / driveTrainDiagonal;
        sideRatio = trackWidth / driveTrainDiagonal;

        // save the IMU for orientation
        this.imu = newImu;


        // default reporting is none

        modeLog = "(none)";
        moveLog = "(none)";
        moveAdjustLog = "(none)";
        speedLog = "(none)";
        angleLog = "(none)";
        orientationLog = "(none)";
        heightLog = "(none)";



        // space for remembering wheel distances
        autoWheelMove = new double[ 5 ];
        autoWheelLast = new double[ 5 ];
        autoDone = Boolean.TRUE;


        // do not use gradual changes for wheels and servos
        useGradual = false;
        // set default swerve mode
        curSwerveMode = swerveModes.SWERVE_AUTO;
    }

    // ***********************************************************************
    // swerveModeName - return name of swerve mode given
    // ***********************************************************************
    public String swerveModeName( swerveModes curMode ) {
        switch ( curMode ) {
            case SWERVE_DRIVER:
                return "SWERVE_DRIVE";
            case SWERVE_DRIVE_TURN:
                return "SWERVE_DRIVE_TURN";
            case SWERVE_DRIVE_ORIENT:
                return "SWERVE_DRIVE_ORIENT";
            case SWERVE_AUTO:
                return "SWERVE_AUTO";
            case SWERVE_DEMO:
                return  "SWERVE_DEMO";
            default:
                return "UNKNOWN swerve mode";
        }
     }

    // ***********************************************************************
    // make it easy to read our telemetry strings
    // ***********************************************************************
    public String getModeLog() {
        return modeLog;
    }
    public String getMoveLog() {
        return moveLog;
    }
    public String getMoveAdjustLog() {
        return moveAdjustLog;
    }
    public String getSpeedLog() {
        return speedLog;
    }
    public String getAngleLog() {
        return angleLog;
    }
    public String getOrientLog() { return orientationLog; }
    public String getAutoDriveLog() { return autoDriveLog; }
    public String getHeightLog() { return heightLog; }

    // ***********************************************************************
    // setSwerveMode - update the base robot orientation
    // ***********************************************************************
    public void setSwerveMode( swerveModes newMode ) {
        // record what was changed
        modeLog = "Swerve Mode: is " + swerveModeName( newMode );

        curSwerveMode = newMode;
    }

    // ***********************************************************************
    // setFieldOrientation - update the base robot orientation
    // ***********************************************************************
    public void setFieldOrientation() {
        // clear the old orientation
        baseOrientationAngle = 0;

        // read the current orientation
        checkOrientation();

        // save the orientation that was found
        baseOrientationAngle = curHeading;

        // be sure that the relative heading is left at zero
        curHeading = 0;

        // and make this last across restarts until it is redone
        swerveWriteAngle();
    }

    // ***********************************************************************
    // setUseGradual - control use of gradual or immediate wheel changes
    // ***********************************************************************
    public void setUseGradual( Boolean newGradual) {
        useGradual = newGradual;
    }

    // ***********************************************************************
    // drive - move the robot
    // ***********************************************************************
    // Move X and Y set the direction and speed of the robot overall.
    // Turn X and Y set the rotation/orientation of the robot.
    // Both may be modified by the orientation read from the IMU.
    public void driveRobot( double moveX, double moveY, double turnX, double turnY ) {
        int wheel;          // wheel being updated
        double angle;       // angle for turning

        // only adjust driving every so often
        if ( nextOrientationTime > swerveTime.milliseconds()) {
    /*  no messages of waiting for now
            swerveLog( "driveRobot:: waiting for "
                    + String.format( ".0f", swerveTime.milliseconds())
                    + " to reach "
                    + String.format( ".0f", nextOrientationTime );
            return;
    */
        }
        // note the next time we will adjust the driving
        nextOrientationTime = minOrientationWait + swerveTime.milliseconds();

        // check the orientation of the robot - for field-oriented driving
        checkOrientation();

        // note drive directions
        moveLog = "MoveXY: " + String.format( dblFormat, moveX ) + ", "
                + String.format( dblFormat, moveY )
                + "  TurnXY: " + String.format( dblFormat, turnX ) + ", "
                + String.format( dblFormat, turnY );

        // if driving with field orietation automation, adjust for the robot orientation
        if (( curSwerveMode == swerveModes.SWERVE_DRIVER ) || ( curSwerveMode == swerveModes.SWERVE_DRIVE_ORIENT || ( curSwerveMode == swerveModes.SWERVE_DRIVE_TURN ) )) {

            // shift the input angles based on robot rotation

            angle = FastMath.atan2( moveY, moveX ) - curHeading * DEG2BASE;
            angle = angle * DEG2BASE;

            moveAdjustLog = "Move Adj: "
                    + String.format( dblFormat, curHeading )
                    + " ( " + String.format( dblFormat, angle ) + " ) ";

        } else {
            moveAdjustLog = "Move Adj: (none)";
        }




        // TODO: write code to adjust for automatic robot spin/orientation
        /*
        // MESSAGES for drive mode, robot base orientation and orientation to field
        double angle = rotation.getDirectionRadians();
        if (fieldOrientedDrive) {
            angle -= FastMath.toRadians(normalizeGyroAngle(gyro.getAngle()));
        }
        */



        // calculate the wheel moves needed
        calculateWheels( moveX, moveY, turnX * Math.PI );

        // update wheels
        for (wheel = 0; wheel < swerveWheels.length; wheel++){
            // if the fastest wheel is almost stopped, just stop the robot
            if ( maxSpeed < 0.05 ) {
                swerveWheels[wheel].updateWheel( 0, positions[wheel] );

            // otherwise, set the target speed and position
            } else {
                swerveWheels[wheel].updateWheel( speeds[wheel], positions[wheel] );
            }
        }
    }


    // ***********************************************************************
    // headless drive - move the robot in relation to the driver
    // ***********************************************************************
    // adjusts the direction of the robot to drive in relation to the driver
//    public void headlessDriveRobot( double moveX, double moveY, double turnX, double turnY, boolean crater ) {
//        int wheel;          // wheel being updated
//        double angle;       // angle for turning
//        double baseHeadlessAngle;
//
//
//        // check the orientation of the robot - for field-oriented driving
//        checkOrientation();
//
//        // note drive directions
//        moveLog = "MoveXY: " + String.format( dblFormat, moveX ) + ", "
//                + String.format( dblFormat, moveY )
//                + "  TurnXY: " + String.format( dblFormat, turnX ) + ", "
//                + String.format( dblFormat, turnY );
//
//
//
//        if (curSwerveMode == swerveModes.SWERVE_DRIVE_ORIENT || curSwerveMode == swerveModes.SWERVE_DEMO) {
//            // the angle for diver oriented driving changes based
//            // upon the starting orientation
//
//            // when we do Demos the robot starts facing the same direction as the drivers
//            // so the base angle should be 0
//            if (curSwerveMode == swerveModes.SWERVE_DEMO) {
//                baseHeadlessAngle = 0;
//            }
//            // on crater side the robot is -135( 3pi / 4 ) from the driver's orientation
//            else if(crater && !(curSwerveMode == swerveModes.SWERVE_DEMO)) {
//                baseHeadlessAngle = baseOrientationAngle + ((3 * Math.PI) / 4);
//            }
//            // on depot side the robot is 135( 3pi / 4 ) from the driver's orientation
//            else {
//                baseHeadlessAngle = baseOrientationAngle - ((3 * Math.PI) / 4);
//            }
//            angle = FastMath.atan2( moveY, moveX ) - baseHeadlessAngle;
//            angle = angle * DEG2BASE;
//
//            moveAdjustLog = "Move Adj: "
//                    + String.format( dblFormat, curHeading )
//                    + " ( " + String.format( dblFormat, angle ) + " ) "
//            ;
//
//        } else {
//            moveAdjustLog = "Move Adj: (none)";
//        }
//
//        // calculate the wheel moves needed
//        calculateWheels( moveX, moveY, turnX * Math.PI );
//
//        // update wheels
//        for (wheel = 0; wheel < swerveWheels.length; wheel++){
//            // if the fastest wheel is almost stopped, just stop the robot
//            if ( maxSpeed < 0.05 ) {
//                swerveWheels[wheel].updateWheel( 0, positions[wheel] );
//
//                // otherwise, set the target speed and position
//            } else {
//                swerveWheels[wheel].updateWheel( speeds[wheel], positions[wheel] );
//            }
//        }
//    }



    boolean isRobotLevel() {

        gravAngles = imu.getGravity();
        if (Math.abs(gravAngles.zAccel) < .30+deltaGravity && Math.abs(gravAngles.yAccel) < .30+deltaGravity) {
         return true;
        }
        else{
            return false;
        }

    }
    // ***********************************************************************
    // getGravXYZAccel() gets x,y and z accel the imu senses.
    // ***********************************************************************

    String getGravXYZAccel(){
        gravAngles=imu.getGravity();
        return("X: "+ gravAngles.xAccel+" Y: "+gravAngles.yAccel+" Z: "+gravAngles.zAccel);
    }

    // ***********************************************************************
    // checkOrientation - gather the current orientation data
    // ***********************************************************************
    void checkOrientation() {
        // read the orientation of the robot
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        // and save the heading
        curHeading = - ( angles.firstAngle - baseOrientationAngle );

        // log the orientation
        orientationLog = "Orient: "
                + String.format( dblFormat, curHeading )
                + "  Base: " + String.format( dblFormat, baseOrientationAngle );
    }

    // ***********************************************************************
    // calculateWheels - calculate the speed and angle needed for each wheel
    // ***********************************************************************
    private void calculateWheels( double movementX, double movementY, double rotationRad ){
        int nextSpeed;

        // calculate x/y components for wheels 1 & 3 as thats all that's needed
        double frontX = movementX - ( rotationRad * frontRatio );
        double backX = movementX + ( rotationRad * frontRatio );
        double rightY = movementY + ( rotationRad * sideRatio );
        double leftY = movementY - ( rotationRad * sideRatio );

        // calculate speed/rotation for each wheel
        // TODO - why did wheels flip in new orientation?
        // --- right front wheel
        speeds[1]    = FastMath.sqrt( FastMath.pow2( frontX ) + FastMath.pow2( rightY ));
        positions[1] = FastMath.atan2( frontX, rightY ) * SWERVE_SCALE;
        // --- left front wheel
        speeds[0]    = FastMath.sqrt( FastMath.pow2( frontX ) + FastMath.pow2( leftY ));
        positions[0] = FastMath.atan2( frontX, leftY ) * SWERVE_SCALE;
        // --- right rear wheel
        speeds[3]    = FastMath.sqrt( FastMath.pow2( backX ) + FastMath.pow2( leftY ));
        positions[3] = FastMath.atan2( backX, leftY ) * SWERVE_SCALE;
        // --- left rear wheel
        speeds[2]    = FastMath.sqrt( FastMath.pow2( backX ) + FastMath.pow2( rightY ));
        positions[2] = FastMath.atan2( backX, rightY ) * SWERVE_SCALE;

        // normalize speeds - always <= 1
        maxSpeed = 0;
        for ( nextSpeed = 0; nextSpeed < 4; nextSpeed++){
            if (speeds[ nextSpeed ] > maxSpeed){
                maxSpeed = speeds[ nextSpeed ];
            }
        }
        // if max is > 1, scale all speeds back
        if ( maxSpeed > 1 ){
            for ( nextSpeed = 0; nextSpeed < 4; nextSpeed++){
                speeds[ nextSpeed ] /= maxSpeed;
            }
            // and normalize the max
            maxSpeed = 1;
        }

        speedLog = "Speed F L/R: " + String.format( dblFormat, speeds[1]) + " / "
                + String.format( dblFormat, speeds[0])
                + "  R L/R: " + String.format( dblFormat, speeds[3]) + ", "
                + String.format( dblFormat, speeds[2]);
        angleLog = "Angle F L/R: " + String.format( dblFormat, positions[1]) + " / "
                + String.format( dblFormat, positions[0])
                + "  R L/R: " + String.format( dblFormat, positions[3]) + ", "
                + String.format( dblFormat, positions[2]);
    }

    // ***********************************************************************
    // ***********************************************************************
    public double normalizeGyroAngle360(double angle){
        double angle180;

        // get our angle to 0-360
        angle180 = angle - (FastMath.floor( angle / 360.0) * 360.0);

        // now adjust to be -180 to 180
        if ( angle180 > 180.0 ) {
            angle180 -= 360.0;
        }

        return ( angle180 );
    }

    // ***********************************************************************
    // ***********************************************************************
//    public double normalizeGyroAngle180(double angle){
//        return (angle - (FastMath.floor( angle / 180) * 180) );
//    }
    // ***********************************************************************
    // stopRobot
    // ***********************************************************************
    // stop all motion
    void stopRobot() {
        driveRobot( 0.0, 0.0, 0.0, 0.0 );
    }

    // ***********************************************************************
    // autoDrive
    // ***********************************************************************
    // move the robot at target speed with wheels at target angle (relative to base)
    // gradually orient the robot top to match the orientation given
    // go until target distance (in cm) is reached
    void autoDrive( double aSpeed, double aAngle, double aOrient, double aDist ) {
        int w;

        autoSpeed = aSpeed;
        autoAngle = aAngle;
        autoOrient = aOrient;
        autoDistance = aDist * wheelcm2encoder;
        autoDone = Boolean.FALSE;

        // clear out the move counters
        for ( w = 0; w < 3; w++ ) {
            autoWheelLast[ w ] = swerveWheels[ w ].motor.getCurrentPosition();
            autoWheelMove[ w ] = 0;
        }

        // update movement and check for done
        autoDriveCheck( Boolean.FALSE );
    }

    // ***********************************************************************
    // autoDriveCheck
    // ***********************************************************************
    // check for done on auto drive
    // if not done, update target movements from auto drive
    boolean autoDriveCheck( boolean forceStop ) {
        int w;
        double wNext;
        double rDist;
        double tAngle;
        double mAngle;
        double turnSpd;
        double moveX;
        double moveY;

        if ( autoDone ) {
            if ( forceStop ) {
                stopRobot();
            }

            return( Boolean.TRUE );
        }

        // no distance so far;
        rDist = 0;

        for ( w = 0; w < 2; w++ ) {
            wNext = swerveWheels[ w ].motor.getCurrentPosition();
            autoWheelMove[ w ] += Math.abs( wNext - autoWheelLast[ w ]);
            autoWheelLast[ w ] = wNext;
            rDist += autoWheelMove[ w ];
        }

        // check for move done
        if (( rDist / 4 ) > autoDistance ) {
            autoDone = Boolean.TRUE;

            if ( forceStop ) {
                stopRobot();
            }
            return( Boolean.TRUE );
        }

        // know our angles
        checkOrientation();

        // move based on angles
        // - get wheel target in degrees
        tAngle = autoAngle - curHeading + baseOrientationAngle - 90;
        // - normalize and convert to radians
        mAngle = normalizeGyroAngle360( tAngle ) * DEG2BASE;

        moveX = FastMath.sin( mAngle ) * autoSpeed;
        moveY = FastMath.cos( mAngle ) * autoSpeed;

        autoDriveLog = "target " + String.format( dblFormat, tAngle) + " ( "
                + String.format( dblFormat, autoAngle) + " ), radians "
                + String.format( dblFormat, mAngle);

        // adjust heading based on orientation target
        if (Math.abs(autoOrient - curHeading) > 60.0) {
            turnSpd = autoSpeed * 0.1;
        } else if (Math.abs(autoOrient - curHeading) > 20.0) {
            turnSpd = autoSpeed * .05;
        } else {
            turnSpd = autoSpeed * .01;
        }
        if (autoOrient > curHeading) {
            turnSpd = -turnSpd;
        }

        // TODO: fix auto orient


        // DEBUG - stop auto orient for now
        //turnSpd = 0;

        // move the robot
        driveRobot( moveX, moveY, turnSpd, 0.0);

        return( Boolean.FALSE );
    }


    //Simply hypothetical concept for "breaking" code created *11/01/2018 08:45*
 //   void autoBreak( double decSpeed ) {

 //      decrementSpeed = decSpeed;
//
//    for(int autoS > decrementSpeed; autoSpeed = 0; autoSpeed--){
//
//        }
//
//
//    }


    // ***********************************************************************
    // swerveRadAngle - read robot to field angle from save file
    // ***********************************************************************
    private int swerveReadAngle() {
        FileInputStream myFile;
        int myValue;                // Becomes a single BYTE that is read....

        try {
            myFile = new FileInputStream(swerveAngleFile);

            // Read the saved angle to field (half, to fit in one byte)
            myValue = myFile.read();
            baseOrientationAngle = myValue * 2.0;

            myFile.close();

        } catch (Exception e) {
            // nothing much to do....
            return -1;
        }

        return 0;
    }

    // ***********************************************************************
    // swerveWriteAngle - write robot to field angle to save file
    // ***********************************************************************
    private int swerveWriteAngle() {
        FileOutputStream myFile;
        int myValue;                // Becomes a single BYTE to write....

        try {
            myFile = new FileOutputStream(swerveAngleFile,false);

            // save half of the real angle - to fit in one byte
            myValue = (int)( baseOrientationAngle / 2 );
            myFile.write(myValue);

            // Close out the file
            myValue = '\n';
            myFile.write(myValue);

            myFile.close();
        } catch (Exception e) {
            // nothing much to do....
            return -1;
        }

        return 0;
    }

    void distance(double distL, double distR) {
        heightLog = "My height is " + (distR - 18.6) + " " + (distL - 18.6);
    }

}
