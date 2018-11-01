// ***********************************************************************
// SwerveWheel
// ***********************************************************************
// Controls for one of four wheels on a swerve drive robot.
// Each wheel is a single unit that has both a motor (speed) and a servo (position).
//
// Speed is 0 to 1 (never negative). Motor configuration needs to have positive as moving forward.
//
// Servo postion is -1 to 1. Servo configuration needs to be set so that the motion range is
// from -180 to 180, with 0 as straight.
//
// Because we are building a servo-based robot, we chose to implement the code below using a
// range of -90 to 90. We flip the sign of the speed when we adapt from values outside this
// target range.
//
// *** DERIVED FROM ***
// GREAT data on swerve drive design found here: https://www.chiefdelphi.com/media/papers/2426
// Posted by Ether starting in 2011. The simple calculations were clear. The spreadsheet model
// was an amazing help for visualizing what we were building before driving it.
//
// We also found some useful code ideas posted on GitHub by FTC team 1251 (references to
// team1251.org not working as of April, 2018)
// The code was downloaded from https://github.com/bob80333/swerve-drive
// That code was created by Eric Engelhart on 3/20/2017

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// ***********************************************************************
// SwerveWheel
// ***********************************************************************
// Class definitions
public class SwerveWheel {
    DcMotor motor;
    private Servo servo;

    // ***********************************************************************
    // SwerveWheel - create a new swerve wheel unit
    // ***********************************************************************
    // Creates a new instance of a wheel, using the motor and servo provided.
    public SwerveWheel( DcMotor useMotor, Servo useServo ){
        this.motor  = useMotor;
        this.servo = useServo;

        // stop any movement
        updateWheel( 0, 0);
    }

    public void updateWheel(double newSpeed, double newPosition){
        // be sure we have a valid new position
        if(( newPosition < -1.0 ) || ( newPosition > 1.0 )) {
            newPosition = 0;
        }
        // be sure we have a valid and useful speed
        if ( newSpeed < 0 ) {
            newSpeed = 0;
        }
        // if below -90, flip 180 degrees and reverse the speed
        if ( newPosition < -0.5 ) {
            newSpeed = -newSpeed;
            newPosition = newPosition + 1.0;
        }
        // if above 90, flip 180 degrees and reverse the speed
        if ( newPosition > 0.5 ) {
            newSpeed = -newSpeed;
            newPosition = newPosition - 1.0;
        }

        // now set the motor power
        motor.setPower( newSpeed );
        // if the wheel is moving, use the servo to set the position
        if (( newSpeed != 0 )) {
            // setPosition wants 0 to 1, we are at -0.5 to +0.5, so we scale here
            servo.setPosition( newPosition + 0.5 );
        }
    }
}
