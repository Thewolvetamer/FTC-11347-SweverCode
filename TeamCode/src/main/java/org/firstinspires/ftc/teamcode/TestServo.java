// ***********************************************************************
// TestServo
// ***********************************************************************
// Manages the testing of a single servo on a robot

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class TestServo {
    private Servo servo;
    private String sName;
    private Boolean isCR;
    private double targetValue;

    // Create a version of a motor, tracking the motor to use and the name
    public TestServo( Servo useServo, Boolean isContinuousRotation, String useName ){
        this.servo  = useServo;
        this.sName = useName;
        this.isCR = isContinuousRotation;

        // stop any movement
        stopMoving();
    }

    public String getName() {
        return sName;
    }

    public double getTarget() {
        return targetValue;
    }

    // set the servo speed/position
    public void setPosition( double newPosition ){
        double  workPosition;

        targetValue = newPosition;

        // set the servo based on a scaled position
        // -- incoming data is -1 to 1, shift to 0 to 1
        workPosition = ( newPosition / 2.0 ) + 0.5;
        if ( workPosition < 0.0 ) {
            workPosition = 0;
        } else if ( workPosition > 1.0 ) {
            workPosition = 1.0;
        }
        servo.setPosition( workPosition );
    }

    // give a general way to stop movement
    public void stopMoving() {
        // stop any movement
        // ... for non-CR servos, we do NOTHING
        if ( isCR ) {
            this.setPosition( 0 );
        }
    }
}
