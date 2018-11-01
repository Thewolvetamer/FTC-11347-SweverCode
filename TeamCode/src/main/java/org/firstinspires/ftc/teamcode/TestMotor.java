// ***********************************************************************
// TestMotor
// ***********************************************************************
// Manages the testing of a single motor on a robot

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TestMotor {
    private DcMotor motor;
    private String mName;

    // Create a version of a motor, tracking the motor to use and the name
    public TestMotor(DcMotor useMotor, String useName ){
        this.motor  = useMotor;
        this.mName = useName;

        // stop any movement
        stopMoving();
    }

    public String getName() {
        return mName;
    }

    public void setSpeed( double newSpeed ){
        motor.setPower( newSpeed );
    }

    // give a general way to stop movement
    public void stopMoving() {
        setSpeed( 0 );
    }
}
