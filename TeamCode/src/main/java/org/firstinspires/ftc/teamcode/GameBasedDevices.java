package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SwerveTeleOp;
/**
 * Created by Alice on 9/26/2018.
 */
//Note every thing in this class will be use for Non-Swerve related actions outside of SwerveTeleOp and Swerve core
//@TeleOp
public class GameBasedDevices extends SwerveTeleOp{}


    /*void setPower(double power);

    void getPower();

    public void slideArm(){
        //Slide Arm control
        double tgtPower = 0;
        while (SwerveTeleOp){
            tgtPower = -this.gamepad2.left_stick_y;
            gameMarkDrop.setPower(tgtPower);

            if(gamepad2.y) {
                gameMarkDrop.setPosition(0);
            } else if (gamepad2.x || gamepad1.b) {
                // move to 90 degrees.
                gameMarkDrop.setPosition(0.5);
            } else if (gamepad2.a) {
                // move to 180 degrees.
                gameMarkDrop.setPosition(1);
            }

            telemetry.addData("Servo Position", gameMarkDrop.getPosition());
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", gameMarkDrop.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }



}*/
