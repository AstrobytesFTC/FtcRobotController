package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class ArmMotorDegrees extends LinearOpMode {

    DcMotor armMotor = null;
//TPR is from specs sheet for the arm motor
    double TPR = 1425.1;
    double ticks;

    public void encoderArm(double degrees){
        /*This is the formula to translate degrees to ticks, 1425.1 * 5 * (degress/360)
        It is multiplied by five for there being a gear ratio 1:5.
        1425.1 is the Ticks Per Rotation
         */
        ticks = 7125.5 * (degrees / 360) ;

        double armMotorPos = armMotor.getCurrentPosition();

        double armMotorTarget = armMotorPos + ticks;

        //Movement Code
        armMotor.setTargetPosition((int) armMotorTarget);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
        // Code for waiting until the motor is at correct position.
        while(Math.abs(armMotor.getCurrentPosition() - armMotor.getTargetPosition()) > 5){
            sleep(67);
            //telemetry for more data ig
            telemetry.addData("armMotor", armMotor.getCurrentPosition());

            telemetry.addData("armMotorTarget", armMotorTarget);

            telemetry.update();
        }


    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        //Intializing Servos




        armMotor = hardwareMap.dcMotor.get("arm_motor");


        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);





        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.


        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;
        // Code for executing encoderArm
        while (opModeIsActive()) {
            if(gamepad1.a){
                encoderArm(45);
            }





        }
    }
}
