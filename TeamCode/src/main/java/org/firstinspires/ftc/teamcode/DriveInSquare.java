package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class DriveInSquare extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Position of the wheel after driving forward
        int droveForwardPos = 600;

        int droveBackwardPos = -600;



        // Original position of the wheel
        int startingPos = 0;



        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("left_front_drive");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("left_back_drive");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("right_front_drive");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("right_back_drive");

        // Reset the motor encoder so that it reads zero ticks
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets the starting position of the wheel to the down position
        //frontLeftMotor.setTargetPosition(startingPos);
        //frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

        while (opModeIsActive()) {
            // If the A button is pressed, move in a square
            if (gamepad1.a) {
                frontLeftMotor.setTargetPosition(droveForwardPos);
                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeftMotor.setPower(0.5);

                backLeftMotor.setTargetPosition(droveForwardPos);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setPower(0.5);

                frontRightMotor.setTargetPosition(droveForwardPos);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightMotor.setPower(0.5);

                backRightMotor.setTargetPosition(droveForwardPos);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setPower(0.5);

                sleep(2000);

                frontLeftMotor.setTargetPosition(droveBackwardPos);
                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeftMotor.setPower(-0.5);


                backLeftMotor.setTargetPosition(droveForwardPos);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setPower(0.5);

                frontRightMotor.setTargetPosition(droveForwardPos);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightMotor.setPower(0.5);

                backRightMotor.setTargetPosition(droveBackwardPos);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setPower(-0.5);

                sleep(2000);

                frontLeftMotor.setTargetPosition(droveBackwardPos);
                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeftMotor.setPower(-0.5);


                backLeftMotor.setTargetPosition(droveBackwardPos);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setPower(-0.5);

                frontRightMotor.setTargetPosition(droveBackwardPos);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightMotor.setPower(-0.5);

                backRightMotor.setTargetPosition(droveBackwardPos);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setPower(-0.5);

                sleep(2000);

                frontLeftMotor.setTargetPosition(droveForwardPos);
                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeftMotor.setPower(0.5);

                backLeftMotor.setTargetPosition(droveBackwardPos);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setPower(-0.5);

                frontRightMotor.setTargetPosition(droveBackwardPos);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightMotor.setPower(-0.5);

                backRightMotor.setTargetPosition(droveForwardPos);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setPower(0.5);


            }



            // Get the current position of the wheel
            double position = frontLeftMotor.getCurrentPosition();

            // Get the target position of the wheel
            double desiredPosition = frontLeftMotor.getTargetPosition();

            // Show the position of the armMotor on telemetry
            telemetry.addData("Encoder Position", position);

            // Show the target position of the armMotor on telemetry
           // telemetry.addData("Desired Position", desiredPosition);

            telemetry.addLine("This is for Testing");

            telemetry.update();
        }
    }
}


