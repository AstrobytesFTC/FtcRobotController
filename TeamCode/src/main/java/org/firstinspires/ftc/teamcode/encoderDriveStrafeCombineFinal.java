package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class encoderDriveStrafeCombineFinal extends LinearOpMode {

    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;



    double wheelCircumfrence = 326.56;
    //Ticks Per Revolution
    double TPR = 537.7;
    //Centimeters Per Tick
    double CENTIMETERS_PER_TICK = 0.0607;


    public void encoderDrive(double dy, double dx) {
        /* 1. Assign the motors
           2. Make a function that takes forwards and backwards inputs
           3. Get all the motors positions and assign to variables
           4. Calculate what the change you need to make for the motors
           5. Tell the motors to move to the new position
           6. End the function
       */


        //dy and dx are negated due to the robot's inversed movement
        double ticks = -dy / CENTIMETERS_PER_TICK;
        double dxticks = 1.1*(-dx / CENTIMETERS_PER_TICK);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double frontLeftMotorPos = frontLeftMotor.getCurrentPosition();
        double backLeftMotorPos = backLeftMotor.getCurrentPosition();
        double frontRightMotorPos = frontRightMotor.getCurrentPosition();
        double backRightMotorPos = backRightMotor.getCurrentPosition();

        //dy & dx
        double frontLeftMotorTarget = frontLeftMotorPos + ticks + dxticks;
        double frontRightMotorTarget = frontRightMotorPos + ticks - dxticks;
        double backLeftMotorTarget = backLeftMotorPos + ticks - dxticks;
        double backRightMotorTarget = backRightMotorPos + ticks + dxticks;

        //dy
        frontLeftMotor.setTargetPosition((int) frontLeftMotorTarget);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setPower(0.5);

        backLeftMotor.setTargetPosition((int) backLeftMotorTarget);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setPower(0.5);

        frontRightMotor.setTargetPosition((int) frontRightMotorTarget);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setPower(0.5);

        backRightMotor.setTargetPosition((int) backRightMotorTarget);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setPower(0.5);


        //dx
        frontLeftMotor.setTargetPosition((int) frontLeftMotorTarget);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setPower(0.5);

        backLeftMotor.setTargetPosition((int) backLeftMotorTarget);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setPower(0.5);

        frontRightMotor.setTargetPosition((int) frontRightMotorTarget);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setPower(0.5);

        backRightMotor.setTargetPosition((int) backRightMotorTarget);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setPower(0.5);

        //dy & dx
        while ((Math.abs(backLeftMotor.getCurrentPosition() - backLeftMotor.getTargetPosition()) > 5)
                && (Math.abs(backRightMotor.getCurrentPosition() - backRightMotor.getTargetPosition()) > 5)
                && (Math.abs(frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition()) > 5)
                && Math.abs(frontRightMotor.getCurrentPosition() - frontRightMotor.getTargetPosition()) > 5) {
            sleep(67);

        }


        telemetry.addData("frontLeftTarget", frontLeftMotorTarget);
        telemetry.addData("frontRightTarget", frontRightMotorTarget);
        telemetry.addData("backLeftTarget", backLeftMotorTarget);
        telemetry.addData("backRightTarget", backRightMotorTarget);

        telemetry.update();

    }


    @Override
    public void runOpMode() throws InterruptedException {
        // Position of the wheel after driving forward

         frontLeftMotor = hardwareMap.dcMotor.get("left_front_drive");
         backLeftMotor = hardwareMap.dcMotor.get("left_back_drive");
         frontRightMotor = hardwareMap.dcMotor.get("right_front_drive");
         backRightMotor = hardwareMap.dcMotor.get("right_back_drive");





        // Reset the motor encoder so that it reads zero ticks

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
            if(gamepad1.a){
                encoderDrive(67, 67);
            }


            if(gamepad1.b){
                encoderDrive(67, 0);
            }

            if(gamepad1.y){
                encoderDrive(0, 67);
            }

            telemetry.addData("frontLeftMotorPos:", frontLeftMotor.getCurrentPosition());
            telemetry.addData("frontRightMotorPos:", frontRightMotor.getCurrentPosition());
            telemetry.addData("backLeftMotorPos", backLeftMotor.getCurrentPosition());
            telemetry.addData("backRightMotorPos", backRightMotor.getCurrentPosition());



            telemetry.update();




        }


    }
}


