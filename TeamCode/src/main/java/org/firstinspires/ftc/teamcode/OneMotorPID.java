package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp
public class OneMotorPID extends LinearOpMode {


    FtcDashboard dashboard = FtcDashboard.getInstance();

    // Send telemetry to BOTH Driver Station and Dashboard


    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double integralSum = 0;

    ElapsedTime timer = new ElapsedTime();
    double lastError = 0;

    public double PIDCONTROL(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();
        double output = (error * Kp) + (derivative *Kd) + (integralSum *Ki);
        return output;

    }


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        //Intializing Servos
        Telemetry dashboardTelemetry = dashboard.getTelemetry();




        DcMotor oneMotor = hardwareMap.dcMotor.get("onemotor");


        oneMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



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

        while (opModeIsActive()) {
            oneMotor.setPower(PIDCONTROL(100, oneMotor.getCurrentPosition()));
            dashboardTelemetry.addData("Motor Power", oneMotor.getPower());
            dashboardTelemetry.addData("Encoder Position", oneMotor .getCurrentPosition());

            // Update the dashboard with the new telemetry data
            dashboardTelemetry.update();


        }
    }
}
