package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.HashMap;
import java.util.List;

@TeleOp(name = "Record_Autonomous")
public class Record_Autonomous extends LinearOpMode {
    List<HashMap<String, Double>> recording;

    boolean isRecording = false;
    final ElapsedTime runtime = new ElapsedTime();

    boolean isPlaying = false;
    int iteration = 0;
    
    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor; //Declaring variables used for motors

    IMU imu;

    @Override
    public void runOpMode() {

        //Attaching the variables declared with the physical motors by name or id
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //Increasing efficiency in getting data from the robot
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //Updating the user screen
        telemetry.addData("Status", "Waiting to Start");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Waiting to start recording");
        telemetry.update();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (gamepad1.right_bumper) {
                    imu.resetYaw();
                }

                if (gamepad1.start) {
                    isRecording = true;
                    runtime.reset();
                    telemetry.addData("Status", "Recording");
                    telemetry.addData("Time until recording end", 30 - runtime.time());
                }

                if (gamepad1.a){
                    isPlaying = true;
                    telemetry.addData("Status", "Playing recording");
                    playRecording(recording);
                }


                if (isRecording && (30 - runtime.time()) > 0) {
                    telemetry.addData("Status", "Recording");
                    telemetry.addData("Time until recording end", 30 - runtime.time());

                    double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    HashMap<String, Double> values = robotMovement(botHeading);

                    recording.add(values);

                    telemetry.addData("RotY", "RotY=%d RotX=%d rx=%d",
                            values.get("rotY"),
                            values.get("rotX"),
                            values.get("rx"));


                } else if ((30 - runtime.time()) <= 0 && isRecording) {
                    isRecording = false;
                    telemetry.clearAll();
                    telemetry.addData("Status", "Waiting to play Recording");
                }

                if(isPlaying && iteration < recording.size()){
                    playRecording(recording);
                }
                
            }

            telemetry.update();
        }
    }

    private void playRecording(List<HashMap<String, Double>> recording){
        //GETTING VALUES FROM CURRENT TIME STAMP IN RECORDING
        HashMap<String, Double> values = recording.get(iteration);

        double rotY = values.getOrDefault("rotY", 0.0);
        double rotX = values.getOrDefault("rotX", 0.0);
        double rx = values.getOrDefault("rx", 0.0);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        if (denominator >= 0.3) {
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }

        iteration += 1;
    }


    private HashMap<String, Double> robotMovement(double botHeading) {

        HashMap<String, Double> values = new HashMap<>();

        //Gets inputs from the controller
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        //Uses constant 0 direction to determine where forward is for the robot is.
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        //Makes sure power of each engine is not below 100% (Math cuts anything above 1.0 to 1.0, meaning you can lose values unless you change values)
        //This gets the highest possible outcome, and if it's over 1.0, it will lower all motor powers by the same ratio to make sure powers stay equal
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        //Makes sure robot doesn't move without intentional inputs
        if (denominator >= 0.3) {
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }

        values.put("rotY", rotY);
        values.put("rotX", rotX);
        values.put("rx", rx);

        return values;
    }



}
