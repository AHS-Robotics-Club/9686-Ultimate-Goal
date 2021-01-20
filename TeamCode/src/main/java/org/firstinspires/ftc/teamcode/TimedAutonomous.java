package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Charles J. Guiteau")
public class TimedAutonomous extends LinearOpMode {
    //private UGContourRingPipeline pipeline;
    //private OpenCvCamera camera;

    //private int cameraMonitorViewId;

    private Motor frontLeft, backLeft, frontRight, backRight, shooter;
    private SimpleServo kicker;

    private MecanumDrive mecDrive;
    private ElapsedTime time, timez;
    private VoltageSensor voltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = new Motor(hardwareMap, "fL");
        backLeft = new Motor(hardwareMap, "fR");
        frontRight = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");

        shooter = new Motor(hardwareMap, "shooter");
        kicker = new SimpleServo(hardwareMap, "kicker", 0.67, 1) {
        };
        time = new ElapsedTime();
        timez = new ElapsedTime();

        mecDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        backRight.setInverted(true);
        frontLeft.setInverted(true);
        frontRight.setInverted(true);
        backLeft.setInverted(true);
        shooter.setInverted(true);

        frontLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

/*
        cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Jesus"), cameraMonitorViewId);
        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, true));
        camera.openCameraDeviceAsync(() -> camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));
*/
        waitForStart();

        frontLeft.set(0);
        frontRight.set(0);
        backLeft.set(0);
        backRight.set(0);

        time.reset();
        timez.reset();
        telemetry.addData("Started Timer", time.seconds());
        telemetry.update();

        //UGContourRingPipeline.Height hgt = pipeline.getHeight();

        while (opModeIsActive() && time.seconds() < 16) {
            double voltage = voltageSensor.getVoltage();
            double shooterSpeed = Math.sqrt(12.35/voltage);

            shooter.set(0.749 * shooterSpeed);
            for(int i = 0; i < 2; i++){
                timez.reset();
                while (!isStopRequested() && timez.milliseconds() < 4000) {
                    idle();
                }
                kicker.setPosition(1.0);
                timez.reset();

                while (!isStopRequested() && timez.milliseconds() < 200) {
                    idle();
                }
                kicker.setPosition(0.67);
                timez.reset();

                while (!isStopRequested() && timez.milliseconds() < 1000) {
                    idle();
                }
            }
        }

        time.reset();

        while (opModeIsActive() && time.seconds() < 2.95) {
            shooter.set(0);

            mecDrive.driveRobotCentric(0.35, 0, 0);

            telemetry.addData("Current time", time.seconds());
            telemetry.update();
        }
/*

        if(hgt == UGContourRingPipeline.Height.ONE){
            while(opModeIsActive() && time.seconds() < 1.6) {
                frontLeft.set(-0.7);
                frontRight.set(-0.7);
                backLeft.set(-0.7);
                backRight.set(-0.7);
                telemetry.addData("Current time", time.seconds());
                telemetry.update();
            }
        } else if (hgt == UGContourRingPipeline.Height.FOUR){
            while(opModeIsActive() && time.seconds() < 0.10) {
                frontLeft.set(0.7);
                frontRight.set(-0.7);
                backLeft.set(0.7);
                backRight.set(-0.7);
                telemetry.addData("Current time", time.seconds());
                telemetry.update();
            }

            time.reset();

            while(opModeIsActive() && time.seconds() < 1.9) {
                frontLeft.set(-0.7);
                frontRight.set(-0.7);
                backLeft.set(-0.7);
                backRight.set(-0.7);
                telemetry.addData("Current time", time.seconds());
                telemetry.update();
            }
        } else {
            while(opModeIsActive() && time.seconds() < 0.12) {
                frontLeft.set(0.7);
                frontRight.set(-0.7);
                backLeft.set(0.7);
                backRight.set(-0.7);
                telemetry.addData("Current time", time.seconds());
                telemetry.update();
            }

            time.reset();

            while(opModeIsActive() && time.seconds() < 0.80) {
                frontLeft.set(-0.7);
                frontRight.set(-0.7);
                backLeft.set(-0.7);
                backRight.set(-0.7);
                telemetry.addData("Current time", time.seconds());
                telemetry.update();
            }

 */
    }
}

