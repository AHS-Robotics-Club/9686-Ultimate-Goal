package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.TimedAction;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Charles J. Guiteau")
public class TimedAutonomous extends LinearOpMode {

    private Pose2d startPose = new Pose2d(-63, -40, Math.toRadians(180));

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    private int cameraMonitorViewId;

    private Motor frontLeft, backLeft, frontRight, backRight, shooter, intake, secondaryIntake;
    private SampleMecanumDrive drive;
    private SimpleServo kicker;

    private MecanumDrive mecDrive;
    private ElapsedTime time, timez;
    private VoltageSensor voltageSensor;

    private TimedAction flickerAction;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = new Motor(hardwareMap, "fL");
        backLeft = new Motor(hardwareMap, "fR");
        frontRight = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");

        shooter = new Motor(hardwareMap, "shooter");
        intake = new Motor(hardwareMap, "intake");
        secondaryIntake = new Motor(hardwareMap, "secondaryIntake");
        kicker = new SimpleServo(hardwareMap, "kicker", 0, 360) {
        };
        time = new ElapsedTime();
        timez = new ElapsedTime();

        mecDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();


        frontLeft.setInverted(true);
        frontRight.setInverted(true);

        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setVeloCoefficients(14, 0.1, 0.1);
        shooter.setFeedforwardCoefficients(0, 1.07 * 12 / voltageSensor.getVoltage());

        frontLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flickerAction = new TimedAction(
                () -> kicker.setPosition(1),
                () -> kicker.setPosition(0.1),
                1400,
                true
        );

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

        waitForStart();

        frontLeft.set(0);
        frontRight.set(0);
        backLeft.set(0);
        backRight.set(0);

        time.reset();
        timez.reset();
        UGContourRingPipeline.Height hgt = pipeline.getHeight();
        telemetry.addData("Ring stack size: ", hgt);
        telemetry.addData("Ring stack size: ", hgt);

        telemetry.update();

        Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-10, -11, 0), 0.5)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToSplineHeading(new Pose2d(5, -44, 0), 0)
                .build();

        if (hgt == UGContourRingPipeline.Height.ONE) {
            int pp = 0;
            telemetry.addData("Ring stack size: ", hgt);
            while (opModeIsActive()) {
                telemetry.update();
                drive.update();
                if (pp == 1) {
                    drive.followTrajectory(traj2);
                } else if (pp == 2) {
                    Trajectory tt = shoot(0.76, shooter, kicker, true, drive, traj2);
                    traj3 =  drive.trajectoryBuilder(tt.end())
                            .splineToSplineHeading(new Pose2d(29, -20, 0), 0)
                            .build();
                    drive.followTrajectory(traj3);
                    shooter.set(0);
                }

                pp++;
            }
        } else if (hgt == UGContourRingPipeline.Height.FOUR) {

            int pp = 0;
            while (opModeIsActive()) {
                telemetry.update();
                drive.update();
                if (pp == 1) {
                    drive.followTrajectory(traj2);
                } else if (pp == 2) {
                    Trajectory tt = shoot(0.76, shooter, kicker, true, drive, traj2);
                    traj3 =  drive.trajectoryBuilder(tt.end())
                            .splineToSplineHeading(new Pose2d(53, -44, 0), 0)
                            .build();
                    drive.followTrajectory(traj3);
                    shooter.set(0);
                }

                pp++;
            }
        } else {
            int pp = 0;
            while (opModeIsActive()) {
                telemetry.update();
                drive.update();
                if (pp == 1) {
                    drive.followTrajectory(traj2);
                } else if (pp == 2) {
                    Trajectory tt = shoot(0.76, shooter, kicker, true, drive, traj2);
                    traj3 = drive.trajectoryBuilder(tt.end())
                            .splineToSplineHeading(new Pose2d(5, -44, 0), 0)
                            .build();
                    drive.followTrajectory(traj3);
                    shooter.set(0);
                }

                pp++;
            }
        }
    }

    public static Trajectory shoot(double wapow, Motor shoot, SimpleServo kickaFlicka, boolean isPowershot, SampleMecanumDrive drive, Trajectory endTraj){
        ElapsedTime temp = new ElapsedTime();
        temp.reset();
        temp.startTime();
        System.out.println("Time: " + temp.time());
        if(isPowershot){
            while(temp.time() <= 9.9) {
                shoot.set(wapow);
                if (temp.time() >= 2 && temp.time() <= 3) {
                    kickaFlicka.setPosition(0.3);
                } else if (temp.time() >= 3 && temp.time() <= 4) {
                    kickaFlicka.setPosition(0.1);
                }

                if (temp.time() >= 5 && temp.time() <= 6) {
                    kickaFlicka.setPosition(0.3);
                } else if (temp.time() >= 6 && temp.time() <= 7) {
                    kickaFlicka.setPosition(0.1);
                }

                if (temp.time() >= 8 && temp.time() <= 9) {
                    kickaFlicka.setPosition(0.3);
                } else if (temp.time() >= 9 && temp.time() <= 10) {
                    kickaFlicka.setPosition(0.1);
                }

            }

            return endTraj;

        } else {
            while(temp.time() <= 5) {
                shoot.set(wapow);
                drive.update();
                if (temp.time() >= 0 && temp.time() <= 0.5) {
                    kickaFlicka.setPosition(0.3);
                } else if (temp.time() >= 0.5 && temp.time() <= 1) {
                    kickaFlicka.setPosition(0.1);
                }

                if(temp.time() >= 1.5 && temp.time() < 2){
                    drive.turn(0.261799);
                    drive.update();
                }


                if (temp.time() >= 2 && temp.time() <= 2.5) {
                    kickaFlicka.setPosition(0.3);
                } else if (temp.time() >= 2.5 && temp.time() <= 3) {
                    kickaFlicka.setPosition(0.1);
                }

                if(temp.time() >= 3 && temp.time() < 3.5){
                    drive.turn(0.523598);
                    drive.update();
                }

                if (temp.time() >= 4 && temp.time() <= 4.5) {
                    kickaFlicka.setPosition(0.3);
                } else if (temp.time() >= 4.5 && temp.time() <= 5) {
                    kickaFlicka.setPosition(0.1);
                }
            }

            Trajectory retTraj = drive.trajectoryBuilder(endTraj.end())
                    .splineToSplineHeading(new Pose2d(-10, -11, 0.523598), 0)
                    .build();

            return retTraj;
        }
    }
}
