package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.TimedAction;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;

@Autonomous(name="Charles J. Guiteau")
public class TimedAutonomous extends LinearOpMode {

    private Pose2d startPose = new Pose2d(-63, -40, Math.toRadians(180));

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    private int cameraMonitorViewId;

    private Motor frontLeft, backLeft, frontRight, backRight, shooter, intake, secondaryIntake, wobbleArm;
    private SampleMecanumDrive drive;
    private SimpleServo kicker, wobbleFingers;

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
        kicker = new SimpleServo(hardwareMap, "kicker", 0, 270) {
        };
        wobbleFingers = new SimpleServo(hardwareMap, "wobbleFingers", 0, 270);
        wobbleArm = new Motor(hardwareMap, "wobbleArm");
        time = new ElapsedTime();
        timez = new ElapsedTime();

        mecDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();


        frontLeft.setInverted(true);
        frontRight.setInverted(true);
        shooter.setInverted(true);


        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setVeloCoefficients(0.3,0,0);
        shooter.setFeedforwardCoefficients(1, 1.305 * 12 / voltageSensor.getVoltage());

        frontLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wobbleArm.resetEncoder();

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

        wobbleFingers.setPosition(0.71);

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
                .splineToSplineHeading(
                        new Pose2d(-10, -11, Math.toRadians(-1.8)), 0.5,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                //.splineToSplineHeading(new Pose2d(-10, -11, Math.toRadians(-5)), 0.5)
                .build();
        Trajectory traj3 = null;
/*
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToSplineHeading(new Pose2d(5, -44, 0), 0)
                .build();

 */

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineToSplineHeading(startPose, 0)
                .build();

        Trajectory traj5 = null;

        if (hgt == UGContourRingPipeline.Height.ONE) {
            int pp = 0;
            telemetry.addData("Ring stack size: ", hgt);
            while (opModeIsActive()) {
                telemetry.update();
                drive.update();
                if (pp == 1) {
                    drive.followTrajectory(traj2);
                } else if (pp == 2) {
                    shoot(0.82, shooter, kicker, true, drive, traj2);
                    shooter.set(0);
                    telemetry.addData("running 1 ring lol", 1);
                    traj3 =  drive.trajectoryBuilder(traj2.end())
                            .lineToLinearHeading(new Pose2d(29, -20, 0))
                            .build();
                    drive.followTrajectory(traj3);
                } else if(pp == 3){
                    wobbleGrab(wobbleFingers, wobbleArm, false);
                } else if(pp == 4){
                    drive.followTrajectory(traj4);
                } else if(pp == 5){
                    wobbleGrab(wobbleFingers, wobbleArm, true);
                } else if(pp == 6){
                    traj5 = drive.trajectoryBuilder(traj4.end())
                            .lineToLinearHeading(new Pose2d(29, -20, 0))
                            .build();
                    drive.followTrajectory(traj5);
                } else if(pp == 7){
                    wobbleGrab(wobbleFingers, wobbleArm, false);
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
                    shoot(0.82, shooter, kicker, true, drive, traj2);
                    shooter.set(0);
                    telemetry.addData("running 4 ring lol", 4);
                    traj3 =  drive.trajectoryBuilder(traj2.end())
                            .splineToSplineHeading(new Pose2d(53, -44, 0), 0)
                            .build();
                    drive.followTrajectory(traj3);
                } else if(pp == 3){
                    wobbleGrab(wobbleFingers, wobbleArm, false);
                } else if(pp == 4){
                    drive.followTrajectory(traj4);
                } else if(pp == 5){
                    wobbleGrab(wobbleFingers, wobbleArm, true);
                } else if(pp == 6){
                    traj5 = drive.trajectoryBuilder(traj4.end())
                            .lineToLinearHeading(new Pose2d(53, -44, 0))
                            .build();
                    drive.followTrajectory(traj5);
                } else if(pp == 7){
                    wobbleGrab(wobbleFingers, wobbleArm, false);
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
                    shoot(0.82, shooter, kicker, true, drive, traj2);
                    shooter.set(0);
                    telemetry.addData("running 0 ring lol", 0);
                    traj3 = drive.trajectoryBuilder(traj2.end())
                            .lineToLinearHeading(new Pose2d(5, -44, 0))
                            .build();
                    drive.followTrajectory(traj3);
                } else if(pp == 3){
                    wobbleGrab(wobbleFingers, wobbleArm, false);
                } else if(pp == 4){
                    drive.followTrajectory(traj4);
                } else if(pp == 5){
                    wobbleGrab(wobbleFingers, wobbleArm, true);
                } else if(pp == 6){
                    traj5 = drive.trajectoryBuilder(traj4.end())
                            .lineToLinearHeading(new Pose2d(5, -44, 0))
                            .build();
                    drive.followTrajectory(traj5);
                } else if(pp == 7){
                    wobbleGrab(wobbleFingers, wobbleArm, false);
                }


                pp++;
            }
        }
    }

    public static void shoot(double wapow, Motor shoot, SimpleServo kickaFlicka, boolean isPowershot, SampleMecanumDrive drive, Trajectory endTraj){
        ElapsedTime temp = new ElapsedTime();
        temp.reset();
        temp.startTime();
        System.out.println("Time: " + temp.time());
        if(isPowershot){
            while(temp.time() <= 6.4) {
                shoot.set(wapow);
                drive.update();

                if (temp.time() >= 2 && temp.time() <= 2.2) {
                    kickaFlicka.setPosition(0.3);
                } else if (temp.time() >= 2.2 && temp.time() <= 2.4) {
                    kickaFlicka.setPosition(0);
                }


                if(temp.time() >= 3 && temp.time() < 3.5){
                    drive.turn(Math.toRadians(16));
                    drive.update();
                }

                if (temp.time() >= 4 && temp.time() <= 4.2) {
                    kickaFlicka.setPosition(0.3);
                } else if (temp.time() >= 4.2 && temp.time() <= 4.4) {
                    kickaFlicka.setPosition(0);
                }

                if(temp.time() >= 5 && temp.time() < 5.5){
                    drive.turn(Math.toRadians(17));
                    drive.update();
                }

                if (temp.time() >= 6 && temp.time() <= 6.2) {
                    kickaFlicka.setPosition(0.3);
                } else if (temp.time() >= 6.2 && temp.time() <= 6.4) {
                    kickaFlicka.setPosition(0);
                }

            }

        } else {

            while(temp.time() <= 4.4) {
                shoot.set(1);

                if (temp.time() >= 0.1 && temp.time() <= 1) {
                    drive.turn(Math.toRadians(-13));
                    drive.update();
                }


                if (temp.time() >= 2 && temp.time() <= 2.2) {
                    kickaFlicka.setPosition(0.3);
                } else if (temp.time() >= 2.2 && temp.time() <= 2.4) {
                    kickaFlicka.setPosition(0);
                }


                if (temp.time() >= 3 && temp.time() <= 3.2) {
                    kickaFlicka.setPosition(0.3);
                } else if (temp.time() >= 3.2 && temp.time() <= 3.4) {
                    kickaFlicka.setPosition(0);
                }

                if (temp.time() >= 4 && temp.time() <= 4.2) {
                    kickaFlicka.setPosition(0.3);
                } else if (temp.time() >= 4.2 && temp.time() <= 4.4) {
                    kickaFlicka.setPosition(0);
                }
            }
        }

        shoot.set(0);

    }

    public static void wobbleGrab(SimpleServo fingies, Motor armie, boolean pickUp){
        ElapsedTime tomp = new ElapsedTime();
        tomp.reset();
        tomp.startTime();
        if(pickUp){
            if(tomp.time() >= 0.1 && tomp.time() <= 0.5){
                fingies.setPosition(0.71);
                //armie.setPosition();
            }
        } else {
            if(tomp.time() >= 0.1 && tomp.time() <= 0.5){
                fingies.setPosition(0.25);
                //armie.setPosition();
            }

        }
    }
}


