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

    private Motor frontLeft, backLeft, frontRight, backRight, intake, secondaryIntake, shooterF, shooterB;
    private SampleMecanumDrive drive;
    private SimpleServo kicker, wobbleFingers, wobbleArmL, wobbleArmR;

    private MecanumDrive mecDrive;
    private ElapsedTime time, timez;
    private VoltageSensor voltageSensor;

    private TimedAction flickerAction;
    static double tt;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = new Motor(hardwareMap, "fL");
        backLeft = new Motor(hardwareMap, "fR");
        frontRight = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");

        shooterF = new Motor(hardwareMap, "shooterF");
        shooterB = new Motor(hardwareMap, "shooterB");
        intake = new Motor(hardwareMap, "intake");
        secondaryIntake = new Motor(hardwareMap, "secondaryIntake");
        kicker = new SimpleServo(hardwareMap, "kicker", 0, 270);
        wobbleFingers = new SimpleServo(hardwareMap, "wobbleFingers", 0, 270);
        wobbleArmL = new SimpleServo(hardwareMap, "wobbleArmL", 0, 270);
        wobbleArmR = new SimpleServo(hardwareMap, "wobbleArmR", 0, 270);
        time = new ElapsedTime();
        timez = new ElapsedTime();

        mecDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        tt = 12 / voltageSensor.getVoltage();


        frontLeft.setInverted(true);
        frontRight.setInverted(true);
        shooterB.setInverted(true);

        shooterF.setRunMode(Motor.RunMode.VelocityControl);
        shooterF.setVeloCoefficients(21.2,0,0.2);
        shooterF.setFeedforwardCoefficients(0, 1.4 * 12 / voltageSensor.getVoltage());

        shooterB.setRunMode(Motor.RunMode.VelocityControl);
        shooterB.setVeloCoefficients(21.2,0,0.2);
        shooterB.setFeedforwardCoefficients(0, 1.4 * 12 / voltageSensor.getVoltage());


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

        //down is 0, up is 300

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

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToSplineHeading(new Pose2d(5, -44, 0), 0)
                .build();



        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(-39, -16,Math.toRadians(-90)))
                .build();

        Trajectory traj5 = null;

        if (hgt == UGContourRingPipeline.Height.ONE) {
            int pp = 0;
            telemetry.addData("Ring stack size: ", hgt);
            while (opModeIsActive()) {
                shooterF.set(0.88);
                shooterB.set(0.88);
                telemetry.update();
                drive.update();
                if (pp == 1) {
                    Trajectory traj2_5 = drive.trajectoryBuilder(startPose)
                            .lineToLinearHeading(new Pose2d(-63, -52, Math.toRadians(180)))
                            .build();
                    drive.followTrajectory(traj2_5);

                    Trajectory traj2_2 = drive.trajectoryBuilder(traj2_5.end())
                            .lineToLinearHeading(new Pose2d(-18, -52, Math.toRadians(0)))
                            .build();

                    drive.followTrajectory(traj2_2);

                    traj2 = drive.trajectoryBuilder(traj2_2.end())
                            .lineToLinearHeading(new Pose2d(-18.5, -32, Math.toRadians(0)))
                            .build();
                    drive.followTrajectory(traj2);
                    frontLeft.set(1.0);
                    frontRight.set(1.0);
                    backLeft.set(1.0);
                    backRight.set(1.0);
                    Thread.sleep(80);
                    frontLeft.set(-1);
                    frontRight.set(-1);
                    backLeft.set(-1);
                    backRight.set(-1);
                    Thread.sleep(80);
                    frontLeft.set(0);
                    frontRight.set(0);
                    backLeft.set(0);
                    backRight.set(0);
                } else if (pp == 2) {
                    shoot(0.88, shooterF, shooterB, kicker, false, drive, traj2);
                    telemetry.addData("running 1 ring lol", 1);
                    traj3 =  drive.trajectoryBuilder(traj2.end())
                            .lineToLinearHeading(new Pose2d(36, -27, 0))
                            .build();
                    drive.followTrajectory(traj3);

                } else if(pp == 3){
                    wobbleArmL.setPosition(0);
                    wobbleArmR.setPosition(0);
                    Thread.sleep(750);
                    wobbleFingers.setPosition(0.25);
                } else if(pp == 4){
                    Trajectory traj3_5 = drive.trajectoryBuilder(traj3.end())
                            .lineToLinearHeading(new Pose2d(-40, 1, 0))
                            .build();
                    drive.followTrajectory(traj3_5);

                    traj4 = drive.trajectoryBuilder(traj3_5.end())
                            .lineToLinearHeading(new Pose2d(-41.5, -17.7, Math.toRadians(-90)))
                            .build();
                    drive.followTrajectory(traj4);

                } else if(pp == 5){
                    wobbleFingers.setPosition(0.71);
                    Thread.sleep(500);
                    wobbleArmL.setPosition(300);
                    wobbleArmR.setPosition(300);
                } else if(pp == 6){
                    traj5 = drive.trajectoryBuilder(traj4.end())
                            .lineToLinearHeading(new Pose2d(26, -22, 0))
                            .build();

                    drive.followTrajectory(traj5);
/*
                    traj5 = drive.trajectoryBuilder(trajtemp.end())
                            .lineToLinearHeading(new Pose2d(26, -21, 0))
                            .build();
                    drive.followTrajectory(traj5);

 */

                } else if(pp == 7){
                    wobbleArmL.setPosition(0);
                    wobbleArmR.setPosition(0);
                    Thread.sleep(750);
                    wobbleFingers.setPosition(0.25);

                } else if(pp == 8){
                    Trajectory endTraj = drive.trajectoryBuilder(traj5.end())
                            .lineToLinearHeading(new Pose2d(4, 0, 0))
                            .build();
                    drive.followTrajectory(endTraj);
                }

                pp++;
            }
        } else if (hgt == UGContourRingPipeline.Height.FOUR) {

            int pp = 0;
            telemetry.addData("Ring stack size: ", hgt);
            while (opModeIsActive()) {
                shooterB.set(0.88);
                shooterF.set(0.88);
                telemetry.update();
                drive.update();
                if (pp == 1) {
                    Trajectory traj2_5 = drive.trajectoryBuilder(startPose)
                            .lineToLinearHeading(new Pose2d(-63, -52, Math.toRadians(180)))
                            .build();
                    drive.followTrajectory(traj2_5);

                    Trajectory traj2_2 = drive.trajectoryBuilder(traj2_5.end())
                            .lineToLinearHeading(new Pose2d(-18, -52, Math.toRadians(0)))
                            .build();

                    drive.followTrajectory(traj2_2);

                    traj2 = drive.trajectoryBuilder(traj2_2.end())
                            .lineToLinearHeading(new Pose2d(-18.5, -32, Math.toRadians(0)))
                            .build();
                    drive.followTrajectory(traj2);

                    frontLeft.set(1.0);
                    frontRight.set(1.0);
                    backLeft.set(1.0);
                    backRight.set(1.0);
                    Thread.sleep(80);
                    frontLeft.set(-1);
                    frontRight.set(-1);
                    backLeft.set(-1);
                    backRight.set(-1);
                    Thread.sleep(80);
                    frontLeft.set(0);
                    frontRight.set(0);
                    backLeft.set(0);
                    backRight.set(0);

                } else if (pp == 2) {
                    shoot(0.88, shooterB, shooterF, kicker, false, drive, traj2);
                    shooterB.set(0);
                    shooterF.set(0);
                    telemetry.addData("running 4 ring lol", 4);
                    traj3 =  drive.trajectoryBuilder(traj2.end())
                            .lineToLinearHeading(new Pose2d(53, -51, 0))
                            .build();
                    drive.followTrajectory(traj3);

                } else if(pp == 3){
                    wobbleArmL.setPosition(0);
                    wobbleArmR.setPosition(0);
                    wobbleFingers.setPosition(0.25);
                } else if(pp == 4){
                    Trajectory traj3_5 = drive.trajectoryBuilder(traj3.end())
                            .lineToLinearHeading(new Pose2d(-40, 1, 0))
                            .build();
                    drive.followTrajectory(traj3_5);

                    traj4 = drive.trajectoryBuilder(traj3_5.end())
                            .lineToLinearHeading(new Pose2d(-42, -15.7, Math.toRadians(-90)))
                            .build();
                    drive.followTrajectory(traj4);

                } else if(pp == 5){
                    wobbleFingers.setPosition(0.71);
                    Thread.sleep(500);
                    wobbleArmL.setPosition(300);
                    wobbleArmR.setPosition(300);
                } else if(pp == 6){
                    Trajectory trajtemp = drive.trajectoryBuilder(traj4.end())
                            .lineToLinearHeading(new Pose2d(46, -20))
                            .build();

                    drive.followTrajectory(trajtemp);

                    traj5 = drive.trajectoryBuilder(trajtemp.end())
                            .lineToLinearHeading(new Pose2d(46, -53, 0))
                            .build();

                    drive.followTrajectory(traj5);
/*
                    traj5 = drive.trajectoryBuilder(trajtemp.end())
                            .lineToLinearHeading(new Pose2d(26, -21, 0))
                            .build();
                    drive.followTrajectory(traj5);

 */

                } else if(pp == 7){
                    wobbleFingers.setPosition(0.25);
                    Thread.sleep(750);
                    wobbleArmL.setPosition(0);
                    wobbleArmR.setPosition(0);
                } else if(pp == 8){
                    Trajectory endTraj = drive.trajectoryBuilder(traj5.end())
                            .lineToLinearHeading(new Pose2d(4, 0, 0))
                            .build();
                    drive.followTrajectory(endTraj);
                }

                pp++;
            }
        } else {
            int pp = 0;
            while (opModeIsActive()) {
                shooterB.set(0.8);
                shooterF.set(0.8);
                telemetry.update();
                drive.update();
                if (pp == 1) {
                    traj2 = drive.trajectoryBuilder(startPose)
                            .back(5)
                            .splineToSplineHeading(
                                    new Pose2d(-28, -31, Math.toRadians(0)), 0,
                                    new MinVelocityConstraint(
                                            Arrays.asList(
                                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                    new MecanumVelocityConstraint(32, DriveConstants.TRACK_WIDTH)
                                            )
                                    ),
                                    new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                            )
                            //.splineToSplineHeading(new Pose2d(-10, -11, Math.toRadians(-5)), 0.5)
                            .build();
                    drive.followTrajectory(traj2);


                    frontLeft.set(1.0);
                    frontRight.set(1.0);
                    backLeft.set(1.0);
                    backRight.set(1.0);
                    Thread.sleep(80);
                    frontLeft.set(-1);
                    frontRight.set(-1);
                    backLeft.set(-1);
                    backRight.set(-1);
                    Thread.sleep(80);
                    frontLeft.set(0);
                    frontRight.set(0);
                    backLeft.set(0);
                    backRight.set(0);
                } else if (pp == 2) {
                    shoot(0.8, shooterB, shooterF, kicker, false, drive, traj2);
                    shooterB.set(0);
                    shooterF.set(0);
                    telemetry.addData("running 0 ring lol", 0);
                    traj3 = drive.trajectoryBuilder(traj2.end())
                            .lineToLinearHeading(new Pose2d(13, -44.5, 0))
                            .build();
                    drive.followTrajectory(traj3);
                } else if(pp == 3){
                    wobbleArmL.setPosition(0);
                    wobbleArmR.setPosition(0);
                    Thread.sleep(300);
                    wobbleFingers.setPosition(0.25);
                } else if(pp == 4){
                    Trajectory traj4_1 = drive.trajectoryBuilder(traj3.end())
                            .strafeLeft(10)
                            .build();
                    drive.followTrajectory(traj4_1);

                    traj4 = drive.trajectoryBuilder(traj4_1.end())
                            .lineToLinearHeading(new Pose2d(-41, -22.1, Math.toRadians(-90)))
                            .build();
                    drive.followTrajectory(traj4);
                } else if(pp == 5){
                    wobbleFingers.setPosition(0.71);
                    Thread.sleep(500);
                    wobbleArmL.setPosition(300);
                    wobbleArmR.setPosition(300);
                } else if(pp == 6){
                    traj5 = drive.trajectoryBuilder(traj4.end())
                            .lineToLinearHeading(new Pose2d(4, -40, 0))
                            .build();
                    drive.followTrajectory(traj5);
                } else if(pp == 7){
                    wobbleArmL.setPosition(0);
                    wobbleArmR.setPosition(0);
                    Thread.sleep(750);
                    wobbleFingers.setPosition(0.25);
                    Thread.sleep(300);
                } else if(pp == 8){
                    Trajectory traj6 = drive.trajectoryBuilder(traj4.end())
                            .lineToLinearHeading(new Pose2d(5, -20, 0))
                            .build();
                    drive.followTrajectory(traj6);
                }
                pp++;
            }
        }
    }

    public static void shoot(double wapow, Motor shoot, Motor shoot1, SimpleServo kickaFlicka, boolean isPowershot, SampleMecanumDrive drive, Trajectory endTraj){
        ElapsedTime temp = new ElapsedTime();
        temp.reset();
        temp.startTime();
        System.out.println("Time: " + temp.time());

        kickaFlicka.setPosition(0.2);

        if(isPowershot){
            while(temp.time() <= 6.9) {
                shoot.set(wapow);
                shoot1.set(wapow);
                drive.update();

                if (temp.time() >= 2.5 && temp.time() <= 2.7) {
                    kickaFlicka.setPosition(0.4);
                } else if (temp.time() >= 2.7 && temp.time() <= 2.9) {
                    kickaFlicka.setPosition(0.2);
                }


                if(temp.time() >= 3 && temp.time() < 3.5){
                    drive.turn(Math.toRadians(16));
                    drive.update();
                }

                if (temp.time() >= 4 && temp.time() <= 4.5) {
                    kickaFlicka.setPosition(0.4);
                } else if (temp.time() >= 4.5 && temp.time() <= 4.9) {
                    kickaFlicka.setPosition(0);
                }

                if(temp.time() >= 5 && temp.time() < 5.5){
                    drive.turn(Math.toRadians(17));
                    drive.update();
                }

                if (temp.time() >= 6 && temp.time() <= 6.5) {
                    kickaFlicka.setPosition(0.4);
                } else if (temp.time() >= 6.5 && temp.time() <= 6.9) {
                    kickaFlicka.setPosition(0);
                }

            }

        } else {

            while(temp.time() <= 2.9) {
                shoot.set(wapow);
                shoot1.set(wapow);
                drive.update();

                if (temp.time() >= 0.1 && temp.time() <= 0.3) {
                    drive.turn(Math.toRadians(14.5));
                    drive.update();
                }


                if (temp.time() >= 0.4 && temp.time() <= 0.7) {
                    shoot.set(wapow);
                    shoot1.set(wapow);
                    kickaFlicka.setPosition(0.4);
                } else if (temp.time() > 0.8 && temp.time() <= 1.1) {
                    kickaFlicka.setPosition(0.2);
                }


                if (temp.time() >= 1.3 && temp.time() <= 1.6) {
                    kickaFlicka.setPosition(0.4);
                } else if (temp.time() > 1.7 && temp.time() <= 2.0) {
                    kickaFlicka.setPosition(0.2);
                }

                if (temp.time() >= 2.2 && temp.time() <= 2.5) {
                    kickaFlicka.setPosition(0.4);
                } else if (temp.time() > 2.6 && temp.time() <= 2.9) {
                    kickaFlicka.setPosition(0.2);
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
            while(tomp.time() <= 0.8){
                if(tomp.time() >= 0.1 && tomp.time() <= 0.3){
                    armie.setTargetPosition(-75);
                } else if (tomp.time() >= 0.6 && tomp.time() <= 0.8){
                    fingies.setPosition(0.71);
                } else {
                    armie.set(0);
                }
                if(armie.atTargetPosition()){
                    armie.stopMotor();
                }
            }
        } else {
            while(tomp.time() <= 0.8){
                if(tomp.time() >= 0.1 && tomp.time() <= 0.3){
                    armie.setTargetPosition(-320);
                } else if (tomp.time() >= 0.6 && tomp.time() <= 0.8){
                    fingies.setPosition(0.25);
                } else {
                    armie.set(0);
                }

                if(armie.atTargetPosition()){
                    armie.stopMotor();
                }
            }
        }
    }
}