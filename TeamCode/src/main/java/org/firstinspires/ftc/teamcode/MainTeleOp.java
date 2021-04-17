package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.TimedAction;

        /**
 * This sample shows how to use dead wheels with external encoders
 * paired with motors that don't require encoders.
 * In this sample, we will use the drive motors' encoder
 * ports as they are not needed due to not using the drive encoders.
 * The external encoders we are using are REV through-bore.
         * x = 55in
 */
@TeleOp(name = "John Wilkes Booth")
public class MainTeleOp extends LinearOpMode {

    // The lateral distance between the left and right odometers
    // is called the trackwidth. This is very important for
    // determining angle for turning approximations
    public static final double TRACKWIDTH = 8.8;

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // This is to correct for the error that might occur when turning.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = 8.8;

    public static final double WHEEL_DIAMETER = 1.42;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    //private UGContourRingPipeline pipeline;
    //private OpenCvCamera camera;

    //private int cameraMonitorViewId;

    private Motor frontLeft, frontRight, backLeft, backRight, shooterF, shooterB, intake, secondaryIntake;
    private SimpleServo kicker, wobbleFingers, wobbleArmL, wobbleArmR;
    private TimedAction flicker;
    private RevIMU imu;
    GamepadEx gPadA, gPadB;
    private MecanumDrive driveTrain;
    private Motor.Encoder leftOdometer, rightOdometer, centerOdometer;
    private OdometrySubsystem odometry;
    private ToggleButtonReader AbuttonReaderY, AbuttonReaderA, AbuttonReaderX, AbuttonReaderB, AbuttonReaderdPadUp, AButtonReaderdPadDown, AButtonReaderdPadRight, AButtonReaderdPadLeft;
    private ToggleButtonReader BbuttonReaderY, BbuttonReaderA, BbuttonReaderX, BbuttonReaderB, BbuttonReaderdPadUp, BButtonReaderdPadDown;

    private ElapsedTime time;

    private ButtonReader AflickerBumper;
    private ButtonReader BflickerBumper;
    private VoltageSensor voltageSensor;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = new Motor(hardwareMap, "fL");
        frontRight = new Motor(hardwareMap, "fR");
        backLeft = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");

        intake = new Motor(hardwareMap, "intake");
        secondaryIntake = new Motor(hardwareMap, "secondaryIntake");
        shooterF = new Motor(hardwareMap, "shooterF");
        shooterB = new Motor(hardwareMap, "shooterB");

        kicker = new SimpleServo(hardwareMap, "kicker", 0, 270);
        wobbleFingers = new SimpleServo(hardwareMap, "wobbleFingers", 0, 270);
        wobbleArmL = new SimpleServo(hardwareMap, "wobbleArmL", 0, 270);
        wobbleArmR = new SimpleServo(hardwareMap, "wobbleArmR", 0, 270);

        driveTrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
//        imu = new RevIMU(hardwareMap);

        gPadA = new GamepadEx(gamepad1);
        gPadB = new GamepadEx(gamepad2);

        time = new ElapsedTime();

        AbuttonReaderY = new ToggleButtonReader(gPadA, GamepadKeys.Button.Y);
        BbuttonReaderY = new ToggleButtonReader(gPadB, GamepadKeys.Button.Y);

        AbuttonReaderA = new ToggleButtonReader(gPadA, GamepadKeys.Button.A);
        BbuttonReaderA = new ToggleButtonReader(gPadB, GamepadKeys.Button.A);

        AbuttonReaderX = new ToggleButtonReader(gPadA, GamepadKeys.Button.X);
        BbuttonReaderX = new ToggleButtonReader(gPadB, GamepadKeys.Button.X);

        AbuttonReaderB = new ToggleButtonReader(gPadA, GamepadKeys.Button.B);
        BbuttonReaderB = new ToggleButtonReader(gPadB, GamepadKeys.Button.B);

        AflickerBumper = new ButtonReader(gPadA, GamepadKeys.Button.LEFT_BUMPER);
        BflickerBumper = new ButtonReader(gPadB, GamepadKeys.Button.LEFT_BUMPER);

        AbuttonReaderdPadUp = new ToggleButtonReader(gPadA, GamepadKeys.Button.DPAD_UP);
        BbuttonReaderdPadUp = new ToggleButtonReader(gPadB, GamepadKeys.Button.DPAD_UP);
        AButtonReaderdPadDown = new ToggleButtonReader(gPadA, GamepadKeys.Button.DPAD_DOWN);
        BButtonReaderdPadDown = new ToggleButtonReader(gPadB, GamepadKeys.Button.DPAD_DOWN);
        AButtonReaderdPadRight = new ToggleButtonReader(gPadA, GamepadKeys.Button.DPAD_RIGHT);
        AButtonReaderdPadLeft = new ToggleButtonReader(gPadA, GamepadKeys.Button.DPAD_LEFT);



        voltageSensor = hardwareMap.voltageSensor.iterator().next();


        // Here we set the distance per pulse of the odometers.
        // This is to keep the units consistent for the odometry.
//        leftOdometer = frontLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
//        rightOdometer = frontRight.encoder.setDistancePerPulse(-DISTANCE_PER_PULSE);
 //       centerOdometer = backLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        frontLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterB.setInverted(true);

        shooterF.resetEncoder();
        shooterB.resetEncoder();



        shooterF.setRunMode(Motor.RunMode.VelocityControl);
        shooterF.setVeloCoefficients(0.3,0,0);
        shooterF.setFeedforwardCoefficients(1, 1.305);
        shooterB.setRunMode(Motor.RunMode.VelocityControl);
        shooterB.setVeloCoefficients(0.3,0,0);
        shooterB.setFeedforwardCoefficients(1, 1.305);
        
/*1`
        odometry = new OdometrySubsystem(new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        ));

        imu.init();
    */

        flicker = new TimedAction(
                () -> kicker.setPosition(0.6),
                () -> kicker.setPosition(0.2),
                120,
                true
        );



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
        camera.openCameraDevice();

*/

        waitForStart();
        time.reset();

        double x = 1;
        boolean awooga = true;
        boolean movingBack = false;

        while (opModeIsActive() && !isStopRequested()) {

            if(gamepad1.right_bumper){
                x = 0.33;
            } else {
                x = 1;
            }


            if ((AflickerBumper.isDown() && !flicker.running()) || (BflickerBumper.isDown() && !flicker.running())) {
                flicker.reset();
            }
            flicker.run();

            driveTrain.driveRobotCentric(-gPadA.getLeftX() * x, -gPadA.getLeftY() * x, -gPadA.getRightX() * x);

            double voltage = voltageSensor.getVoltage();
            double shooterSpeed = Math.sqrt(12.35/voltage);


            AbuttonReaderY.readValue();
            BbuttonReaderY.readValue();

            AbuttonReaderA.readValue();
            BbuttonReaderA.readValue();

            AbuttonReaderX.readValue();
            BbuttonReaderX.readValue();

            AbuttonReaderB.readValue();
            BbuttonReaderB.readValue();

            AbuttonReaderdPadUp.readValue();
            BbuttonReaderdPadUp.readValue();

            AButtonReaderdPadDown.readValue();
            BButtonReaderdPadDown.readValue();

            AButtonReaderdPadLeft.readValue();
            AButtonReaderdPadRight.readValue();

            if(AbuttonReaderY.getState() || BbuttonReaderY.getState()){
                shooterF.set(0.9 * shooterSpeed);
                shooterB.set(0.9 * shooterSpeed);
            }else if(AbuttonReaderA.getState() || BbuttonReaderA.getState()){
                shooterF.set(0.68 * shooterSpeed);
                shooterB.set(0.68 * shooterSpeed);
            }else{
                shooterF.set(0);
                shooterB.set(0);
            }

            if(AbuttonReaderdPadUp.getState() || BbuttonReaderdPadUp.getState()){
                wobbleFingers.setPosition(0.71);
            } else {
                wobbleFingers.setPosition(0.25);
            }

            if(AButtonReaderdPadRight.wasJustPressed()){
                wobbleArmL.setPosition(0);
                wobbleArmR.setPosition(0);
            } else if (AButtonReaderdPadLeft.wasJustPressed()){
                wobbleArmL.setPosition(300);
                wobbleArmR.setPosition(300);
            }

/*
            if(BButtonReaderdPadDown.isDown()){
                wobbleArm.set(-0.5);
            } else {
                wobbleArm.set(0);
            }

 */


            if(gamepad1.right_trigger >= 0.15|| gamepad2.right_trigger >= 0.15){
                kicker.setPosition(0.6); //270
                Thread.sleep(250);
                kicker.setPosition(0.1);
            } else if (gamepad1.left_trigger >= 0.15 || gamepad2.left_trigger >= 0.15){
                kicker.setPosition(-1);
            }


            if(AbuttonReaderX.getState() || BbuttonReaderX.getState()){
                intake.set(1);
                secondaryIntake.set(1);
            } else if (AbuttonReaderB.getState() || BbuttonReaderB.getState()) {
                intake.set(-1);
                secondaryIntake.set(-1);
            } else {
                intake.set(0);
                secondaryIntake.set(0);
            }

            //telemetry.addData("Angle: ", kicker.getAngle());
            telemetry.addData("wobbleArmL pos", wobbleArmL.getPosition());
            telemetry.addData("wobbleArmR pos", wobbleArmR.getPosition());
            telemetry.addData("Wobble fingies pos", wobbleFingers.getPosition());
            telemetry.update();
            //telemetry.addData("Stack Height", pipeline.getHeight());
            /*
            telemetry.addData("x", odometry.getPose().getX());
            telemetry.addData("y", odometry.getPose().getY());
            telemetry.addData("heading", odometry.getPose().getRotation().getDegrees());
            telemetry.addData("Angle: ", kicker.getAngle());
            odometry.update();
            telemetry.update();

             */
        }
    }
}