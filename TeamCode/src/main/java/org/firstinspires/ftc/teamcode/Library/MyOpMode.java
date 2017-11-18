package org.firstinspires.ftc.teamcode.Library;

import android.util.Log;

//import com.qualcomm.hardware.adafruit.BNO055IMU;
//import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMUCalibration;

import java.util.Locale;

/**
 * Created by jspspike on 1/15/2016.
 */

public abstract class MyOpMode extends LinearOpMode {

//    public static boolean fail;
//
//    public static final int MOVEMENT_DELAY = 300;
//
//
//
//    public static final double BUTTONP_CENTER = .47;
//    public static final double BUTTONP_LEFT = 1;
//    public static final double BUTTONP_RIGHT = .31;

    private ElapsedTime     runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 2240 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double     DRIVE_SPEED             = .65;
    public static final double     TURN_SPEED              = .5;

    public static DcMotor motorBL;
    public static DcMotor motorBR;
    public static DcMotor motorFL;
    public static DcMotor motorFR;

    public static Servo jewelHand;
    public static Servo jewelArm;

    ColorSensor jewelColor;
    public BNO055IMU imu;

//    public static DcMotor liftLeft;
//    public static DcMotor liftRight;

    //public static DcMotor relic;

    //public static DcMotor manip;


//    public static Servo jewelArm;
//    public static Servo jewelHand;
//    public static Servo relicGrabber;

//    public static OpticalDistanceSensor floorL;
//    public static OpticalDistanceSensor floorR;

//    public static BNO055IMU gyro;
//    public static BNO055IMU.Parameters gyroParam;
//    public static SensorREVColorDistance jewelColor;

//   private static ModernRoboticsI2cRangeSensor rangeR;
//    private static ModernRoboticsI2cRangeSensor rangeL;
//    private static ModernRoboticsI2cRangeSensor ultra;


    public Orientation angles;
//    public Acceleration gravity;
    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

//    public double grayL;
//    public double grayR;
//    public double turn;
//    public double gyroError = 0;
//    public double ultraDistance;
    public double rangeDistance;

      public void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            gravity  = imu.getGravity();
        }
        });
//
//        telemetry.addLine()
//                .addData("status", new Func<String>() {
//                    @Override public String value() {
//                        return imu.getSystemStatus().toShortString();
//                    }
//                })
//                .addData("calib", new Func<String>() {
//                    @Override public String value() {
//                        return imu.getCalibrationStatus().toString();
//                    }
//                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle); //Control Robot Pivot
                    }
                });
//                .addData("roll", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.secondAngle);
//                    }
//                })
//                .addData("pitch", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.thirdAngle);
//                    }
//                });
//
//        telemetry.addLine()
//                .addData("gravtiy", new Func<String>() {
//                    @Override public String value() {
//                        return gravity.toString();
//                    }
//                })
//                .addData("mag", new Func<String>() {
//                    @Override public String value() {
//                        return String.format(Locale.getDefault(), "%.3f",
//                                Math.sqrt(gravity.xAccel*gravity.xAccel
//                                        + gravity.yAccel*gravity.yAccel
//                                        + gravity.zAccel*gravity.zAccel));
//                    }
//                });
    }

//    public void hardwareMap() {
//        motorBL = hardwareMap.dcMotor.get("motorBL");
//        motorBR = hardwareMap.dcMotor.get("motorBR");
//        motorFL = hardwareMap.dcMotor.get("motorFL");
//        motorFR = hardwareMap.dcMotor.get("motorFR");


//        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
//        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
//        jewelColor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "jewelColor");
//
//
//        floorL = hardwareMap.get(OpticalDistanceSensor.class, "floorL");
//        floorR = hardwareMap.get(OpticalDistanceSensor.class, "floorR");
//
//
//        liftLeft = hardwareMap.dcMotor.get("liftLeft");
//        liftRight = hardwareMap.dcMotor.get("liftRight");
//
//        relic = hardwareMap.dcMotor.get("relic");
//
//        manip = hardwareMap.dcMotor.get("manip");
//
//
//        jewelArm = hardwareMap.servo.get("jewelArm");
//        jewelHand = hardwareMap.servo.get("jewelHand");
//        relicGrabber = hardwareMap.servo.get("relicGrabber");
//
//        telemetry.addData("Status", "Hardware Mapped");
//        telemetry.update();
//    }

    public void initSensors() {
        telemetry.addData("Sensors", "Initializing...");
        telemetry.update();



        //grayL = floorL.getRawLightDetected();
        //grayR = floorR.getRawLightDetected();

//        Log.w("grayL", "" + grayL);
//        Log.w("grayR", "" + grayR);

//        ultraDistance = -1;

//        gyroParam = new BNO055IMU.Parameters();
//        gyroParam.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        gyroParam.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        gyroParam.calibrationDataFile = "AdafruitIMUCalibration.json";
//        gyroParam.loggingEnabled = true;
//        gyroParam.loggingTag = "Gryo";
//        gyroParam.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


//        gyro.initialize(gyroParam);

        telemetry.addData("Sensors", "Initialized");
        telemetry.update();
    }
    // Add all servos in later

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void initServos() {

    }

    public void delay(long milliseconds) throws InterruptedException {
        if (milliseconds < 0)
            milliseconds = 0;

        Thread.sleep(milliseconds);
    }

    public void setMotors(double left, double right) {
        if (!opModeIsActive())
            return;

        motorFL.setPower(-left);
        motorBL.setPower(-left);
        motorFR.setPower(right);
        motorBR.setPower(right);
    }

//    public void setMotorsMec() {
//        if (!opModeIsActive())
//            return;
//
//        if (gamepad1.left_trigger > .1) {
//            motorFL.setPower(-gamepad1.left_trigger);
//            motorBL.setPower(gamepad1.left_trigger);
//            motorFR.setPower(gamepad1.left_trigger);
//            motorBR.setPower(-gamepad1.left_trigger);
//        }
//        else if (gamepad1.right_trigger > .1){
//            motorFL.setPower(gamepad1.right_trigger);
//            motorBL.setPower(-gamepad1.right_trigger);
//            motorFR.setPower(-gamepad1.right_trigger);
//            motorBR.setPower(gamepad1.right_trigger);
//        }
//        else {
//            motorFL.setPower(0);
//            motorBL.setPower(0);
//            motorFR.setPower(0);
//            motorBR.setPower(0);
//        }
//    }

    // code for strafing using the dpad on driver 1 controller


    //Why do we have this method if we do the same thing in TeleOp ????
//    public void setMotorsMecDPAD(double left, double right, double lessenPower, double increasePower) {
//        if (!opModeIsActive())
//            return;
//        double less = lessenPower;
//        double increase = increasePower;
//
//        if (gamepad1.dpad_up &&  left <= .75 && right <= .75) {
//            left += increasePower;
//            right += increasePower;
//        }
//
//        else if (gamepad1.dpad_down && left >= 0.25 && right >= 0.25){
//            left += lessenPower;
//            right += lessenPower;
//
//        }
//
//
//        if (gamepad1.dpad_left) {
//            motorFL.setPower(left);
//            motorBL.setPower(-left);
//            motorFR.setPower(-right);
//            motorBR.setPower(right);
//        }
//        else if (gamepad1.dpad_right){
//            motorFL.setPower(-left);
//            motorBL.setPower(left);
//            motorFR.setPower(right);
//            motorBR.setPower(-right);
//        }
//        else {
//            motorFL.setPower(0);
//            motorBL.setPower(0);
//            motorFR.setPower(0);
//            motorBR.setPower(0);
//        }
//
//    }


    public void slowDown (double reduction){
        //first reduction making power 0.5
        while (motorFL.getPower() > 0.05 && motorBL.getPower() > 0.05 && motorFR.getPower() > 0.05 && motorFL.getPower() > 0.05) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            motorFL.setPower(motorFL.getPower() * reduction);
            motorBL.setPower(motorBL.getPower() * reduction);
            motorFR.setPower(motorFL.getPower() * reduction);
            motorBR.setPower(motorBL.getPower() * reduction);
        }

        if(motorFL.getPower() < 0.05 && motorBL.getPower() < 0.05 && motorFR.getPower() < 0.05 && motorFL.getPower() < 0.05){
            motorFL.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(0);
            motorBR.setPower(0);
        }

    }


//    public void mecAutoLeft(double left, double right, double distance ,int tim ) {
//        if (!opModeIsActive())
//            return;
//        ElapsedTime time = new ElapsedTime();
//
//        time.reset();
//        resetStartTime();
//
//
//        if (distance > 0 && getRangeDistanceRight() < distance && time.milliseconds() < tim) {
//            motorFL.setPower(-left);
//            motorBL.setPower(left);
//            motorFR.setPower(right);
//            motorBR.setPower(-right);
//        }
//        else {
//            motorFL.setPower(0);
//            motorBL.setPower(0);
//            motorFR.setPower(0);
//            motorBR.setPower(0);
//        }
//    }

//    public void mecAutoRight(double left, double right, double distance ,int tim) {
//        if (!opModeIsActive())
//            return;
//
//        ElapsedTime time = new ElapsedTime();
//
//        time.reset();
//        resetStartTime();
//
//        if (distance > 0 && getRangeDistanceRight() < distance && time.milliseconds() < tim) {
//            motorFL.setPower(left);
//            motorBL.setPower(-left);
//            motorFR.setPower(-right);
//            motorBR.setPower(right);
//        }
//        else {
//            motorFL.setPower(0);
//            motorBL.setPower(0);
//            motorFR.setPower(0);
//            motorBR.setPower(0);
//        }
//    }

//    public double getRangeDistanceRight() {
//        double value = rangeR.getDistance(DistanceUnit.CM);
//        if (value != 255)
//            rangeDistance = value;
//        return rangeDistance;
//    }

//    public void depositBlockAuto(double dep) {
//        if (dep > 0.1) {
//            manip.setPower(dep);
//        } else if (dep < -0.1) {
//            manip.setPower(-dep);
//        } else {
//            manip.setPower(0);
//        }
//        manip.setPower(0);
//
//    }
//
   public void jewelKnockerRed() {
       jewelArm.setPosition(.6);
       jewelHand.setPosition(.45);
       sleep(2000);
       jewelArm.setPosition(.15);
       sleep(2000);

       if (jewelColor.red() > jewelColor.blue()) {
           jewelHand.setPosition((.3));

       } else if (jewelColor.red() < jewelColor.blue()) {
           jewelHand.setPosition((.6));
       }

       sleep(1000);
       jewelArm.setPosition(.6);
       jewelHand.setPosition(.45);
       sleep(3000);

       jewelHand.setPosition(.3);
       sleep(1000);
   }
        // int colorDif = jewelColor.red() - jewelColor.blue();

    public void jewelKnockerBlue() {
        jewelArm.setPosition(.6);
        jewelHand.setPosition(.45);
        sleep(2000);
        jewelArm.setPosition(.15);
        sleep(2000);

        if (jewelColor.red() < jewelColor.blue()) {
            jewelHand.setPosition((.3));

        } else if (jewelColor.red() > jewelColor.blue()) {
            jewelHand.setPosition((.6));
        }

        sleep(1000);
        jewelArm.setPosition(.6);
        jewelHand.setPosition(.45);
        sleep(1000);

        jewelHand.setPosition(.3);
        sleep(1000);
    }
//    public void gyrostab() {
//        String startPos = formatAngle(angles.angleUnit, angles.firstAngle); //heading - -numerical value or it = null
//        while (formatAngle(angles.angleUnit, angles.firstAngle) > startPos)
//    }



    public void stopMotors() {
        if (!opModeIsActive())
            return;

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }

    public void resetEncoders() {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorFL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = motorFR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);


            motorFL.setTargetPosition(newLeftTarget);
            motorFR.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorFL.setPower(speed);
            motorBL.setPower(speed);
            motorBR.setPower(speed);
            motorFR.setPower(speed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFL.isBusy()  && motorFR.isBusy())) {
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorFL.getCurrentPosition(),
                        motorFR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorFL.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(0);
            motorBR.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

              sleep(250);   // optional pause after each move
        }
    }
    public void encoderDriveMec(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorFL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = motorFR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            motorBL.setTargetPosition(newLeftTarget);
            motorFL.setTargetPosition(newLeftTarget);
            motorBR.setTargetPosition(newRightTarget);
            motorFR.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            if (speed < 0){
                motorFL.setPower(speed);
                motorBL.setPower(-speed);
                motorBR.setPower(-speed);
                motorFR.setPower(speed);
            } else if (speed > 0){
                motorFL.setPower(-speed);
                motorBL.setPower(speed);
                motorBR.setPower(speed);
                motorFR.setPower(-speed);
            }



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFL.isBusy() && motorBL.isBusy() && motorBR.isBusy() && motorFR.isBusy())) {
            }

            // Stop all motion;
            motorFL.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(0);
            motorBR.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public int getEncoderAverage() {
        return Math.abs(motorBL.getCurrentPosition());
    }

//    public void resetGyro() {
//        turn = gyro.getAngularOrientation().firstAngle;
//    }

    public int encoderPow() {
        double startingVoltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();

        if (startingVoltage > 13.9) {
            return 4755;
        }

        if (startingVoltage > 13.8) {
            return 4740;
        } else if (startingVoltage > 13.5) {
            return 4735;
        }

        return 4730;

    }

//    public double getGyroYaw() {
//
//        double turnAbs = Math.abs(turn);
//        Orientation angles = gyro.getAngularOrientation();
//        if (turnAbs > 270 && Math.abs(angles.firstAngle) < 90)
//            return (Math.abs(angles.firstAngle) - (turnAbs - 360));
//        else if (turnAbs < 90 && Math.abs(angles.firstAngle) > 270)
//            return ((Math.abs(angles.firstAngle) - 360) - turnAbs);
//        return (Math.abs(angles.firstAngle) - turnAbs);
//    }
//
//    public double getGryoPitch() {
//        Orientation angles = gyro.getAngularOrientation();
//        return angles.secondAngle;
//    }
//
//    public double getGyroRoll() {
//        Orientation angles = gyro.getAngularOrientation();
//        return angles.thirdAngle;
//    }

//    public double getUltraDistance() {
//        double value = ultra.cmUltrasonic();
//        if (value != 255)
//            ultraDistance = value;
//        return ultraDistance;
//    }
//
//    public void setManip(double pow) {
//        if (!opModeIsActive())
//            return;
//
//        manip.setPower(pow);
//    }


    public void setServoSlow(Servo servo, double pos) throws InterruptedException {
        double currentPosition = servo.getPosition();

        if (currentPosition - pos > 0) {
            for (; currentPosition > pos; currentPosition -= .005) {
                servo.setPosition(currentPosition);
                delay(1);
                idle();
            }
        } else for (; currentPosition < pos; currentPosition += .005) {
            servo.setPosition(currentPosition);
            delay(1);
            idle();
        }
    }

//    public void turnPID(double pow, double deg) throws InterruptedException {
//        turnPID(pow, deg, 5000);
//    }

//    public void turnPID(double pow, double deg, int tim) throws InterruptedException {
//        if (!opModeIsActive())
//            return;
//
//        delay(MOVEMENT_DELAY);
//        resetGyro();
//
//        double inte = 0;
//        double power;
//        double der;
//        double error;
//        double previousError = deg - getGyroYaw();
//
//        ElapsedTime time = new ElapsedTime();
//
//        time.reset();
//        resetStartTime();
//
//        do {
//            error = deg - getGyroYaw();
//            power = pow * error * .0222;
//            inte = inte + (getRuntime() * error * .02);
//            der = (error - previousError) / getRuntime() * .02;
//
//            power += inte + der;
//
//            if (power > 1)
//                power = 1;
//            else if (power < -1) {
//                power = -1;
//            }
//
//            setMotors(power, -power);
//            resetStartTime();
//            previousError = error;
//            idle();
//
//        } while (Math.abs(power) > .15 && time.milliseconds() < tim);
//
//        stopMotors();
//    }

//    public void moveToRange(double pow, double deg, int cm) throws InterruptedException {
//        moveToRange(pow, deg, cm, 1.5);
//    }
//
//    public void moveToRange(double pow, double deg, int cm, double threshold) throws InterruptedException {
//        moveToRange(pow, deg, cm, threshold, 4.0);
//    }
//
//    public void moveToRange(double pow, double deg, int cm, double threshold, double red) throws InterruptedException {
//        moveToRange(pow, deg, cm, threshold, red, 15000);
//    }
//
//    public void moveToRange(double pow, double deg, int cm, double threshold, double red, int tim) throws InterruptedException {
//
//        if (!opModeIsActive())
//            return;
//
//        ElapsedTime time = new ElapsedTime();
//        resetGyro();
//        resetEncoders();
//        delay(MOVEMENT_DELAY);
//
//        time.reset();
//
//        if (deg > 0) {
//            while (deg > getEncoderAverage() && time.milliseconds() < tim) {
//                if (getUltraDistance() < cm)
//                    setMotors(pow / (red * .5), pow);
//                else if (getUltraDistance() > cm)
//                    setMotors(pow, pow / (red * .5));
//
//                else {
//                    if (getGyroYaw() > threshold)
//                        setMotors(pow / red, pow);
//                    else if (getGyroYaw() < -threshold)
//                        setMotors(pow, pow / red);
//                    else
//                        setMotors(pow, pow);
//                }
//                telemetry.addData("Gryo", getGyroYaw());
//                telemetry.addData("Ultra", getUltraDistance());
//                telemetry.update();
//                idle();
//            }
//        } else {
//            while (Math.abs(deg) > getEncoderAverage() && time.milliseconds() < tim) {
//                if (getUltraDistance() < cm)
//                    setMotors(pow / (red * 1.5), pow);
//                else if (getUltraDistance() > cm)
//                    setMotors(pow, pow / (red * 1.5));
//                else {
//                    if (getGyroYaw() > threshold)
//                        setMotors(pow, pow / red);
//                    else if (getGyroYaw() < -threshold)
//                        setMotors(pow / red, pow);
//                    else
//                        setMotors(pow, pow);
//                }
//                telemetry.addData("Gryo", getGyroYaw());
//                telemetry.addData("Ultra", getUltraDistance());
//                telemetry.update();
//                idle();
//            }
//        }
//
//        stopMotors();
//    }

//    public void moveTo(double pow, double deg) throws InterruptedException {
//        moveTo(pow, deg, .6);
//    }
//
//    public void moveTo(double pow, double deg, double threshold) throws InterruptedException {
//        moveTo(pow, deg, threshold, 2.2);
//    }
//
//    public void moveTo(double pow, double deg, double threshold, double red) throws InterruptedException {
//        moveTo(pow, deg, threshold, red, 15000, true);
//    }
//
//    public void moveTo(double pow, double deg, double threshold, double red, int tim, boolean stop) throws InterruptedException {
//
//        if (!opModeIsActive())
//            return;
//
//
//        ElapsedTime time = new ElapsedTime();
//
//
//        resetGyro();
//        resetEncoders();
//        delay(MOVEMENT_DELAY);
//
//        time.reset();
//
//        If user wants robot to move forward (1st if), (else): the user wants to move backwards
//        While the opMode is running, and the robot still has to move, 
//        and it has not passed the timeout seconds
//        If the gyro is not correct, then turn the robot until gyro value is within the amount of degrees it is allowed off perfect


//        if (deg > 0) {
//            while (opModeIsActive() && deg > getEncoderAverage() && time.milliseconds() < tim) {
//                if (getGyroYaw() + gyroError > threshold)
//                    setMotors(pow / red, pow);
//                else if (getGyroYaw() + gyroError < -threshold)
//                    setMotors(pow, pow / red);
//                else
//                    setMotors(pow, pow);
////                telemetry.addData("Gyro", getGyroYaw());
////                telemetry.addData("Gyro Error", gyroError);
////                telemetry.addData("Encoder", getEncoderAverage());
////                telemetry.update();
////                Log.w("Gyro", "" + getGyroYaw());
//                idle();
//            }
//        } else {


//            while (opModeIsActive() && Math.abs(deg) > getEncoderAverage() && time.milliseconds() < tim) {
//                if (getGyroYaw() + gyroError > threshold)
//                    setMotors(-pow, -pow / red);
//                else if (getGyroYaw() + gyroError < -threshold)
//                    setMotors(-pow / red, -pow);
//                else
//                    setMotors(-pow, -pow);
//
////                telemetry.addData("Gyro", getGyroYaw());
////                telemetry.addData("Gyro Error", gyroError);
////                telemetry.addData("Encoder", getEncoderAverage());
////                telemetry.update();
////                Log.w("Gyro", "" + getGyroYaw());
//                idle();
//            }
//        }
//        if (stop)
//            stopMotors();
//
//        gyroError = getGyroYaw() + gyroError;
//    }
//
//
//    public void turnCorr(double pow, double deg) throws InterruptedException {
//        turnCorr(pow, deg, 8000);
//    }

    public void turnCorr(double pow, double deg, int tim) throws InterruptedException {
        if (!opModeIsActive())
            return;

        double newPow;

        ElapsedTime time = new ElapsedTime();

        float startP = angles.firstAngle; // startP is the current position

//        delay(100);
        time.reset();

        if (deg > 0) {
            while ((deg > startP && time.milliseconds() < tim)) {
                newPow = pow * (Math.abs(deg - startP) / 80);

                if (newPow < .15)
                    newPow = .15;

//                setMotors(-newPow, newPow);
                telemetry.addData("Gyro", startP);
                telemetry.addData("newpower", newPow);
                telemetry.update();
                idle();
            }
        } else {
            while (deg < startP && time.milliseconds() < tim) {
                newPow = pow * (Math.abs(deg - startP) / 80);

                if (newPow < .15)
                    newPow = .15;
//                setMotors(newPow, -newPow);
                telemetry.addData("Gyro", startP);
                telemetry.addData("newpower", newPow);
                telemetry.update();
                idle();
            }
        }

//        stopMotors();

        if (startP > deg) {
            while (opModeIsActive() && deg < startP ) {
                newPow = pow * (Math.abs(deg - startP) / 80);

                if (newPow < .15)
                    newPow = .15;
//                setMotors(.15, -.15);
                telemetry.addData("Gyro", startP);
                telemetry.addData("newpower", newPow);
                telemetry.update();
                idle();
            }
        } else {
            while (opModeIsActive() && deg > startP) {
                newPow = pow * (Math.abs(deg - startP) / 80);

                if (newPow < .15)
                    newPow = .15;

//                setMotors(-.15, .15);
                telemetry.addData("Gyro", startP);
                telemetry.addData("newpower", newPow);
                telemetry.update();
                idle();
            }
        }
        stopMotors();
    }

//    public void arcTurnCorr(double pow, double deg) throws InterruptedException {
//        arcTurnCorr(pow, deg, 6000);
//    }
//
//    public void arcTurnCorr(double pow, double deg, int tim) throws InterruptedException {
//
//        if (!opModeIsActive())
//            return;
//
//        double newPow;
//
//        ElapsedTime time = new ElapsedTime();
//
//        resetGyro();
//        delay(MOVEMENT_DELAY);
//        time.reset();
//
//        if (deg > 0) {
//            while (opModeIsActive() && deg > getGyroYaw() + gyroError && time.milliseconds() < tim) {
//                newPow = Math.abs(pow) * (Math.abs(deg - getGyroYaw()) / 70);
//
//                if (newPow < .15)
//                    newPow = .15;
//
//                if (pow > 0)
//                    setMotors(newPow, 0);
//                else
//                    setMotors(0, -newPow);
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.update();
//                idle();
//            }
//        } else {
//            while (opModeIsActive() && deg < getGyroYaw() + gyroError && time.milliseconds() < tim) {
//                newPow = Math.abs(pow) * (Math.abs(deg - getGyroYaw()) / 70);
//
//                if (newPow < .15)
//                    newPow = .15;
//
//                if (pow > 0)
//                    setMotors(0, newPow);
//                else
//                    setMotors(-newPow, 0);
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.update();
//                idle();
//            }
//        }
//
//        stopMotors();
//
//        if (getGyroYaw() > deg) {
//            while (opModeIsActive() && deg < getGyroYaw() + gyroError) {
//                if (pow > 0)
//                    setMotors(-.14, 0);
//                else
//                    setMotors(0, .14);
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.update();
//                idle();
//            }
//        } else {
//            while (opModeIsActive() && deg > getGyroYaw() + gyroError) {
//                if (pow > 0)
//                    setMotors(0, -.14);
//                else
//                    setMotors(.14, 0);
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.update();
//                idle();
//            }
//        }
//        stopMotors();
//        delay(100);
//
//        gyroError = getGyroYaw() + gyroError - deg;
//    }
//
//
//
//    public void arcTurn(double pow, double deg) throws InterruptedException {
//        arcTurn(pow, deg, true);
//    }
//    public void arcTurn(double pow, double deg, boolean stop) throws InterruptedException {
//        arcTurn(pow, deg, stop, 6000);
//    }
//
//    public void  arcTurn(double pow, double deg, boolean stop, int tim) throws InterruptedException {
//
//        if (!opModeIsActive())
//            return;
//
//        double newPow;
//
//        ElapsedTime time = new ElapsedTime();
//
//        resetGyro();
//        delay(MOVEMENT_DELAY);
//        time.reset();
//
//        if (deg + gyroError > 0) {
//            while (opModeIsActive() && deg > getGyroYaw() + gyroError && time.milliseconds() < tim) {
//                newPow = Math.abs(pow) * (Math.abs(deg - getGyroYaw()) / 70);
//
//                if (newPow < .15)
//                    newPow = .15;
//
//                if (pow > 0)
//                    setMotors(newPow, 0);
//                else
//                    setMotors(0, -newPow);
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.update();
//                idle();
//            }
//        } else {
//            while (opModeIsActive() && deg < getGyroYaw() + gyroError && time.milliseconds() < tim) {
//                newPow = Math.abs(pow) * (Math.abs(deg - getGyroYaw()) / 70);
//
//
//                if (newPow < .15)
//                    newPow = .15;
//
//                if (pow > 0)
//                    setMotors(0, newPow);
//                else
//                    setMotors(-newPow, 0);
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.update();
//                idle();
//            }
//        }
//
//        if (stop)
//            stopMotors();
//        delay(100);
//        gyroError = getGyroYaw() + gyroError - deg;
//    }
//
//    public void arcTurnPID(double pow, double deg) throws InterruptedException {
//        arcTurnPID(pow, deg, 6000);
//    }
//
//    public void arcTurnPID(double pow, double deg, int tim) throws InterruptedException {
//
//        if (!opModeIsActive())
//            return;
//
//        double newPow;
//        double prop;
//        double inte = 0;
//        double deriv;
//        double error;
//        double lastError;
//
//        ElapsedTime time = new ElapsedTime();
//
//        resetGyro();
//        delay(MOVEMENT_DELAY);
//        time.reset();
//
//        if (deg + gyroError > 0) {
//            lastError = deg - getGyroYaw();
//            while (opModeIsActive() && deg - 2.1 > getGyroYaw() + gyroError && time.milliseconds() < tim) {
//
//                error = deg - getGyroYaw();
//
//                prop = (error * Math.abs(pow)) * .02455 + .015;
//                inte = inte + (getRuntime() * error * .000013);
//                deriv = ((error - lastError) / getRuntime()) * .000006;
//
//                newPow = prop + inte /* +  deriv*/;
//
//
//                if (pow > 0)
//                    setMotors(newPow, 0);
//                else
//                    setMotors(0, -newPow);
//
////                telemetry.addData("Gyro", getGyroYaw());
////                telemetry.addData("NewPow", newPow);
////                telemetry.update();
////                Log.i("Error", "" + error);
////                Log.i("Prop", "" + prop);
////                Log.i("Inte", "" + inte);
////                Log.i("Total", "" + newPow);
//                resetStartTime();
//                lastError = error;
//                idle();
//            }
//        } else {
//            lastError = (deg - getGyroYaw()) * -1;
//            while (opModeIsActive() && deg + 2.1 < getGyroYaw() + gyroError && time.milliseconds() < tim) {
//
//                error = (deg - getGyroYaw()) * -1;
//
//                prop = (error * Math.abs(pow)) * .02455 + .015;
//                inte = inte + (getRuntime() * error * .000013);
//                deriv = ((error - lastError) / getRuntime()) * .000006;
//
//                newPow = prop  +  inte /* + deriv*/;
//
//                if (pow > 0)
//                    setMotors(0, newPow);
//                else
//                    setMotors(-newPow, 0);
//
////            telemetry.addData("Gyro", getGyroYaw());
////            telemetry.addData("NewPow", newPow);
////            telemetry.update();
////            Log.i("Error", ""  + error);
////            Log.i("Prop", ""  + prop);
////            Log.i("Inte", ""  + inte);
////            Log.i("Total", ""  + newPow);
//                lastError = error;
//                resetStartTime();
//                idle();
//            }
//        }
//
//        stopMotors();
//
//        delay(100);
//        gyroError = getGyroYaw() + gyroError - deg;
//    }
//
//    public void arcTurnPIDHy(double pow, double deg) throws InterruptedException {
//        arcTurnPIDHy(pow, deg, 6000);
//    }
//
//    public void arcTurnPIDHy(double pow, double deg, int tim) throws InterruptedException {
//
//        if (!opModeIsActive())
//            return;
//
//        double newPow;
//        double prop;
//        double inte = 0;
//        double deriv;
//        double error;
//        double lastError;
//
//        ElapsedTime time = new ElapsedTime();
//
//        resetGyro();
//        delay(MOVEMENT_DELAY);
//        time.reset();
//
//        if (deg + gyroError > 0) {
//            lastError = deg - getGyroYaw();
//            while (opModeIsActive() && deg - 2.1 > getGyroYaw() + gyroError && time.milliseconds() < tim) {
//
//                error = deg - getGyroYaw();
//
//                prop = (error * Math.abs(pow)) * .02455 + .015;
//                inte = inte + (getRuntime() * error * .000013);
//                deriv = ((error - lastError) / getRuntime()) * .000006;
//
//                newPow = prop + inte /* +  deriv*/;
//
//
//                if (pow > 0)
//                    setMotors(newPow, newPow / -1.8);
//                else
//                    setMotors(newPow / 1.8, -newPow);
//
////                telemetry.addData("Gyro", getGyroYaw());
////                telemetry.addData("NewPow", newPow);
////                telemetry.update();
////                Log.i("Error", "" + error);
////                Log.i("Prop", "" + prop);
////                Log.i("Inte", "" + inte);
////                Log.i("Total", "" + newPow);
//                resetStartTime();
//                lastError = error;
//                idle();
//            }
//        } else {
//            lastError = (deg - getGyroYaw()) * -1;
//            while (opModeIsActive() && deg + 2.1 < getGyroYaw() + gyroError && time.milliseconds() < tim) {
//
//                error = (deg - getGyroYaw()) * -1;
//
//                prop = (error * Math.abs(pow)) * .02455 + .015;
//                inte = inte + (getRuntime() * error * .000013);
//                deriv = ((error - lastError) / getRuntime()) * .000006;
//
//                newPow = prop  +  inte /* + deriv*/;
//
//                if (pow > 0)
//                    setMotors(newPow / -1.8, newPow);
//                else
//                    setMotors(-newPow, newPow / 1.8);
//
////            telemetry.addData("Gyro", getGyroYaw());
////            telemetry.addData("NewPow", newPow);
////            telemetry.update();
////            Log.i("Error", ""  + error);
////            Log.i("Prop", ""  + prop);
////            Log.i("Inte", ""  + inte);
////            Log.i("Total", ""  + newPow);
//                lastError = error;
//                resetStartTime();
//                idle();
//            }
//        }
//
//        stopMotors();
//
//        delay(100);
//        gyroError = getGyroYaw() + gyroError - deg;
//    }
//
//    public void turn(double pow, double deg) throws InterruptedException {
//        turn(pow, deg, 6000);
//    }
//
//    public void turn(double pow, double deg, int tim) throws InterruptedException {
//
//        if (!opModeIsActive())
//            return;
//
//        double newPow;
//        double prop;
//        double inte = 0;
//        double deriv;
//        double error;
//        double lastError;
//
//        ElapsedTime time = new ElapsedTime();
//
//        resetGyro();
//        delay(MOVEMENT_DELAY);
//        time.reset();
//
//        if (deg + gyroError > 0) {
//            lastError = deg - getGyroYaw();
//            while (opModeIsActive() && deg - 2.1 > getGyroYaw() + gyroError && time.milliseconds() < tim) {
//
//                error = deg - getGyroYaw();
//
//                prop = (error * Math.abs(pow)) * .02455 + .015;
//                inte = inte + (getRuntime() * error * .000013);
//                deriv = ((error - lastError) / getRuntime()) * .000006;
//
//                newPow = prop + inte /* +  deriv*/;
//
//
//                if (pow > 0)
//                    setMotors(newPow, 0);
//                else
//                    setMotors(0, -newPow);
//
////                telemetry.addData("Gyro", getGyroYaw());
////                telemetry.addData("NewPow", newPow);
////                telemetry.update();
////                Log.i("Error", "" + error);
////                Log.i("Prop", "" + prop);
////                Log.i("Inte", "" + inte);
////                Log.i("Total", "" + newPow);
//                resetStartTime();
//                lastError = error;
//                idle();
//            }
//        } else {
//            lastError = (deg - getGyroYaw()) * -1;
//            while (opModeIsActive() && deg + 2.1 < getGyroYaw() + gyroError && time.milliseconds() < tim) {
//
//                error = (deg - getGyroYaw()) * -1;
//
//                prop = (error * Math.abs(pow)) * .02455 + .015;
//                inte = inte + (getRuntime() * error * .000013);
//                deriv = ((error - lastError) / getRuntime()) * .000006;
//
//                newPow = prop  +  inte /* + deriv*/;
//
//                if (pow > 0)
//                    setMotors(0, newPow);
//                else
//                    setMotors(-newPow, 0);
//
////            telemetry.addData("Gyro", getGyroYaw());
////            telemetry.addData("NewPow", newPow);
////            telemetry.update();
////            Log.i("Error", ""  + error);
////            Log.i("Prop", ""  + prop);
////            Log.i("Inte", ""  + inte);
////            Log.i("Total", ""  + newPow);
//                lastError = error;
//                resetStartTime();
//                idle();
//            }
//        }
//
//        stopMotors();
//
//        delay(100);
//        gyroError = getGyroYaw() + gyroError - deg;
//    }
//
//    public void untilWhiteRange(double pow, double powWhite, double cm, double deg, int degFail) throws InterruptedException {
//        untilWhiteRange(pow, powWhite, cm, deg, degFail, .6, 1.5, 7000);
//    }
//
//    public void untilWhiteRange(double pow, double powWhite, double cm, double deg, int degFail, double threshold, double reduction, int tim) throws InterruptedException {
//
//        if (!opModeIsActive())
//            return;
//
//        fail = false;
//
//        resetEncoders();
//        resetGyro();
////        grayL = floorL.getRawLightDetected();
////        grayR = floorR.getRawLightDetected();
//        delay(MOVEMENT_DELAY);
//
//        ElapsedTime time = new ElapsedTime();
//        time.reset();
//
//        if (deg > 0) {
//            while (opModeIsActive() && deg > getEncoderAverage() && time.milliseconds() < tim) {
//
//                if (getUltraDistance() > cm && getGyroYaw() < 3) {
//                    setMotors(pow, pow / (reduction));
//                } else if (getUltraDistance() < cm && getGyroYaw() > -3) {
//                    setMotors(pow / (reduction), pow);
//                } else {
//
//                    if (getGyroYaw() + gyroError > threshold)
//                        setMotors(pow / (reduction), pow);
//                    else if (getGyroYaw() + gyroError < -threshold)
//                        setMotors(pow, pow / (reduction));
//                    else
//                        setMotors(pow, pow);
//                    telemetry.addData("Gyro", getGyroYaw());
//                    telemetry.addData("Gyro Error", gyroError);
//                    telemetry.addData("Encoder", getEncoderAverage());
//                    telemetry.addData("Ultra", getUltraDistance());
//                    telemetry.update();
//                    Log.w("Gyro", "" + getGyroYaw());
//                    idle();
//                }
//            }
//        } else {
//            while (opModeIsActive() && Math.abs(deg) > getEncoderAverage() && time.milliseconds() < tim) {
//
//                if (getUltraDistance() > cm && getGyroYaw() > -3) {
//                    setMotors(pow, pow / (reduction));
//                } else if (getUltraDistance() < cm && getGyroYaw() < 3) {
//                    setMotors(pow / (reduction), pow);
//                } else {
//                    if (getGyroYaw() + gyroError > threshold)
//                        setMotors(pow, pow / (reduction));
//                    else if (getGyroYaw() + gyroError < -threshold)
//                        setMotors(pow / (reduction), pow);
//                    else
//                        setMotors(pow, pow);
//                    telemetry.addData("Gyro", getGyroYaw());
//                    telemetry.addData("Gyro Error", gyroError);
//                    telemetry.addData("Encoder", getEncoderAverage());
//                    telemetry.addData("Ultra", getUltraDistance());
//                    telemetry.update();
//                    Log.w("Gyro", "" + getGyroYaw());
//                    idle();
//                }
//            }
//        }
//
//        if (pow > 0) {
//            while (opModeIsActive() && (floorL.getRawLightDetected() < grayL + .5 && floorR.getRawLightDetected() < grayR + .5) && time.milliseconds() < tim) {
//                if (getUltraDistance() > cm && getGyroYaw() < 3) {
//                    setMotors(powWhite, powWhite / (reduction + .05));
//                } else if (getUltraDistance() < cm && getGyroYaw() > -3) {
//                    setMotors(powWhite / (reduction + .05), powWhite);
//                } else {
//
//                    if (getGyroYaw() + gyroError > threshold)
//                        setMotors(powWhite / reduction, powWhite);
//                    else if (getGyroYaw() + gyroError < -threshold)
//                        setMotors(powWhite, powWhite / reduction);
//                    else
//                        setMotors(powWhite, powWhite);
//                }
//
//                if (Math.abs(degFail) < getEncoderAverage()) {
//                    untilWhiteRange(-.15, -.15, 14, 0, 3000);
//                    moveTo(.2, 90, .6, 1.5);
//                    fail = true;
//                    break;
//                }
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.addData("FloorL", floorL.getRawLightDetected());
//                telemetry.addData("FloorR", floorR.getRawLightDetected());
//                telemetry.addData("Ultra", getUltraDistance());
//                Log.w("FloorL", "" + floorL.getRawLightDetected());
//                Log.w("FloorR", "" + floorR.getRawLightDetected());
//                telemetry.update();
//                idle();
//            }
//        } else {
//            while (opModeIsActive() && (floorL.getRawLightDetected() < grayL + .5 && floorR.getRawLightDetected() < grayR + .5) && time.milliseconds() < tim) {
//                if (getUltraDistance() > cm && getGyroYaw() > -3) {
//                    setMotors(powWhite, powWhite / (reduction + .05));
//                } else if (getUltraDistance() < cm && getGyroYaw() < 3) {
//                    setMotors(powWhite / (reduction + .05), powWhite);
//                } else {
//                    if (getGyroYaw() + gyroError > threshold) {
//                        setMotors(powWhite, powWhite / reduction);
//                    } else if (getGyroYaw() + gyroError < -threshold) {
//                        setMotors(powWhite / reduction, powWhite);
//                    } else {
//                        setMotors(powWhite, powWhite);
//                    }
//                }
//
//                if (Math.abs(degFail) < getEncoderAverage()) {
//                    untilWhiteRange(.15, .15, 14, 0, 3000);
//                    moveTo(.2, -100, .6, 1.5);
//                    fail = true;
//                    break;
//                }
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.addData("FloorL", floorL.getRawLightDetected());
//                telemetry.addData("FloorR", floorR.getRawLightDetected());
//                telemetry.addData("Ultra", getUltraDistance());
//                Log.w("FloorL", "" + floorL.getRawLightDetected());
//                Log.w("FloorR", "" + floorR.getRawLightDetected());
//                telemetry.update();
//                idle();
//            }
//        }
//
//        gyroError = getGyroYaw() + gyroError;
//        stopMotors();
//    }
//
//    public void untilWhite(double pow, double powWhite) throws InterruptedException {
//        untilWhite(pow, powWhite, 0, 10000);
//    }
//
//    public void untilWhite(double pow, double powWhite, int deg, int degFail) throws InterruptedException {
//        untilWhite(pow, powWhite, deg, degFail, .6, 1.75, 7000);
//    }
//
//    public void untilWhite(double pow, double powWhite, int deg, int degFail, double threshold, double reduction, int tim) throws InterruptedException {
//
//        if (!opModeIsActive())
//            return;
//
//        fail = false;
//
//        resetEncoders();
//        resetGyro();
////        grayL = floorL.getRawLightDetected();
////        grayR = floorR.getRawLightDetected();
//        delay(MOVEMENT_DELAY);
//
//        ElapsedTime time = new ElapsedTime();
//        time.reset();
//
//        if (deg > 0) {
//            while (opModeIsActive() && deg > getEncoderAverage() && time.milliseconds() < tim) {
//
//                if (getGyroYaw() + gyroError > threshold)
//                    setMotors(pow / reduction, pow);
//                else if (getGyroYaw() + gyroError < -threshold)
//                    setMotors(pow, pow / reduction);
//                else
//                    setMotors(pow, pow);
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.addData("Encoder", getEncoderAverage());
//                telemetry.update();
//                Log.w("Gyro", "" + getGyroYaw());
//                idle();
//            }
//        } else {
//            while (opModeIsActive() && Math.abs(deg) > getEncoderAverage() && time.milliseconds() < tim) {
//
//                if (getGyroYaw() + gyroError > threshold)
//                    setMotors(pow, pow / reduction);
//                else if (getGyroYaw() + gyroError < -threshold)
//                    setMotors(pow / reduction, pow);
//                else
//                    setMotors(pow, pow);
//
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.addData("Encoder", getEncoderAverage());
//                telemetry.update();
//                Log.w("Gyro", "" + getGyroYaw());
//                idle();
//            }
//        }
//
//        if (pow > 0) {
//            while (opModeIsActive() && (floorL.getRawLightDetected() < grayL + .5 && floorR.getRawLightDetected() < grayR + .5) && time.milliseconds() < tim) {
//
//                if (getGyroYaw() + gyroError > threshold)
//                    setMotors(powWhite / reduction, powWhite);
//                else if (getGyroYaw() + gyroError < -threshold)
//                    setMotors(powWhite, powWhite / reduction);
//                else
//                    setMotors(powWhite, powWhite);
//
//                if (Math.abs(degFail) < getEncoderAverage()) {
//                    untilWhite(-.15, -.15, 0, 3000);
//                    moveTo(.2, 100, .6, 1.5);
//                    fail = true;
//                    break;
//                }
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.addData("FloorL", floorL.getRawLightDetected());
//                telemetry.addData("FloorR", floorR.getRawLightDetected());
//                Log.w("FloorL", "" + floorL.getRawLightDetected());
//                Log.w("FloorR", "" + floorR.getRawLightDetected());
//                telemetry.update();
//                idle();
//            }
//        } else {
//            while (opModeIsActive() && (floorL.getRawLightDetected() < grayL + .5 && floorR.getRawLightDetected() < grayR + .5) && time.milliseconds() < tim) {
//
//                if (getGyroYaw() + gyroError > threshold) {
//                    setMotors(powWhite, powWhite / reduction);
//                } else if (getGyroYaw() + gyroError < -threshold) {
//                    setMotors(powWhite / reduction, powWhite);
//                } else {
//                    setMotors(powWhite, powWhite);
//                }
//
//                if (Math.abs(degFail) < getEncoderAverage()) {
//                    untilWhite(.15, .15, 0, 3000);
//                    moveTo(.2, -100, .6, 1.5);
//                    fail = true;
//                    break;
//                }
//
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.addData("FloorL", floorL.getRawLightDetected());
//                telemetry.addData("FloorR", floorR.getRawLightDetected());
//                Log.w("FloorL", "" + floorL.getRawLightDetected());
//                Log.w("FloorR", "" + floorR.getRawLightDetected());
//                telemetry.update();
//                idle();
//            }
//        }
//
//        gyroError = getGyroYaw() + gyroError;
//        stopMotors();
//    }
//
//    public void whiteTurn(double pow, int turns) {
//        int count = 0;
//
//        while (count < turns) {
//            while (floorR.getRawLightDetected() > grayR - .4) {
//                setMotors(0, pow);
//            }
//
//            count++;
//            if (floorL.getRawLightDetected() < grayL - .4 && floorR.getRawLightDetected() < grayR - .4)
//                break;
//
//            while (floorL.getRawLightDetected() > grayL - .4) {
//                setMotors(-pow, 0);
//            }
//
//            count++;
//
//            if (floorL.getRawLightDetected() < grayL - .4 && floorR.getRawLightDetected() < grayR - .4)
//                break;
//        }
//    }
//
//
//
//
//    public void untilWhiteAlign(double pow, double powWhite) throws InterruptedException {
//        untilWhiteAlign(pow, powWhite, 0, 10000);
//    }
//
//    public void untilWhiteAlign(double pow, double powWhite, int deg, int degFail) throws InterruptedException {
//        untilWhiteAlign(pow, powWhite, deg, degFail, .65 , 7000, true);
//    }
//
//    public void untilWhiteAlign(double pow, double powWhite, int deg, int degFail, double reduction, int tim, boolean failSafe) throws InterruptedException {
//
//        if (!opModeIsActive())
//            return;
//
//        fail = false;
//
//        resetEncoders();
//        resetGyro();
////        grayL = floorL.getRawLightDetected();
////        grayR = floorR.getRawLightDetected();
//        delay(MOVEMENT_DELAY);
//
//        ElapsedTime time = new ElapsedTime();
//        time.reset();
//
//        double prevEncoder = 0;
//        ElapsedTime time2 = new ElapsedTime();
//        time2.reset();
//
//        if (pow > 0) {
//            while (opModeIsActive() && deg > getEncoderAverage() && time.milliseconds() < tim) {
//                setMotors(pow, pow * reduction);
//                if (time2.seconds() > .25) {
//                    if (getEncoderAverage() - prevEncoder < 100 && failSafe) {
//                        arcTurnPID(-.65, -20, 1000);
//                        tim += 1000;
//                    }
//                    time2.reset();
//                    prevEncoder = getEncoderAverage();
//                }
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.addData("Encoder", getEncoderAverage());
//                telemetry.update();
//                Log.w("Gyro", "" + getGyroYaw());
//                idle();
//            }
//        } else {
//            while (opModeIsActive() && Math.abs(deg) > getEncoderAverage() && time.milliseconds() < tim) {
//                setMotors(pow, pow * reduction);
//                if (time2.seconds() > .25) {
//                    if (getEncoderAverage() - prevEncoder < 100 && failSafe) {
//                        arcTurnPID(.65, 20, 1000);
//                        tim += 1000;
//                    }
//                    time2.reset();
//                    prevEncoder = getEncoderAverage();
//                }
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.addData("Encoder", getEncoderAverage());
//                telemetry.update();
//                Log.w("Gyro", "" + getGyroYaw());
//                idle();
//            }
//        }
//
//        if (pow > 0) {
//            while (opModeIsActive() && (floorL.getRawLightDetected() < grayL + .5 && floorR.getRawLightDetected() < grayR + .5) && time.milliseconds() < tim) {
//
//                setMotors(powWhite, powWhite * reduction);
//
//                if (Math.abs(degFail) < getEncoderAverage()) {
//                    untilWhiteAlign(-.15, -.15, 0, 2300, .65, 3000, false);
//                    moveTo(.2, 100, .6, 1.5);
//                    fail = true;
//                    break;
//                }
//
//                if (time2.seconds() > .25) {
//                    if (getEncoderAverage() - prevEncoder < 100 && failSafe) {
//                        arcTurnPID(-.65, -15, 1000);
//                        tim += 1;
//                    }
//                    time2.reset();
//                    prevEncoder = getEncoderAverage();
//                }
////                telemetry.addData("Gyro", getGyroYaw());
////                telemetry.addData("Gyro Error", gyroError);
////                telemetry.addData("FloorL", floorL.getRawLightDetected());
////                telemetry.addData("FloorR", floorR.getRawLightDetected());
////                telemetry.addData("Encoder", getEncoderAverage());
////                Log.w("FloorL", "" + floorL.getRawLightDetected());
////                Log.w("FloorR", "" + floorR.getRawLightDetected());
////                telemetry.update();
//                idle();
//            }
//        } else {
//            while (opModeIsActive() && (floorL.getRawLightDetected() < grayL + .5 && floorR.getRawLightDetected() < grayR + .5) && time.milliseconds() < tim) {
//
//                setMotors(powWhite, powWhite * reduction);
//
//                if (Math.abs(degFail) < getEncoderAverage()) {
//                    untilWhiteAlign(.15, .15, 60, 2300, .65, 3000, false);
//                    moveTo(.2, -150, .6, 1.5);
//                    fail = true;
//                    break;
//                }
//
//                if (time2.seconds() > .25) {
//                    if (getEncoderAverage() - prevEncoder < 100  && failSafe) {
//                        arcTurnPID(.65, 15, 1000);
//                        tim += 1;
//                    }
//                    time2.reset();
//                    prevEncoder = getEncoderAverage();
//                }
//
////                telemetry.addData("Gyro", getGyroYaw());
////                telemetry.addData("Gyro Error", gyroError);
////                telemetry.addData("FloorL", floorL.getRawLightDetected());
////                telemetry.addData("FloorR", floorR.getRawLightDetected());
////                telemetry.addData("Encoder", getEncoderAverage());
////                Log.w("FloorL", "" + floorL.getRawLightDetected());
////                Log.w("FloorR", "" + floorR.getRawLightDetected());
////                telemetry.update();
//                idle();
//            }
//        }
//
//        gyroError = getGyroYaw() + gyroError;
//        stopMotors();
//    }
//
//    public void untilRange(double pow) throws InterruptedException {
//        untilRange(pow, 30, .5, 2.2, 7000);
//    }
//
//    public void untilRange(double pow, double endDistance, double threshold, double reduction, int tim) throws InterruptedException {
//
//        if (!opModeIsActive())
//            return;
//
//        resetEncoders();
//        delay(1000);
//
//        ElapsedTime time = new ElapsedTime();
//        time.reset();
//
//        if (pow > 0) {
//            while (opModeIsActive() && (getUltraDistance() < endDistance) && time.milliseconds() < tim) {
//                if (getGyroYaw() > threshold)
//                    setMotors(pow / reduction, pow);
//                else if (getGyroYaw() < -threshold)
//                    setMotors(pow, pow / reduction);
//                else
//                    setMotors(pow, pow);
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Ultrasonic", getUltraDistance());
//                Log.w("Ultrasonic", "" + getUltraDistance());
//                telemetry.update();
//                idle();
//            }
//        } else {
//            while (opModeIsActive() && (getUltraDistance() > endDistance) && time.milliseconds() < tim) {
//                if (getGyroYaw() > threshold) {
//                    setMotors(pow, pow / reduction);
//                } else if (getGyroYaw() < -threshold) {
//                    setMotors(pow / reduction, pow);
//                } else {
//                    setMotors(pow, pow);
//                }
//
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Ultrasonic", getUltraDistance());
//                Log.w("Ultrasonic", "" + getUltraDistance());
//                telemetry.update();
//                idle();
//            }
//        }
//        stopMotors();
//    }
//
//    public void moveParallel(double pow) throws InterruptedException {
//        if (!opModeIsActive())
//            return;
//
//        resetEncoders();
//        delay(MOVEMENT_DELAY);
//
//        while (getGyroYaw() > 3) {
//            setMotors(pow, pow);
//        }
//
//        stopMotors();
//    }
//
//
//
//    public void moveAlign(double pow, double ramPow, double dis, double failDis) throws InterruptedException {
//
//        if (!opModeIsActive())
//            return;
//
//        resetEncoders();
//        delay(MOVEMENT_DELAY);
//
//        if (pow > 0) {
//            while(getEncoderAverage() < dis) {
//                setMotors(pow, pow);
//            }
//
//            while (opModeIsActive() && getGyroYaw() > 6.5 && failDis > getEncoderAverage()) {
//                setMotors(ramPow, ramPow);
//                telemetry.addData("Gryo", getGyroYaw());
//                telemetry.update();
//            }
//
//        } else {
//            while(getEncoderAverage() < dis) {
//                setMotors(pow, pow);
//            }
//
//            while (opModeIsActive() && getGyroYaw() - 180 < -6.5 && failDis > getEncoderAverage()) {
//                setMotors(ramPow, ramPow);
//            }
//        }
//
//        stopMotors();
//    }
//    public void moveToSlow(double pow, double deg) throws InterruptedException {
//        moveTo(pow, deg, .6);
//    }
//
//    public void moveToSlow(double pow, double deg, double threshold) throws InterruptedException {
//        moveToSlow(pow, deg, threshold, 2.2);
//    }
//
//    public void moveToSlow(double pow, double deg, double threshold, double red) throws InterruptedException {
//        moveToSlow(pow, deg, threshold, red, 15000, true);
//    }
//
//    public void moveToSlow(double pow, double deg, double threshold, double red, int tim, boolean stop) throws InterruptedException {
//
//        if (!opModeIsActive())
//            return;
//
//        ElapsedTime time = new ElapsedTime();
//
//
//        resetGyro();
//        resetEncoders();
//        delay(MOVEMENT_DELAY);
//
//        time.reset();
//
//        if (pow > 0) {
//            while (opModeIsActive() && deg > getEncoderAverage() && time.milliseconds() < tim) {
//                double power = (((pow-.2)*((deg - getEncoderAverage())/deg))+.2);
//                if (getGyroYaw() + gyroError > threshold)
//                    setMotors(power / red, power);
//                else if (getGyroYaw() + gyroError < -threshold)
//                    setMotors(power, power / red);
//                else
//                    setMotors(power, power);
////                telemetry.addData("Gyro", getGyroYaw());
////                telemetry.addData("Gyro Error", gyroError);
////                telemetry.addData("Power", power);
////                telemetry.addData("Encoder", getEncoderAverage());
////                telemetry.update();
////                Log.w("Gyro", "" + getGyroYaw());
//                idle();
//            }
//        } else {
//            while (opModeIsActive() && deg > getEncoderAverage() && time.milliseconds() < tim) {
//                double power = (((pow+.2)*((deg - getEncoderAverage())/deg))-.2);
//                if (getGyroYaw() + gyroError > threshold)
//                    setMotors(power, power / red);
//                else if (getGyroYaw() + gyroError < -threshold)
//                    setMotors(power / red, power);
//                else
//                    setMotors(power, power);
//
////                telemetry.addData("Gyro", getGyroYaw());
////                telemetry.addData("Gyro Error", gyroError);
////                telemetry.addData("Power", power);
////                telemetry.addData("Encoder", getEncoderAverage());
////                telemetry.update();
////                Log.w("Gyro", "" + getGyroYaw());
//                idle();
//            }
//        }
//        if (stop)
//            stopMotors();
//
//        gyroError = getGyroYaw() + gyroError;
//    }
//
//
}