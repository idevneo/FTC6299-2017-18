package org.firstinspires.ftc.teamcode.Library;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


<<<<<<< HEAD
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
=======
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

>>>>>>> refs/remotes/origin/master

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


<<<<<<< HEAD
    public static boolean fail;

    public static final int MOVEMENT_DELAY = 300;



    public static final double BUTTONP_CENTER = .47;
    public static final double BUTTONP_LEFT = 1;
    public static final double BUTTONP_RIGHT = .31;



=======
import java.util.Locale;
>>>>>>> refs/remotes/origin/master

public abstract class MyOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    public static BNO055IMU imu;
    public static DcMotor motorBL;
    public static DcMotor motorBR;
    public static DcMotor motorFL;
    public static DcMotor motorFR;
<<<<<<< HEAD

    public static DcMotor liftLeft;
    public static DcMotor liftRight;

    public static DcMotor relic;

    public static DcMotor manip;


    public static Servo jewelArm;
    public static Servo jewelHand;
    public static Servo relicGrabber;

    public static OpticalDistanceSensor floorL;
    public static OpticalDistanceSensor floorR;



    public static BNO055IMU gyro;
    public static BNO055IMU.Parameters gyroParam;
    public static ModernRoboticsI2cColorSensor jewelColor;

    private static ModernRoboticsI2cRangeSensor range;
    private static ModernRoboticsI2cRangeSensor ultra;
=======
    public static DcMotor liftLeft;
    public static DcMotor liftRight;
    public static DcMotor manip;
    public static Servo jewelArm;
    public static Servo jewelHand;
    public static ColorSensor jewelColor;
    //public static DcMotor relic;
//    public static Servo relicGrabber;

    public static Orientation angles;
    public static Acceleration gravity;

    public static ModernRoboticsI2cRangeSensor rangeR;
    public static ModernRoboticsI2cRangeSensor rangeL;
    public static ModernRoboticsI2cRangeSensor rangeF;
>>>>>>> refs/remotes/origin/master

    public VuforiaLocalizer vuforia;
    public char column;

<<<<<<< HEAD


    public double grayL;
    public double grayR;
    public double turn;
    public double gyroError = 0;

    public double ultraDistance;
    public double rangeDistance;

    public void hardwareMap() {
=======
    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle); //Control Robot Pivot
                    }
                });
//        telemetry.addLine()
//                .addData("VumarkGoal", new Func<String>() {
//                    @Override public String  value() {
//                        return Character.toString(getVuMark());
//                    }
//                });
          telemetry.addLine()
                  .addData("Left", new Func<String>() {
                      @Override public String  value() {
                          double localRange;
                          double sensor = 0;
                          localRange = rangeL.getDistance(DistanceUnit.INCH);
                          if (!Double.isNaN(localRange) && (localRange < 1000)) {
                              sensor = localRange;
                          }
                          return Double.toString(sensor);
                      }
                  });
        telemetry.addLine()
                .addData("Right", new Func<String>() {
                    @Override public String  value() {
                        double localRange;
                        double sensor = 0;
                        localRange = rangeR.getDistance(DistanceUnit.INCH);
                        if (!Double.isNaN(localRange) && (localRange < 1000)) {
                            sensor = localRange;
                        }
                        return Double.toString(sensor);
                    }
                });

        telemetry.addLine()
                .addData("Front", new Func<String>() {
                    @Override public String  value() {
                        double localRange;
                        double sensor = 0;
                        localRange = rangeF.getDistance(DistanceUnit.INCH);
                        if (!Double.isNaN(localRange) && (localRange < 1000)) {
                            sensor = localRange;
                        }
                        return Double.toString(sensor);
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

    public void hMap(HardwareMap type) {
>>>>>>> refs/remotes/origin/master
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");

<<<<<<< HEAD

        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        jewelColor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "jewelColor");


        floorL = hardwareMap.get(OpticalDistanceSensor.class, "floorL");
        floorR = hardwareMap.get(OpticalDistanceSensor.class, "floorR");


        liftLeft = hardwareMap.dcMotor.get("liftLeft");
        liftRight = hardwareMap.dcMotor.get("liftRight");

        relic = hardwareMap.dcMotor.get("relic");

        manip = hardwareMap.dcMotor.get("manip");


=======
        liftLeft = hardwareMap.dcMotor.get("liftL");
        liftRight = hardwareMap.dcMotor.get("liftR");
        manip = hardwareMap.dcMotor.get("manip");

        jewelColor = hardwareMap.get(ColorSensor.class, "jewelColor");
>>>>>>> refs/remotes/origin/master
        jewelArm = hardwareMap.servo.get("jewelArm");
        jewelHand = hardwareMap.servo.get("jewelHand");
        relicGrabber = hardwareMap.servo.get("relicGrabber");

        telemetry.addData("Status", "Hardware Mapped");
        telemetry.update();
    }

<<<<<<< HEAD
    public void hardwareMapTroll() {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
=======
        rangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeR");
        rangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeL");
        rangeF = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeC");

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
//        relic = hardwareMap.dcMotor.get("relic");
//        relicGrabber = hardwareMap.servo.get("relicGrabber");
    }
>>>>>>> refs/remotes/origin/master

    public void delay(long milliseconds) throws InterruptedException {
        if (milliseconds < 0)
            milliseconds = 0;
        Thread.sleep(milliseconds);
    }

    public void setMotors(double left, double right) {
        if (!opModeIsActive())
            return;

<<<<<<< HEAD
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
=======
        motorFL.setPower(-left);
        motorBL.setPower(-left);
        motorFR.setPower(right);
        motorBR.setPower(right);
    }
>>>>>>> refs/remotes/origin/master

    public void setMotorStrafe(double pow) { //Strafes right when positive.
        motorFL.setPower(-pow);
        motorBL.setPower(pow);
        motorFR.setPower(-pow);
        motorBR.setPower(pow);

    }

    public void stopMotors() {
        if (!opModeIsActive())
            return;

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }

    public void manipAuto(double pow) {
        if (!opModeIsActive())
            return;
        if (pow > 0) {
            manip.setPower(pow);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            manip.setPower(0);
        } else if (pow < 0) {
            manip.setPower(pow);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            manip.setPower(0);
        } else {
            manip.setPower(0);
        }

    }
    public char getVuMark() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AXb/g5n/////AAAAGSUed2rh5Us1jESA1cUn5r5KDUqTfwO2woh7MxjiLKSUyDslqBAgwCi0Qmc6lVczErnF5TIw7vG5R4TJ2igvrDVp+dP+3i2o7UUCRRj/PtyVgb4ZfNrDzHE80/6TUHifpKu4QCM04eRWYZocWNWhuRfytVeWy6NSTWefM9xadqG8FFrFk3XnvqDvk/6ZAgerNBdq5SsJ90eDdoAhgYEee40WxasoUUM9YVMvkWOqZgHSuraV2IyIUjkW/u0O+EkFtTNRUWP+aZwn1qO1H4Lk07AJYe21eqioBLMdzY7A8YqR1TeQ//0WJg8SFdXjuGbF6uHykBe2FF5UeyaehA0iTqfPS+59FLm8y1TuUt57eImq";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

<<<<<<< HEAD
        Log.w("grayL", "" + grayL);
        Log.w("grayR", "" + grayR);

        ultraDistance = -1;

        gyroParam = new BNO055IMU.Parameters();
        gyroParam.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParam.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParam.calibrationDataFile = "AdafruitIMUCalibration.json";
        gyroParam.loggingEnabled = true;
        gyroParam.loggingTag = "Gryo";
        gyroParam.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
=======
        //telemetry.addData(">", "Press Play to start");

        relicTrackables.activate();

>>>>>>> refs/remotes/origin/master

        // copy pasta from the ftc ppl
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

<<<<<<< HEAD
        gyro.initialize(gyroParam);
=======
>>>>>>> refs/remotes/origin/master

        telemetry.addData("VuMark ", vuMark);
        telemetry.update();
        if (vuMark == RelicRecoveryVuMark.CENTER)
            column = 'C';
        else if (vuMark == RelicRecoveryVuMark.LEFT)
            column = 'L';
        else if (vuMark == RelicRecoveryVuMark.RIGHT)
            column = 'R';
        else if (vuMark == RelicRecoveryVuMark.UNKNOWN)
            column = 'U';
        return column;

    }

    public void vfMovePerp(char rb, ModernRoboticsI2cRangeSensor sensorVar) {
        double centerDis = 30;
        double kC = 0;
        if (rb == 'r')
            kC = 8.5;
        if (rb == 'b')
            kC = -8.5;
        if (column == 'L') {
            rangeMoveStrafe((centerDis + kC) , sensorVar);
        } else if (column == 'R') {
            rangeMoveStrafe((centerDis - kC), sensorVar);
        } else if (column == 'C') {
            rangeMoveStrafe(centerDis, sensorVar);
        } else if (column == 'U'){
            rangeMoveStrafe(centerDis, sensorVar);
        }

    }



//    public void rangeMove(double pow, double inAway, ModernRoboticsI2cRangeSensor sensorVar)    { //Set pow negative to move backward.
//        double sensor = sensorVar.getDistance(DistanceUnit.INCH);
//
//        while ((sensor < inAway - .25) || (sensor > inAway + .25)) { //While sensor doesn't = tolerance, run.
//            double localRange;
//            localRange = sensorVar.getDistance(DistanceUnit.INCH);
//            if (!Double.isNaN(localRange) && (localRange < 1000)) {
//                sensor = localRange;
//            }
//
//            telemetry.addData("SensorValue", sensor); //optional telemetry
//            telemetry.update();
//
//            if (sensor > inAway) {
//                setMotors(pow, pow);
//            }
//            if (sensor < inAway) {
//                setMotors(-pow, -pow);
//            }
//        }
//        stopMotors();
//    }

    public void rangeMovePID(double inAway, ModernRoboticsI2cRangeSensor sensorVar)    {
        double sensor = sensorVar.getDistance(DistanceUnit.INCH);
//        double differenceDis;
//        double kP = .025; //to be determined
        double pow;
        double localRange;
        while (((sensor < inAway - .25) || (sensor > inAway + .25)) && opModeIsActive()) { //While sensor doesn't = tolerance, run.
            localRange = sensorVar.getDistance(DistanceUnit.INCH);
            while (Double.isNaN(localRange) || (localRange > 1000)) {
                localRange = sensorVar.getDistance(DistanceUnit.INCH);
            }
            sensor = localRange;

//            differenceDis = Math.abs(sensor - inAway);
//            pow = differenceDis*kP;
            pow = .125;

//            if (pow > .2)
//                pow = .2;
//            if (pow < .1)
//                pow = .1;
            telemetry.addData("SensorValue", sensor); //optional telemetry
            telemetry.update();

            if (sensor > inAway) {
                setMotors(pow, pow);
            }
            if (sensor < inAway) {
                setMotors(-pow, -pow);
            }
        }
        stopMotors();
    }





    public void rangeMoveStrafe(double inAway , ModernRoboticsI2cRangeSensor sensorVar) { //Set pow to negative if we want to move left.
        double sensor = sensorVar.getDistance(DistanceUnit.INCH);
//        double differenceDis;
//        double kP = .025; //to be determined
        double pow;
        double localRange;
          while (((sensor < inAway - .5) || (sensor > inAway + .5))&& opModeIsActive()) {
              localRange = sensorVar.getDistance(DistanceUnit.INCH);
              while (Double.isNaN(localRange) || (localRange > 1000)) {
                  localRange = sensorVar.getDistance(DistanceUnit.INCH);
              }
              sensor = localRange;
              //differenceDis = Math.abs(sensor - inAway);
              //pow = differenceDis*kP;
              pow = 0.15;
//              if (pow > .2) edit once working on single power
//                  pow = .2;
//              if (pow < .19)
//                  pow = .19;

            if (sensor > inAway) {
                setMotorStrafe(pow);
            }
            if (sensor < inAway) {
                setMotorStrafe(-pow);

            }
              telemetry.addData("rightwhile", sensor);
              telemetry.update();
        }
        stopMotors();
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

    public void setMotorsMecDPAD(double left, double right, double lessenPower, double increasePower) {
        if (!opModeIsActive())
            return;
        double less = lessenPower;
        double increase = increasePower;

<<<<<<< HEAD
        if (gamepad1.dpad_up &&  left <= .75 && right <= .75) {
            left += increasePower;
            right += increasePower;
        }

        else if (gamepad1.dpad_down && left >= 0.25 && right >= 0.25){
            left += lessenPower;
            right += lessenPower;

        }


        if (gamepad1.dpad_left) {
            motorFL.setPower(-left);
            motorBL.setPower(left);
            motorFR.setPower(right);
            motorBR.setPower(-right);
        }
        else if (gamepad1.dpad_right){
            motorFL.setPower(left);
            motorBL.setPower(-left);
            motorFR.setPower(-right);
            motorBR.setPower(right);
        }
        else {
            motorFL.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(0);
            motorBR.setPower(0);
        }

    }
=======
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
>>>>>>> refs/remotes/origin/master

    public void slowDown(double reduction) {
        //first reduction making power 0.5
        while (motorFL.getPower() > 0.05 && motorBL.getPower() > 0.05 && motorFR.getPower() > 0.05 && motorFL.getPower() > 0.05) {
            try {
                wait(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            motorFL.setPower(motorFL.getPower() * reduction);
            motorBL.setPower(motorBL.getPower() * reduction);
            motorFR.setPower(motorFL.getPower() * reduction);
            motorBR.setPower(motorBL.getPower() * reduction);
        }

        if (motorFL.getPower() < 0.05 && motorBL.getPower() < 0.05 && motorFR.getPower() < 0.05 && motorFL.getPower() < 0.05) {
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

<<<<<<< HEAD
    public void mecAutoLeft(double left, double right, double distance ,int tim ) {
        if (!opModeIsActive())
            return;
        ElapsedTime time = new ElapsedTime();

        time.reset();
        resetStartTime();


        if (distance < 0 && getRangeDistance() < distance && time.milliseconds() < tim) {
            motorFL.setPower(-left);
            motorBL.setPower(left);
            motorFR.setPower(right);
            motorBR.setPower(-right);
        }
        else {
            motorFL.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(0);
            motorBR.setPower(0);
        }
    }
=======
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
>>>>>>> refs/remotes/origin/master

//    public double getRangeDistanceRight() {
//        double value = rangeR.getDistance(DistanceUnit.CM);
//        if (value != 255)
//            rangeDistance = value;
//        return rangeDistance;
//    }

    public void jewelKnockerRed() {
        jewelArm.setPosition(.6);
        jewelHand.setPosition(.45);
        sleep(2000);
        jewelArm.setPosition(.15);
        sleep(2000);

        if (jewelColor.red() > jewelColor.blue()) {
            jewelHand.setPosition((.3));

<<<<<<< HEAD
        if (distance < 0 && getRangeDistance() < distance && time.milliseconds() < tim) {
            motorFL.setPower(left);
            motorBL.setPower(-left);
            motorFR.setPower(-right);
            motorBR.setPower(right);
        }
        else {
            motorFL.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(0);
            motorBR.setPower(0);
=======
        } else if (jewelColor.red() < jewelColor.blue()) {
            jewelHand.setPosition((.6));
>>>>>>> refs/remotes/origin/master
        }

<<<<<<< HEAD
    public double getRangeDistance() {
        double value = range.getDistance(DistanceUnit.CM);
        if (value != 255)
            rangeDistance = value;
        return rangeDistance;
    }


    public void depositBlockAuto(double dep) {
        if (dep > 0.1) {
            manip.setPower(dep);
        } else if (dep < -0.1) {
            manip.setPower(-dep);
        } else {
            manip.setPower(0);
        }
        manip.setPower(0);

    }






    public void jewelKnockerRed(double servoArmD, double servoArmS, double servoHandL, double servoHandR, double servoHandS)
    {
        if(!opModeIsActive())
            return;

        jewelArm.setPosition(servoArmD);
        if (jewelColor.blue() < jewelColor.red())
        {
            jewelHand.setPosition((servoHandR));
        }
        else if (jewelColor.blue() > jewelColor.red())
        {
            jewelHand.setPosition((servoHandL));
        }

        jewelHand.setPosition(servoHandS);
        jewelArm.setPosition(servoArmS);
    }

    public void jewelKnockerBlue(double servoArmD, double servoArmS, double servoHandL, double servoHandR, double servoHandS)
    {
        if(!opModeIsActive())
            return;

        jewelArm.setPosition(servoArmD);
        if (jewelColor.blue() > jewelColor.red())
        {
            jewelHand.setPosition((servoHandR));
        }
        else if (jewelColor.blue() < jewelColor.red())
        {
            jewelHand.setPosition((servoHandL));
=======
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
>>>>>>> refs/remotes/origin/master
        }


        jewelHand.setPosition(servoHandS);
        jewelArm.setPosition(servoArmS);
    }

<<<<<<< HEAD



=======
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
>>>>>>> refs/remotes/origin/master

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

<<<<<<< HEAD
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

    public int getEncoderAverage() {
        return Math.abs(motorBL.getCurrentPosition());
    }

    public void resetGyro() {
        turn = gyro.getAngularOrientation().firstAngle;
    }

    public int encoderPow() {
        double startingVoltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();
=======
//    public void turnCorr(double pow, double deg) throws InterruptedException {
//        turnCorr(pow, deg, 8000);
//    }

    public double getDiff(double angle) {
        imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        return   currPos - angle;
    }
    public Double gyroVal() {
        imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        return currPos;
    }
    public void turnPID(double angle) throws InterruptedException {
        double kP = .20/90;
        double min = -.2;
        double max = .2;
        double changeCon = .04;
        double PIDchange;
        double angleDiff = getDiff(angle);
        double oldDiff = angleDiff;
        int counter = 0;
        double startAngle = gyroVal();
        while ((Math.abs(angleDiff) <= Math.abs(oldDiff)) && opModeIsActive()) {
            PIDchange = angleDiff * kP;

            if (PIDchange < 0) {
                motorFR.setPower(Range.clip(-PIDchange + changeCon, min, max));
                motorBR.setPower(Range.clip(-PIDchange + changeCon, min, max));
                motorFL.setPower(Range.clip(-PIDchange + changeCon, min, max));
                motorBL.setPower(Range.clip(-PIDchange + changeCon, min, max));
            }
            else {
                motorFR.setPower(Range.clip(PIDchange - changeCon, min, max));
                motorBR.setPower(Range.clip(PIDchange - changeCon, min, max));
                motorFL.setPower(Range.clip(PIDchange - changeCon, min, max));
                motorBL.setPower(Range.clip(PIDchange - changeCon, min, max));
            }

            sleep(250);
            telemetry.addData("gyroStart", startAngle);
            telemetry.addData("counter", counter++);
            telemetry.addData("GyroVal", gyroVal());
            telemetry.addData("GyroDiff",getDiff(angle));
            telemetry.addData("Pow", -PIDchange - changeCon);
            telemetry.update();

            oldDiff = angleDiff;
            angleDiff = getDiff(angle);
        }
        stopMotors();
    }

    public void turn(double pow, double deg) throws InterruptedException {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        if (deg > 0) {
            while ((currPos > (deg + .25)) || (currPos < (deg-.25))){
                if (currPos > (deg + .25))
                    setMotors(-pow, pow);
                if (currPos < (deg -.25))
                    setMotors(pow, -pow);

                currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                idle();
            }
            stopMotors();

        }
        if (deg < 0) {
            while ((currPos > (deg + .25)) || (currPos < (deg-.25))){
                if (currPos > (deg + .25))
                    setMotors(-pow, pow);
                if (currPos < (deg -.25))
                    setMotors(pow, -pow);
                currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                idle();
            }
            stopMotors();
        }
    }
    public void turnCorr(double pow, double deg, int tim) throws InterruptedException {
        if (!opModeIsActive())
            return;

        double newPow;
>>>>>>> refs/remotes/origin/master

        ElapsedTime time = new ElapsedTime();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)); // startP is the current position

        delay(100);
        time.reset();

        if (deg > 0) {
            while ((deg > currPos && time.milliseconds() < tim)) {
                newPow = pow * (Math.abs(deg - currPos) / 80);

                if (newPow < .15)
                    newPow = .1;

                setMotors(-newPow, newPow);
                currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.addData("Gyro", currPos);
                telemetry.addData("newpower", newPow);
                telemetry.update();
                idle();
            }
        } else {
            while (deg < currPos && time.milliseconds() < tim) {
                newPow = pow * (Math.abs(deg - currPos) / 80);

                if (newPow < .15)
                    newPow = .1;
                setMotors(newPow, -newPow);
                currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.addData("Gyro", currPos);
                telemetry.addData("newpower", newPow);
                telemetry.update();
                idle();
            }
        }

        stopMotors();

<<<<<<< HEAD
    public double getGyroYaw() {

        double turnAbs = Math.abs(turn);
        Orientation angles = gyro.getAngularOrientation();
        if (turnAbs > 270 && Math.abs(angles.firstAngle) < 90)
            return (Math.abs(angles.firstAngle) - (turnAbs - 360));
        else if (turnAbs < 90 && Math.abs(angles.firstAngle) > 270)
            return ((Math.abs(angles.firstAngle) - 360) - turnAbs);
        return (Math.abs(angles.firstAngle) - turnAbs);
    }

    public double getGryoPitch() {
        Orientation angles = gyro.getAngularOrientation();
        return angles.secondAngle;
    }

    public double getGyroRoll() {
        Orientation angles = gyro.getAngularOrientation();
        return angles.thirdAngle;
    }

    public double getUltraDistance() {
        double value = ultra.cmUltrasonic();
        if (value != 255)
            ultraDistance = value;
        return ultraDistance;
    }

    public void setManip(double pow) {
        if (!opModeIsActive())
            return;

        manip.setPower(pow);
    }
=======
        if (currPos > deg) {
            while (opModeIsActive() && deg < currPos) {
                newPow = pow * (Math.abs(deg - currPos) / 80);
>>>>>>> refs/remotes/origin/master

                if (newPow < .15)
                    newPow = .1;
                setMotors(newPow, -newPow);
                currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.addData("Gyro", currPos);
                telemetry.addData("newpower", newPow);
                telemetry.update();
                idle();
            }
        } else {
            while (opModeIsActive() && deg > currPos) {
                newPow = pow * (Math.abs(deg - currPos) / 80);

                if (newPow < .15)
                    newPow = .1;

                setMotors(-newPow, newPow);
                currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.addData("Gyro", currPos);
                telemetry.addData("newpower", newPow);
                telemetry.update();
                idle();
            }
        }
        stopMotors();
    }
<<<<<<< HEAD

    public void turnPID(double pow, double deg) throws InterruptedException {
        turnPID(pow, deg, 5000);
    }

    public void turnPID(double pow, double deg, int tim) throws InterruptedException {
        if (!opModeIsActive())
            return;

        delay(MOVEMENT_DELAY);
        resetGyro();

        double inte = 0;
        double power;
        double der;
        double error;
        double previousError = deg - getGyroYaw();

        ElapsedTime time = new ElapsedTime();

        time.reset();
        resetStartTime();

        do {
            error = deg - getGyroYaw();
            power = pow * error * .0222;
            inte = inte + (getRuntime() * error * .02);
            der = (error - previousError) / getRuntime() * .02;

            power += inte + der;

            if (power > 1)
                power = 1;
            else if (power < -1) {
                power = -1;
            }

            setMotors(power, -power);
            resetStartTime();
            previousError = error;
            idle();

        } while (Math.abs(power) > .15 && time.milliseconds() < tim);

        stopMotors();
    }

    public void moveToRange(double pow, double deg, int cm) throws InterruptedException {
        moveToRange(pow, deg, cm, 1.5);
    }

    public void moveToRange(double pow, double deg, int cm, double threshold) throws InterruptedException {
        moveToRange(pow, deg, cm, threshold, 4.0);
    }

    public void moveToRange(double pow, double deg, int cm, double threshold, double red) throws InterruptedException {
        moveToRange(pow, deg, cm, threshold, red, 15000);
    }

    public void moveToRange(double pow, double deg, int cm, double threshold, double red, int tim) throws InterruptedException {

        if (!opModeIsActive())
            return;

        ElapsedTime time = new ElapsedTime();
        resetGyro();
        resetEncoders();
        delay(MOVEMENT_DELAY);

        time.reset();

        if (deg > 0) {
            while (deg > getEncoderAverage() && time.milliseconds() < tim) {
                if (getUltraDistance() < cm)
                    setMotors(pow / (red * .5), pow);
                else if (getUltraDistance() > cm)
                    setMotors(pow, pow / (red * .5));

                else {
                    if (getGyroYaw() > threshold)
                        setMotors(pow / red, pow);
                    else if (getGyroYaw() < -threshold)
                        setMotors(pow, pow / red);
                    else
                        setMotors(pow, pow);
                }
                telemetry.addData("Gryo", getGyroYaw());
                telemetry.addData("Ultra", getUltraDistance());
                telemetry.update();
                idle();
            }
        } else {
            while (Math.abs(deg) > getEncoderAverage() && time.milliseconds() < tim) {
                if (getUltraDistance() < cm)
                    setMotors(pow / (red * 1.5), pow);
                else if (getUltraDistance() > cm)
                    setMotors(pow, pow / (red * 1.5));
                else {
                    if (getGyroYaw() > threshold)
                        setMotors(pow, pow / red);
                    else if (getGyroYaw() < -threshold)
                        setMotors(pow / red, pow);
                    else
                        setMotors(pow, pow);
                }
                telemetry.addData("Gryo", getGyroYaw());
                telemetry.addData("Ultra", getUltraDistance());
                telemetry.update();
                idle();
            }
        }

        stopMotors();
    }

    public void moveTo(double pow, double deg) throws InterruptedException {
        moveTo(pow, deg, .6);
    }

    public void moveTo(double pow, double deg, double threshold) throws InterruptedException {
        moveTo(pow, deg, threshold, 2.2);
    }

    public void moveTo(double pow, double deg, double threshold, double red) throws InterruptedException {
        moveTo(pow, deg, threshold, red, 15000, true);
    }

    public void moveTo(double pow, double deg, double threshold, double red, int tim, boolean stop) throws InterruptedException {

        if (!opModeIsActive())
            return;


        ElapsedTime time = new ElapsedTime();


        resetGyro();
        resetEncoders();
        delay(MOVEMENT_DELAY);

        time.reset();

        if (deg > 0) {
            while (opModeIsActive() && deg > getEncoderAverage() && time.milliseconds() < tim) {
                if (getGyroYaw() + gyroError > threshold)
                    setMotors(pow / red, pow);
                else if (getGyroYaw() + gyroError < -threshold)
                    setMotors(pow, pow / red);
                else
                    setMotors(pow, pow);
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.addData("Encoder", getEncoderAverage());
//                telemetry.update();
//                Log.w("Gyro", "" + getGyroYaw());
                idle();
            }
        } else {
            while (opModeIsActive() && Math.abs(deg) > getEncoderAverage() && time.milliseconds() < tim) {
                if (getGyroYaw() + gyroError > threshold)
                    setMotors(-pow, -pow / red);
                else if (getGyroYaw() + gyroError < -threshold)
                    setMotors(-pow / red, -pow);
                else
                    setMotors(-pow, -pow);

//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.addData("Encoder", getEncoderAverage());
//                telemetry.update();
//                Log.w("Gyro", "" + getGyroYaw());
                idle();
            }
        }
        if (stop)
            stopMotors();

        gyroError = getGyroYaw() + gyroError;
    }


    public void turnCorr(double pow, double deg) throws InterruptedException {
        turnCorr(pow, deg, 8000);
    }

    public void turnCorr(double pow, double deg, int tim) throws InterruptedException {
        if (!opModeIsActive())
            return;

        double newPow;

        ElapsedTime time = new ElapsedTime();

        resetGyro();
        delay(MOVEMENT_DELAY);
        time.reset();

        if (deg > 0) {
            while (deg > getGyroYaw() && time.milliseconds() < tim) {
                newPow = pow * (Math.abs(deg - getGyroYaw()) / 80);

                if (newPow < .15)
                    newPow = .15;

                setMotors(newPow, -newPow);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        } else {
            while (deg < getGyroYaw() && time.milliseconds() < tim) {
                newPow = pow * (Math.abs(deg - getGyroYaw()) / 80);

                if (newPow < .15)
                    newPow = .15;
                setMotors(-newPow, newPow);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        }

        stopMotors();

        if (getGyroYaw() > deg) {
            while (opModeIsActive() && deg < getGyroYaw()) {
                setMotors(-.15, .15);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        } else {
            while (opModeIsActive() && deg > getGyroYaw()) {
                setMotors(.15, -.15);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        }
        stopMotors();
    }

    public void arcTurnCorr(double pow, double deg) throws InterruptedException {
        arcTurnCorr(pow, deg, 6000);
    }

    public void arcTurnCorr(double pow, double deg, int tim) throws InterruptedException {

        if (!opModeIsActive())
            return;

        double newPow;

        ElapsedTime time = new ElapsedTime();

        resetGyro();
        delay(MOVEMENT_DELAY);
        time.reset();

        if (deg > 0) {
            while (opModeIsActive() && deg > getGyroYaw() + gyroError && time.milliseconds() < tim) {
                newPow = Math.abs(pow) * (Math.abs(deg - getGyroYaw()) / 70);

                if (newPow < .15)
                    newPow = .15;

                if (pow > 0)
                    setMotors(newPow, 0);
                else
                    setMotors(0, -newPow);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Gyro Error", gyroError);
                telemetry.update();
                idle();
            }
        } else {
            while (opModeIsActive() && deg < getGyroYaw() + gyroError && time.milliseconds() < tim) {
                newPow = Math.abs(pow) * (Math.abs(deg - getGyroYaw()) / 70);

                if (newPow < .15)
                    newPow = .15;

                if (pow > 0)
                    setMotors(0, newPow);
                else
                    setMotors(-newPow, 0);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Gyro Error", gyroError);
                telemetry.update();
                idle();
            }
        }

        stopMotors();

        if (getGyroYaw() > deg) {
            while (opModeIsActive() && deg < getGyroYaw() + gyroError) {
                if (pow > 0)
                    setMotors(-.14, 0);
                else
                    setMotors(0, .14);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        } else {
            while (opModeIsActive() && deg > getGyroYaw() + gyroError) {
                if (pow > 0)
                    setMotors(0, -.14);
                else
                    setMotors(.14, 0);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        }
        stopMotors();
        delay(100);

        gyroError = getGyroYaw() + gyroError - deg;
    }



    public void arcTurn(double pow, double deg) throws InterruptedException {
        arcTurn(pow, deg, true);
    }
    public void arcTurn(double pow, double deg, boolean stop) throws InterruptedException {
        arcTurn(pow, deg, stop, 6000);
    }

    public void  arcTurn(double pow, double deg, boolean stop, int tim) throws InterruptedException {

        if (!opModeIsActive())
            return;

        double newPow;

        ElapsedTime time = new ElapsedTime();

        resetGyro();
        delay(MOVEMENT_DELAY);
        time.reset();

        if (deg + gyroError > 0) {
            while (opModeIsActive() && deg > getGyroYaw() + gyroError && time.milliseconds() < tim) {
                newPow = Math.abs(pow) * (Math.abs(deg - getGyroYaw()) / 70);

                if (newPow < .15)
                    newPow = .15;

                if (pow > 0)
                    setMotors(newPow, 0);
                else
                    setMotors(0, -newPow);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        } else {
            while (opModeIsActive() && deg < getGyroYaw() + gyroError && time.milliseconds() < tim) {
                newPow = Math.abs(pow) * (Math.abs(deg - getGyroYaw()) / 70);


                if (newPow < .15)
                    newPow = .15;

                if (pow > 0)
                    setMotors(0, newPow);
                else
                    setMotors(-newPow, 0);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        }

        if (stop)
            stopMotors();
        delay(100);
        gyroError = getGyroYaw() + gyroError - deg;
    }

    public void arcTurnPID(double pow, double deg) throws InterruptedException {
        arcTurnPID(pow, deg, 6000);
    }

    public void arcTurnPID(double pow, double deg, int tim) throws InterruptedException {

        if (!opModeIsActive())
            return;

        double newPow;
        double prop;
        double inte = 0;
        double deriv;
        double error;
        double lastError;

        ElapsedTime time = new ElapsedTime();

        resetGyro();
        delay(MOVEMENT_DELAY);
        time.reset();

        if (deg + gyroError > 0) {
            lastError = deg - getGyroYaw();
            while (opModeIsActive() && deg - 2.1 > getGyroYaw() + gyroError && time.milliseconds() < tim) {

                error = deg - getGyroYaw();

                prop = (error * Math.abs(pow)) * .02455 + .015;
                inte = inte + (getRuntime() * error * .000013);
                deriv = ((error - lastError) / getRuntime()) * .000006;

                newPow = prop + inte /* +  deriv*/;


                if (pow > 0)
                    setMotors(newPow, 0);
                else
                    setMotors(0, -newPow);

//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("NewPow", newPow);
//                telemetry.update();
//                Log.i("Error", "" + error);
//                Log.i("Prop", "" + prop);
//                Log.i("Inte", "" + inte);
//                Log.i("Total", "" + newPow);
                resetStartTime();
                lastError = error;
                idle();
            }
        } else {
            lastError = (deg - getGyroYaw()) * -1;
            while (opModeIsActive() && deg + 2.1 < getGyroYaw() + gyroError && time.milliseconds() < tim) {

                error = (deg - getGyroYaw()) * -1;

                prop = (error * Math.abs(pow)) * .02455 + .015;
                inte = inte + (getRuntime() * error * .000013);
                deriv = ((error - lastError) / getRuntime()) * .000006;

                newPow = prop  +  inte /* + deriv*/;

                if (pow > 0)
                    setMotors(0, newPow);
                else
                    setMotors(-newPow, 0);

//            telemetry.addData("Gyro", getGyroYaw());
//            telemetry.addData("NewPow", newPow);
//            telemetry.update();
//            Log.i("Error", ""  + error);
//            Log.i("Prop", ""  + prop);
//            Log.i("Inte", ""  + inte);
//            Log.i("Total", ""  + newPow);
                lastError = error;
                resetStartTime();
                idle();
            }
        }

        stopMotors();

        delay(100);
        gyroError = getGyroYaw() + gyroError - deg;
    }

    public void arcTurnPIDHy(double pow, double deg) throws InterruptedException {
        arcTurnPIDHy(pow, deg, 6000);
    }

    public void arcTurnPIDHy(double pow, double deg, int tim) throws InterruptedException {

        if (!opModeIsActive())
            return;

        double newPow;
        double prop;
        double inte = 0;
        double deriv;
        double error;
        double lastError;

        ElapsedTime time = new ElapsedTime();

        resetGyro();
        delay(MOVEMENT_DELAY);
        time.reset();

        if (deg + gyroError > 0) {
            lastError = deg - getGyroYaw();
            while (opModeIsActive() && deg - 2.1 > getGyroYaw() + gyroError && time.milliseconds() < tim) {

                error = deg - getGyroYaw();

                prop = (error * Math.abs(pow)) * .02455 + .015;
                inte = inte + (getRuntime() * error * .000013);
                deriv = ((error - lastError) / getRuntime()) * .000006;

                newPow = prop + inte /* +  deriv*/;


                if (pow > 0)
                    setMotors(newPow, newPow / -1.8);
                else
                    setMotors(newPow / 1.8, -newPow);

//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("NewPow", newPow);
//                telemetry.update();
//                Log.i("Error", "" + error);
//                Log.i("Prop", "" + prop);
//                Log.i("Inte", "" + inte);
//                Log.i("Total", "" + newPow);
                resetStartTime();
                lastError = error;
                idle();
            }
        } else {
            lastError = (deg - getGyroYaw()) * -1;
            while (opModeIsActive() && deg + 2.1 < getGyroYaw() + gyroError && time.milliseconds() < tim) {

                error = (deg - getGyroYaw()) * -1;

                prop = (error * Math.abs(pow)) * .02455 + .015;
                inte = inte + (getRuntime() * error * .000013);
                deriv = ((error - lastError) / getRuntime()) * .000006;

                newPow = prop  +  inte /* + deriv*/;

                if (pow > 0)
                    setMotors(newPow / -1.8, newPow);
                else
                    setMotors(-newPow, newPow / 1.8);

//            telemetry.addData("Gyro", getGyroYaw());
//            telemetry.addData("NewPow", newPow);
//            telemetry.update();
//            Log.i("Error", ""  + error);
//            Log.i("Prop", ""  + prop);
//            Log.i("Inte", ""  + inte);
//            Log.i("Total", ""  + newPow);
                lastError = error;
                resetStartTime();
                idle();
            }
        }

        stopMotors();

        delay(100);
        gyroError = getGyroYaw() + gyroError - deg;
    }

    public void turn(double pow, double deg) throws InterruptedException {
        turn(pow, deg, 6000);
    }

    public void turn(double pow, double deg, int tim) throws InterruptedException {

        if (!opModeIsActive())
            return;

        double newPow;
        double prop;
        double inte = 0;
        double deriv;
        double error;
        double lastError;

        ElapsedTime time = new ElapsedTime();

        resetGyro();
        delay(MOVEMENT_DELAY);
        time.reset();

        if (deg + gyroError > 0) {
            lastError = deg - getGyroYaw();
            while (opModeIsActive() && deg - 2.1 > getGyroYaw() + gyroError && time.milliseconds() < tim) {

                error = deg - getGyroYaw();

                prop = (error * Math.abs(pow)) * .02455 + .015;
                inte = inte + (getRuntime() * error * .000013);
                deriv = ((error - lastError) / getRuntime()) * .000006;

                newPow = prop + inte /* +  deriv*/;


                if (pow > 0)
                    setMotors(newPow, 0);
                else
                    setMotors(0, -newPow);

//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("NewPow", newPow);
//                telemetry.update();
//                Log.i("Error", "" + error);
//                Log.i("Prop", "" + prop);
//                Log.i("Inte", "" + inte);
//                Log.i("Total", "" + newPow);
                resetStartTime();
                lastError = error;
                idle();
            }
        } else {
            lastError = (deg - getGyroYaw()) * -1;
            while (opModeIsActive() && deg + 2.1 < getGyroYaw() + gyroError && time.milliseconds() < tim) {

                error = (deg - getGyroYaw()) * -1;

                prop = (error * Math.abs(pow)) * .02455 + .015;
                inte = inte + (getRuntime() * error * .000013);
                deriv = ((error - lastError) / getRuntime()) * .000006;

                newPow = prop  +  inte /* + deriv*/;

                if (pow > 0)
                    setMotors(0, newPow);
                else
                    setMotors(-newPow, 0);

//            telemetry.addData("Gyro", getGyroYaw());
//            telemetry.addData("NewPow", newPow);
//            telemetry.update();
//            Log.i("Error", ""  + error);
//            Log.i("Prop", ""  + prop);
//            Log.i("Inte", ""  + inte);
//            Log.i("Total", ""  + newPow);
                lastError = error;
                resetStartTime();
                idle();
            }
        }

        stopMotors();

        delay(100);
        gyroError = getGyroYaw() + gyroError - deg;
    }

    public void untilWhiteRange(double pow, double powWhite, double cm, double deg, int degFail) throws InterruptedException {
        untilWhiteRange(pow, powWhite, cm, deg, degFail, .6, 1.5, 7000);
    }

    public void untilWhiteRange(double pow, double powWhite, double cm, double deg, int degFail, double threshold, double reduction, int tim) throws InterruptedException {

        if (!opModeIsActive())
            return;

        fail = false;

        resetEncoders();
        resetGyro();
//        grayL = floorL.getRawLightDetected();
//        grayR = floorR.getRawLightDetected();
        delay(MOVEMENT_DELAY);

        ElapsedTime time = new ElapsedTime();
        time.reset();

        if (deg > 0) {
            while (opModeIsActive() && deg > getEncoderAverage() && time.milliseconds() < tim) {

                if (getUltraDistance() > cm && getGyroYaw() < 3) {
                    setMotors(pow, pow / (reduction));
                } else if (getUltraDistance() < cm && getGyroYaw() > -3) {
                    setMotors(pow / (reduction), pow);
                } else {

                    if (getGyroYaw() + gyroError > threshold)
                        setMotors(pow / (reduction), pow);
                    else if (getGyroYaw() + gyroError < -threshold)
                        setMotors(pow, pow / (reduction));
                    else
                        setMotors(pow, pow);
                    telemetry.addData("Gyro", getGyroYaw());
                    telemetry.addData("Gyro Error", gyroError);
                    telemetry.addData("Encoder", getEncoderAverage());
                    telemetry.addData("Ultra", getUltraDistance());
                    telemetry.update();
                    Log.w("Gyro", "" + getGyroYaw());
                    idle();
                }
            }
        } else {
            while (opModeIsActive() && Math.abs(deg) > getEncoderAverage() && time.milliseconds() < tim) {

                if (getUltraDistance() > cm && getGyroYaw() > -3) {
                    setMotors(pow, pow / (reduction));
                } else if (getUltraDistance() < cm && getGyroYaw() < 3) {
                    setMotors(pow / (reduction), pow);
                } else {
                    if (getGyroYaw() + gyroError > threshold)
                        setMotors(pow, pow / (reduction));
                    else if (getGyroYaw() + gyroError < -threshold)
                        setMotors(pow / (reduction), pow);
                    else
                        setMotors(pow, pow);
                    telemetry.addData("Gyro", getGyroYaw());
                    telemetry.addData("Gyro Error", gyroError);
                    telemetry.addData("Encoder", getEncoderAverage());
                    telemetry.addData("Ultra", getUltraDistance());
                    telemetry.update();
                    Log.w("Gyro", "" + getGyroYaw());
                    idle();
                }
            }
        }

        if (pow > 0) {
            while (opModeIsActive() && (floorL.getRawLightDetected() < grayL + .5 && floorR.getRawLightDetected() < grayR + .5) && time.milliseconds() < tim) {
                if (getUltraDistance() > cm && getGyroYaw() < 3) {
                    setMotors(powWhite, powWhite / (reduction + .05));
                } else if (getUltraDistance() < cm && getGyroYaw() > -3) {
                    setMotors(powWhite / (reduction + .05), powWhite);
                } else {

                    if (getGyroYaw() + gyroError > threshold)
                        setMotors(powWhite / reduction, powWhite);
                    else if (getGyroYaw() + gyroError < -threshold)
                        setMotors(powWhite, powWhite / reduction);
                    else
                        setMotors(powWhite, powWhite);
                }

                if (Math.abs(degFail) < getEncoderAverage()) {
                    untilWhiteRange(-.15, -.15, 14, 0, 3000);
                    moveTo(.2, 90, .6, 1.5);
                    fail = true;
                    break;
                }
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Gyro Error", gyroError);
                telemetry.addData("FloorL", floorL.getRawLightDetected());
                telemetry.addData("FloorR", floorR.getRawLightDetected());
                telemetry.addData("Ultra", getUltraDistance());
                Log.w("FloorL", "" + floorL.getRawLightDetected());
                Log.w("FloorR", "" + floorR.getRawLightDetected());
                telemetry.update();
                idle();
            }
        } else {
            while (opModeIsActive() && (floorL.getRawLightDetected() < grayL + .5 && floorR.getRawLightDetected() < grayR + .5) && time.milliseconds() < tim) {
                if (getUltraDistance() > cm && getGyroYaw() > -3) {
                    setMotors(powWhite, powWhite / (reduction + .05));
                } else if (getUltraDistance() < cm && getGyroYaw() < 3) {
                    setMotors(powWhite / (reduction + .05), powWhite);
                } else {
                    if (getGyroYaw() + gyroError > threshold) {
                        setMotors(powWhite, powWhite / reduction);
                    } else if (getGyroYaw() + gyroError < -threshold) {
                        setMotors(powWhite / reduction, powWhite);
                    } else {
                        setMotors(powWhite, powWhite);
                    }
                }

                if (Math.abs(degFail) < getEncoderAverage()) {
                    untilWhiteRange(.15, .15, 14, 0, 3000);
                    moveTo(.2, -100, .6, 1.5);
                    fail = true;
                    break;
                }
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Gyro Error", gyroError);
                telemetry.addData("FloorL", floorL.getRawLightDetected());
                telemetry.addData("FloorR", floorR.getRawLightDetected());
                telemetry.addData("Ultra", getUltraDistance());
                Log.w("FloorL", "" + floorL.getRawLightDetected());
                Log.w("FloorR", "" + floorR.getRawLightDetected());
                telemetry.update();
                idle();
            }
        }

        gyroError = getGyroYaw() + gyroError;
        stopMotors();
    }

    public void untilWhite(double pow, double powWhite) throws InterruptedException {
        untilWhite(pow, powWhite, 0, 10000);
    }

    public void untilWhite(double pow, double powWhite, int deg, int degFail) throws InterruptedException {
        untilWhite(pow, powWhite, deg, degFail, .6, 1.75, 7000);
    }

    public void untilWhite(double pow, double powWhite, int deg, int degFail, double threshold, double reduction, int tim) throws InterruptedException {

        if (!opModeIsActive())
            return;

        fail = false;

        resetEncoders();
        resetGyro();
//        grayL = floorL.getRawLightDetected();
//        grayR = floorR.getRawLightDetected();
        delay(MOVEMENT_DELAY);

        ElapsedTime time = new ElapsedTime();
        time.reset();

        if (deg > 0) {
            while (opModeIsActive() && deg > getEncoderAverage() && time.milliseconds() < tim) {

                if (getGyroYaw() + gyroError > threshold)
                    setMotors(pow / reduction, pow);
                else if (getGyroYaw() + gyroError < -threshold)
                    setMotors(pow, pow / reduction);
                else
                    setMotors(pow, pow);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Gyro Error", gyroError);
                telemetry.addData("Encoder", getEncoderAverage());
                telemetry.update();
                Log.w("Gyro", "" + getGyroYaw());
                idle();
            }
        } else {
            while (opModeIsActive() && Math.abs(deg) > getEncoderAverage() && time.milliseconds() < tim) {

                if (getGyroYaw() + gyroError > threshold)
                    setMotors(pow, pow / reduction);
                else if (getGyroYaw() + gyroError < -threshold)
                    setMotors(pow / reduction, pow);
                else
                    setMotors(pow, pow);

                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Gyro Error", gyroError);
                telemetry.addData("Encoder", getEncoderAverage());
                telemetry.update();
                Log.w("Gyro", "" + getGyroYaw());
                idle();
            }
        }

        if (pow > 0) {
            while (opModeIsActive() && (floorL.getRawLightDetected() < grayL + .5 && floorR.getRawLightDetected() < grayR + .5) && time.milliseconds() < tim) {

                if (getGyroYaw() + gyroError > threshold)
                    setMotors(powWhite / reduction, powWhite);
                else if (getGyroYaw() + gyroError < -threshold)
                    setMotors(powWhite, powWhite / reduction);
                else
                    setMotors(powWhite, powWhite);

                if (Math.abs(degFail) < getEncoderAverage()) {
                    untilWhite(-.15, -.15, 0, 3000);
                    moveTo(.2, 100, .6, 1.5);
                    fail = true;
                    break;
                }
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Gyro Error", gyroError);
                telemetry.addData("FloorL", floorL.getRawLightDetected());
                telemetry.addData("FloorR", floorR.getRawLightDetected());
                Log.w("FloorL", "" + floorL.getRawLightDetected());
                Log.w("FloorR", "" + floorR.getRawLightDetected());
                telemetry.update();
                idle();
            }
        } else {
            while (opModeIsActive() && (floorL.getRawLightDetected() < grayL + .5 && floorR.getRawLightDetected() < grayR + .5) && time.milliseconds() < tim) {

                if (getGyroYaw() + gyroError > threshold) {
                    setMotors(powWhite, powWhite / reduction);
                } else if (getGyroYaw() + gyroError < -threshold) {
                    setMotors(powWhite / reduction, powWhite);
                } else {
                    setMotors(powWhite, powWhite);
                }

                if (Math.abs(degFail) < getEncoderAverage()) {
                    untilWhite(.15, .15, 0, 3000);
                    moveTo(.2, -100, .6, 1.5);
                    fail = true;
                    break;
                }

                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Gyro Error", gyroError);
                telemetry.addData("FloorL", floorL.getRawLightDetected());
                telemetry.addData("FloorR", floorR.getRawLightDetected());
                Log.w("FloorL", "" + floorL.getRawLightDetected());
                Log.w("FloorR", "" + floorR.getRawLightDetected());
                telemetry.update();
                idle();
            }
        }

        gyroError = getGyroYaw() + gyroError;
        stopMotors();
    }

    public void whiteTurn(double pow, int turns) {
        int count = 0;

        while (count < turns) {
            while (floorR.getRawLightDetected() > grayR - .4) {
                setMotors(0, pow);
            }

            count++;
            if (floorL.getRawLightDetected() < grayL - .4 && floorR.getRawLightDetected() < grayR - .4)
                break;

            while (floorL.getRawLightDetected() > grayL - .4) {
                setMotors(-pow, 0);
            }

            count++;

            if (floorL.getRawLightDetected() < grayL - .4 && floorR.getRawLightDetected() < grayR - .4)
                break;
        }
    }




    public void untilWhiteAlign(double pow, double powWhite) throws InterruptedException {
        untilWhiteAlign(pow, powWhite, 0, 10000);
    }

    public void untilWhiteAlign(double pow, double powWhite, int deg, int degFail) throws InterruptedException {
        untilWhiteAlign(pow, powWhite, deg, degFail, .65 , 7000, true);
    }

    public void untilWhiteAlign(double pow, double powWhite, int deg, int degFail, double reduction, int tim, boolean failSafe) throws InterruptedException {

        if (!opModeIsActive())
            return;

        fail = false;

        resetEncoders();
        resetGyro();
//        grayL = floorL.getRawLightDetected();
//        grayR = floorR.getRawLightDetected();
        delay(MOVEMENT_DELAY);

        ElapsedTime time = new ElapsedTime();
        time.reset();

        double prevEncoder = 0;
        ElapsedTime time2 = new ElapsedTime();
        time2.reset();

        if (pow > 0) {
            while (opModeIsActive() && deg > getEncoderAverage() && time.milliseconds() < tim) {
                setMotors(pow, pow * reduction);
                if (time2.seconds() > .25) {
                    if (getEncoderAverage() - prevEncoder < 100 && failSafe) {
                        arcTurnPID(-.65, -20, 1000);
                        tim += 1000;
                    }
                    time2.reset();
                    prevEncoder = getEncoderAverage();
                }
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Gyro Error", gyroError);
                telemetry.addData("Encoder", getEncoderAverage());
                telemetry.update();
                Log.w("Gyro", "" + getGyroYaw());
                idle();
            }
        } else {
            while (opModeIsActive() && Math.abs(deg) > getEncoderAverage() && time.milliseconds() < tim) {
                setMotors(pow, pow * reduction);
                if (time2.seconds() > .25) {
                    if (getEncoderAverage() - prevEncoder < 100 && failSafe) {
                        arcTurnPID(.65, 20, 1000);
                        tim += 1000;
                    }
                    time2.reset();
                    prevEncoder = getEncoderAverage();
                }
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Gyro Error", gyroError);
                telemetry.addData("Encoder", getEncoderAverage());
                telemetry.update();
                Log.w("Gyro", "" + getGyroYaw());
                idle();
            }
        }

        if (pow > 0) {
            while (opModeIsActive() && (floorL.getRawLightDetected() < grayL + .5 && floorR.getRawLightDetected() < grayR + .5) && time.milliseconds() < tim) {

                setMotors(powWhite, powWhite * reduction);

                if (Math.abs(degFail) < getEncoderAverage()) {
                    untilWhiteAlign(-.15, -.15, 0, 2300, .65, 3000, false);
                    moveTo(.2, 100, .6, 1.5);
                    fail = true;
                    break;
                }

                if (time2.seconds() > .25) {
                    if (getEncoderAverage() - prevEncoder < 100 && failSafe) {
                        arcTurnPID(-.65, -15, 1000);
                        tim += 1;
                    }
                    time2.reset();
                    prevEncoder = getEncoderAverage();
                }
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.addData("FloorL", floorL.getRawLightDetected());
//                telemetry.addData("FloorR", floorR.getRawLightDetected());
//                telemetry.addData("Encoder", getEncoderAverage());
//                Log.w("FloorL", "" + floorL.getRawLightDetected());
//                Log.w("FloorR", "" + floorR.getRawLightDetected());
//                telemetry.update();
                idle();
            }
        } else {
            while (opModeIsActive() && (floorL.getRawLightDetected() < grayL + .5 && floorR.getRawLightDetected() < grayR + .5) && time.milliseconds() < tim) {

                setMotors(powWhite, powWhite * reduction);

                if (Math.abs(degFail) < getEncoderAverage()) {
                    untilWhiteAlign(.15, .15, 60, 2300, .65, 3000, false);
                    moveTo(.2, -150, .6, 1.5);
                    fail = true;
                    break;
                }

                if (time2.seconds() > .25) {
                    if (getEncoderAverage() - prevEncoder < 100  && failSafe) {
                        arcTurnPID(.65, 15, 1000);
                        tim += 1;
                    }
                    time2.reset();
                    prevEncoder = getEncoderAverage();
                }

//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.addData("FloorL", floorL.getRawLightDetected());
//                telemetry.addData("FloorR", floorR.getRawLightDetected());
//                telemetry.addData("Encoder", getEncoderAverage());
//                Log.w("FloorL", "" + floorL.getRawLightDetected());
//                Log.w("FloorR", "" + floorR.getRawLightDetected());
//                telemetry.update();
                idle();
            }
        }

        gyroError = getGyroYaw() + gyroError;
        stopMotors();
    }

    public void untilRange(double pow) throws InterruptedException {
        untilRange(pow, 30, .5, 2.2, 7000);
    }

    public void untilRange(double pow, double endDistance, double threshold, double reduction, int tim) throws InterruptedException {

        if (!opModeIsActive())
            return;

        resetEncoders();
        delay(1000);

        ElapsedTime time = new ElapsedTime();
        time.reset();

        if (pow > 0) {
            while (opModeIsActive() && (getUltraDistance() < endDistance) && time.milliseconds() < tim) {
                if (getGyroYaw() > threshold)
                    setMotors(pow / reduction, pow);
                else if (getGyroYaw() < -threshold)
                    setMotors(pow, pow / reduction);
                else
                    setMotors(pow, pow);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Ultrasonic", getUltraDistance());
                Log.w("Ultrasonic", "" + getUltraDistance());
                telemetry.update();
                idle();
            }
        } else {
            while (opModeIsActive() && (getUltraDistance() > endDistance) && time.milliseconds() < tim) {
                if (getGyroYaw() > threshold) {
                    setMotors(pow, pow / reduction);
                } else if (getGyroYaw() < -threshold) {
                    setMotors(pow / reduction, pow);
                } else {
                    setMotors(pow, pow);
                }

                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Ultrasonic", getUltraDistance());
                Log.w("Ultrasonic", "" + getUltraDistance());
                telemetry.update();
                idle();
            }
        }
        stopMotors();
    }

    public void moveParallel(double pow) throws InterruptedException {
        if (!opModeIsActive())
            return;

        resetEncoders();
        delay(MOVEMENT_DELAY);

        while (getGyroYaw() > 3) {
            setMotors(pow, pow);
        }

        stopMotors();
    }



    public void moveAlign(double pow, double ramPow, double dis, double failDis) throws InterruptedException {

        if (!opModeIsActive())
            return;

        resetEncoders();
        delay(MOVEMENT_DELAY);

        if (pow > 0) {
            while(getEncoderAverage() < dis) {
                setMotors(pow, pow);
            }

            while (opModeIsActive() && getGyroYaw() > 6.5 && failDis > getEncoderAverage()) {
                setMotors(ramPow, ramPow);
                telemetry.addData("Gryo", getGyroYaw());
                telemetry.update();
            }

        } else {
            while(getEncoderAverage() < dis) {
                setMotors(pow, pow);
            }

            while (opModeIsActive() && getGyroYaw() - 180 < -6.5 && failDis > getEncoderAverage()) {
                setMotors(ramPow, ramPow);
            }
        }

        stopMotors();
    }
    public void moveToSlow(double pow, double deg) throws InterruptedException {
        moveTo(pow, deg, .6);
    }

    public void moveToSlow(double pow, double deg, double threshold) throws InterruptedException {
        moveToSlow(pow, deg, threshold, 2.2);
    }

    public void moveToSlow(double pow, double deg, double threshold, double red) throws InterruptedException {
        moveToSlow(pow, deg, threshold, red, 15000, true);
    }

    public void moveToSlow(double pow, double deg, double threshold, double red, int tim, boolean stop) throws InterruptedException {

        if (!opModeIsActive())
            return;

        ElapsedTime time = new ElapsedTime();


        resetGyro();
        resetEncoders();
        delay(MOVEMENT_DELAY);

        time.reset();

        if (pow > 0) {
            while (opModeIsActive() && deg > getEncoderAverage() && time.milliseconds() < tim) {
                double power = (((pow-.2)*((deg - getEncoderAverage())/deg))+.2);
                if (getGyroYaw() + gyroError > threshold)
                    setMotors(power / red, power);
                else if (getGyroYaw() + gyroError < -threshold)
                    setMotors(power, power / red);
                else
                    setMotors(power, power);
//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.addData("Power", power);
//                telemetry.addData("Encoder", getEncoderAverage());
//                telemetry.update();
//                Log.w("Gyro", "" + getGyroYaw());
                idle();
            }
        } else {
            while (opModeIsActive() && deg > getEncoderAverage() && time.milliseconds() < tim) {
                double power = (((pow+.2)*((deg - getEncoderAverage())/deg))-.2);
                if (getGyroYaw() + gyroError > threshold)
                    setMotors(power, power / red);
                else if (getGyroYaw() + gyroError < -threshold)
                    setMotors(power / red, power);
                else
                    setMotors(power, power);

//                telemetry.addData("Gyro", getGyroYaw());
//                telemetry.addData("Gyro Error", gyroError);
//                telemetry.addData("Power", power);
//                telemetry.addData("Encoder", getEncoderAverage());
//                telemetry.update();
//                Log.w("Gyro", "" + getGyroYaw());
                idle();
            }
        }
        if (stop)
            stopMotors();

        gyroError = getGyroYaw() + gyroError;
    }


=======
>>>>>>> refs/remotes/origin/master
}