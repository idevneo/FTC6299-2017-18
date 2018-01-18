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


import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import java.util.Locale;

public abstract class MyOpMode extends LinearOpMode {

    public static BNO055IMU imu;

    public VuforiaLocalizer vuforia;

    public static DcMotor motorBL;
    public static DcMotor motorBR;
    public static DcMotor motorFL;
    public static DcMotor motorFR;
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

    public char column;

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
//                        return Character.toString(vfValue());
//                    }
//                });
        telemetry.addLine()
                .addData("Left", new Func<String>() {
                    @Override
                    public String value() {
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
                    @Override
                    public String value() {
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
                    @Override
                    public String value() {
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
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");

        liftLeft = hardwareMap.dcMotor.get("liftL");
        liftRight = hardwareMap.dcMotor.get("liftR");
        manip = hardwareMap.dcMotor.get("manip");

        jewelColor = hardwareMap.get(ColorSensor.class, "jewelColor");
        jewelArm = hardwareMap.servo.get("jewelArm");
        jewelHand = hardwareMap.servo.get("jewelHand");

        rangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeR");
        rangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeL");
        rangeF = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeC");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AXb/g5n/////AAAAGSUed2rh5Us1jESA1cUn5r5KDUqTfwO2woh7MxjiLKSUyDslqBAgwCi0Qmc6lVczErnF5TIw7vG5R4TJ2igvrDVp+dP+3i2o7UUCRRj/PtyVgb4ZfNrDzHE80/6TUHifpKu4QCM04eRWYZocWNWhuRfytVeWy6NSTWefM9xadqG8FFrFk3XnvqDvk/6ZAgerNBdq5SsJ90eDdoAhgYEee40WxasoUUM9YVMvkWOqZgHSuraV2IyIUjkW/u0O+EkFtTNRUWP+aZwn1qO1H4Lk07AJYe21eqioBLMdzY7A8YqR1TeQ//0WJg8SFdXjuGbF6uHykBe2FF5UeyaehA0iTqfPS+59FLm8y1TuUt57eImq";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        BNO055IMU.Parameters Gparameters = new BNO055IMU.Parameters();
        Gparameters.mode = BNO055IMU.SensorMode.IMU;
        Gparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Gparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Gparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        Gparameters.loggingEnabled = true;
        Gparameters.loggingTag = "IMU";
        Gparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(Gparameters);
//        relic = hardwareMap.dcMotor.get("relic");
//        relicGrabber = hardwareMap.servo.get("relicGrabber");

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void vfValue() {
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
        telemetry.addData("Column ", column);

        ElapsedTime time = new ElapsedTime();
        time.reset();
        resetStartTime();

        while ((time.milliseconds() < 2000) && opModeIsActive()) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                if (vuMark == RelicRecoveryVuMark.CENTER) {
                    column = 'C';
                } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                    column = 'L';
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    column = 'R';
                }
            } else {
                column = 'U';
                telemetry.addData("VuMark", "%s visible", vuMark);
            }
            telemetry.update();
        }
    }

    public void vfMovePar(char rb, ModernRoboticsI2cRangeSensor sensorVar, double bSwitch) {
        double centerDis = 26.75;
        double kC = 0;
        if (rb == 'r')
            kC = 6;
        if (rb == 'b')
            kC = -6;
        if (column == 'L') {
            rangeMoveStrafe((centerDis + kC), sensorVar, bSwitch);
        } else if (column == 'R') {
            rangeMoveStrafe((centerDis - kC), sensorVar, bSwitch);
        } else if (column == 'C') {

        } else if (column == 'U') {

        }

    }

    public void vfMovePerp(char rb, ModernRoboticsI2cRangeSensor sensorVar, double bSwitch) {
        double centerDis = 26.75; //needs to be tested
        double kC = 0;
        if (rb == 'r')
            kC = 6; //needs to be tested
        if (rb == 'b')
            kC = -6; //needs to be tested
        if (column == 'L') {
            rangeMoveStrafe((centerDis + kC), sensorVar, bSwitch);
        } else if (column == 'R') {
            rangeMoveStrafe((centerDis - kC), sensorVar, bSwitch);
        } else if (column == 'C') {

        } else if (column == 'U') {

        }
    }

    public void vfMoveAlt() {

        if (column == 'L') {
            setMotorStrafe(-.25);
            sleep(750);
            stopMotors();
        } else if (column == 'R') {
            setMotorStrafe(.25);
            sleep(675);
            stopMotors();
        } else if (column == 'C') {
        } else if (column == 'U') {
        }

    }

    public void rangeMovePID(double inAway, ModernRoboticsI2cRangeSensor sensorVar) {
        double sensor = sensorVar.getDistance(DistanceUnit.INCH);
//        double differenceDis;
//        double kP = .025; //to be determined
        double pow;
        double localRange;
        while (((sensor < inAway - .35) || (sensor > inAway + .35)) && opModeIsActive()) { //While sensor doesn't = tolerance, run.
            localRange = sensorVar.getDistance(DistanceUnit.INCH);
            while ((Double.isNaN(localRange) || (localRange > 1000)) && opModeIsActive()) {
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

    public void rangeMoveStrafe(double inAway, ModernRoboticsI2cRangeSensor sensorVar, double bSet) { //Basing Switch | 1 = Left | 0 = Right
        double sensor = sensorVar.getDistance(DistanceUnit.INCH);

//        double range;
        double pow = .15;
//        double newPow;
        double localRange;

        while (((sensor < inAway - .5) || (sensor > inAway + .5)) && opModeIsActive()) {
            localRange = sensorVar.getDistance(DistanceUnit.INCH);
            while ((Double.isNaN(localRange) || (localRange > 1000)) && opModeIsActive()) {
                localRange = sensorVar.getDistance(DistanceUnit.INCH);
            }
            sensor = localRange;
//            range = Math.abs(inAway - sensor);



            pow = .15;

            //RED SIDE AUTOS
            if (bSet == 0) {
                if (sensor > inAway) {
                    setMotorStrafe(pow);
                }
                if (sensor < inAway) {
                    setMotorStrafe(-pow);
                }
            }


            //BLUE SIDE AUTOS
            if (bSet == 1) {
                if (sensor > inAway) {
                    setMotorStrafe(-pow);
                }
                if (sensor < inAway) {
                    setMotorStrafe(pow);
                }
            }
            telemetry.addData("rangeDis", sensor);
            telemetry.update();
        }
        stopMotors();
    }

    public void turnCorr2(double pow, double deg, int timer) throws InterruptedException {
        if (!opModeIsActive())
            return;

        double newPow;
        double error;
        double errorMove;

        ElapsedTime time = new ElapsedTime();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)); // currPos is the current position

        delay(100);
        time.reset();

        while (((currPos > (deg + 2.5)) || (currPos < (deg - 2.5))) && (time.milliseconds() < timer) && opModeIsActive()) {
            error = deg - currPos;
            errorMove = Math.abs(deg - currPos);

            if (error > 180) {
                error = error - 360;
            } else if (error < -180) {
                error = error + 360;
            }

            newPow = pow * (Math.abs(error) / 70);
            if (newPow < .15)
                newPow = .1;

            if (currPos < deg) {
                if (errorMove < 180) {
                    setMotors(-newPow, newPow); //Turns left
                }
                if (errorMove > 180) {
                    setMotors(newPow, -newPow); //Turns right if we go past the pos/neg mark.
                }
            } else if (currPos > deg) {
                if (errorMove < 180) {
                    setMotors(newPow, -newPow); //Turns right
                }
                if (errorMove > 180) {
                    setMotors(-newPow, newPow); //Turns left if we go past the pos/neg mark.
                }
            }
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("Gyro", currPos);
            telemetry.update();
        }
        stopMotors();
    }

    public void slowDown(double reduction) {
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

        if (motorFL.getPower() < 0.05 && motorBL.getPower() < 0.05 && motorFR.getPower() < 0.05 && motorFL.getPower() < 0.05) {
            motorFL.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(0);
            motorBR.setPower(0);
        }

    }

    public void jewelKnockerRed() {
        jewelArm.setPosition(.6);
        jewelHand.setPosition(.4);
        sleep(750);
        jewelArm.setPosition(.15);
        sleep(1000);
        if (jewelColor.red() > jewelColor.blue()) {
            jewelHand.setPosition((.3));
        } else if (jewelColor.red() < jewelColor.blue()) {
            jewelHand.setPosition((.6));
        }
        sleep(500);

        jewelArm.setPosition(.6);
        jewelHand.setPosition(.45);
        sleep(1000);
        jewelHand.setPosition(.3);
        sleep(500);
    }

    public void jewelKnockerBlue() {
        jewelArm.setPosition(.55);
        jewelHand.setPosition(.4);
        sleep(500);
        jewelArm.setPosition(.15);
        sleep(1000);
        if (jewelColor.red() < jewelColor.blue()) {
            jewelHand.setPosition((.3));
        } else if (jewelColor.red() > jewelColor.blue()) {
            jewelHand.setPosition((.6));
        }
        sleep(500);

        jewelArm.setPosition(.55);
        jewelHand.setPosition(.45);
        sleep(500);
        jewelHand.setPosition(.3);
        sleep(500);
    }

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

//    public void setServoSlow(Servo servo, double pos) throws InterruptedException {
//        double currentPosition = servo.getPosition();
//
//        if (currentPosition - pos > 0) {
//            for (; currentPosition > pos; currentPosition -= .005) {
//                servo.setPosition(currentPosition);
//                delay(1);
//                idle();
//            }
//        } else for (; currentPosition < pos; currentPosition += .005) {
//            servo.setPosition(currentPosition);
//            delay(1);
//            idle();
//        }
//    }

//    public double getDiff(double angle) {
//        imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
//        return currPos - angle;
//    }
//
//    public Double gyroVal() {
//        imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
//        return currPos;
//    }
//
//    public void turnPID(double angle) throws InterruptedException {
//        double kP = .20 / 90;
//        double min = -.2;
//        double max = .2;
//        double changeCon = .04;
//        double PIDchange;
//        double angleDiff = getDiff(angle);
//        double oldDiff = angleDiff;
//        int counter = 0;
//        double startAngle = gyroVal();
//        while ((Math.abs(angleDiff) <= Math.abs(oldDiff)) && opModeIsActive()) {
//            PIDchange = angleDiff * kP;
//
//            if (PIDchange < 0) {
//                motorFR.setPower(Range.clip(-PIDchange + changeCon, min, max));
//                motorBR.setPower(Range.clip(-PIDchange + changeCon, min, max));
//                motorFL.setPower(Range.clip(-PIDchange + changeCon, min, max));
//                motorBL.setPower(Range.clip(-PIDchange + changeCon, min, max));
//            } else {
//                motorFR.setPower(Range.clip(PIDchange - changeCon, min, max));
//                motorBR.setPower(Range.clip(PIDchange - changeCon, min, max));
//                motorFL.setPower(Range.clip(PIDchange - changeCon, min, max));
//                motorBL.setPower(Range.clip(PIDchange - changeCon, min, max));
//            }
//
//            sleep(250);
//            telemetry.addData("gyroStart", startAngle);
//            telemetry.addData("counter", counter++);
//            telemetry.addData("GyroVal", gyroVal());
//            telemetry.addData("GyroDiff", getDiff(angle));
//            telemetry.addData("Pow", -PIDchange - changeCon);
//            telemetry.update();
//
//            oldDiff = angleDiff;
//            angleDiff = getDiff(angle);
//        }
//        stopMotors();
//    }

//    public void turnCorr(double pow, double deg, int tim) throws InterruptedException {
//        if (!opModeIsActive())
//            return;
//
//        double newPow = 0.0;
//        double error;
//        double timer = 0.0;
//
//        ElapsedTime time = new ElapsedTime();
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)); // startP is the current position
//
//        delay(100);
//        time.reset();
//
//        if (deg > 0) {
//<<<<<<<HEAD
//            while ((deg > currPos && time.milliseconds() < tim)) {
//                error = deg - currPos;
//
//                if (error > 180) {
//                    error = error - 360;
//                } else if (error <= -180) {
//                    error = error + 360;
//                }
//=======
//                telemetry.addData("deg", "positive");
//                while ((deg > currPos && time.milliseconds() < tim && opModeIsActive())) {
////                error = deg - currPos;
////
////                if (error > 180){
////                    error = error -360;
////                } else if (error < -180){
////                    error = error +360;
////                }
////
////                newPow = pow * (Math.abs(error) / 80);
////
//////                if ((deg - currPos) - 360)
////
////                if (newPow < .15)
////                    newPow = .1;
////
////                if (currPos > 0) {
////                    setMotors(-newPow, newPow);
////                } else if (currPos < 0) {
////                    setMotors(-newPow, newPow);
////
////                }
//                    //delay(100);
////                if (Math.abs(time.milliseconds() - timer) > 100) {
////                    if (currPos > 0) {
////                        setMotors(-newPow, newPow);
////                    } else if (currPos < 0) {
////                        setMotors(-newPow, newPow);
////
////                    }
////                    timer = time.milliseconds();
////                }
//                    if (Math.abs(time.milliseconds() - timer) > 100) {
//                        error = deg - currPos;
//
//                        if (error > 180) {
//                            error = error - 360;
//                        } else if (error < -180) {
//                            error = error + 360;
//                        }
//>>>>>>>3208f 66678255ed 1025e3 ab9fac5fa2ffba0980b
//
//                                newPow = pow * (Math.abs(error) / 80);
//
////                if ((deg - currPos) - 360)
//
//                        if (newPow < .15)
//                            newPow = .1;
//
//                        if (currPos > 0) {
//                            setMotors(-newPow, newPow);
//                        } else if (currPos < 0) {
//                            setMotors(-newPow, newPow);
//
//                        }
//                        timer = time.milliseconds();
//                    }
//                    currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
//                    telemetry.addData("Gyro", currPos);
//                    telemetry.addData("newpower", newPow);
//                    telemetry.addData("Error", (Math.abs(deg - currPos)));
//                    telemetry.update();
//                }
//            } else if (deg < 0) {
//
//                telemetry.addData("deg", "negative");
//                telemetry.update();
//                while (deg < currPos && time.milliseconds() < tim && opModeIsActive()) {
//
////                error = deg - currPos;
////
////                if (error > 180){
////                    error = error - 360;
////                } else if (error < -180){
////                    error = error + 360;
////                }
////
////                newPow = pow * (Math.abs(error) / 80);
////
////                if (newPow < .15)
////                    newPow = .1;
////
////                if (currPos < 0) {
////                    setMotors(newPow, -newPow);
////                }else if (currPos > 0){
////                    setMotors(newPow, -newPow);
////
////                }
////                delay(100);
//                    if (Math.abs(time.milliseconds() - timer) > 100) {
//                        error = deg - currPos;
//
//                        if (error > 180) {
//                            error = error - 360;
//                        } else if (error < -180) {
//                            error = error + 360;
//                        }
//
//                        newPow = pow * (Math.abs(error) / 80);
//
//                        if (newPow < .15)
//                            newPow = .1;
//
//                        if (currPos < 0) {
//                            setMotors(newPow, -newPow);
//                        } else if (currPos > 0) {
//                            setMotors(newPow, -newPow);
//                        }
//                        timer = time.milliseconds();
//                    }
//                    currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
//                    telemetry.addData("Gyro", currPos);
//                    telemetry.addData("newpower", newPow);
//                    telemetry.addData("Error", (Math.abs(deg - currPos)));
//                    telemetry.update();
//                    // idle();
//                }
//            } else {
//                telemetry.addData("nothing", "nothing");
//                telemetry.update();
//            }
//            telemetry.addData("status", "done");
//            telemetry.update();
//            stopMotors();
    }
