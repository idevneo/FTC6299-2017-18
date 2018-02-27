package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
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

import android.util.Log;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

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
    public static Servo manipWall;
    public static DcMotor relicDrive;
    public static Servo jewelArm;
    public static Servo jewelHand;
    public static Servo relicFlip;
    public static Servo relicHand;
    public static ColorSensor jewelColor;

    public static Orientation angles;
    public static Acceleration gravity;

    public static ModernRoboticsI2cRangeSensor rangeR;
    public static ModernRoboticsI2cRangeSensor rangeL;
    public static ModernRoboticsI2cRangeSensor rangeF;

    public char column;
    private boolean align;

    public String formatAngle(AngleUnit angleUnit, double angle) { //Formats the IMU Angle data into strings that can pass into telemetry.
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees) { //Formats the IMU Degree data into strings that can pass into telemetry.
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void composeTelemetry() { //Method that holds a majority of sensor telemetry for testing.
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
        telemetry.addLine()
                .addData("Angle", new Func<String>() { //Position of the robots angle.
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle); //Control Robot Pivot
                    }
                });
        telemetry.addLine()
                .addData("Left", new Func<String>() { //Left Range Sensor Telemetry.
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
                .addData("Right", new Func<String>() { //Right Range Sensor Telemetry.
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
                .addData("Front", new Func<String>() { //Front Range Sensor Telemetry.
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
    }

    public void hMap(HardwareMap type) { //Initialization of the Robot's hardware map in autonomous.
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");

        liftLeft = hardwareMap.dcMotor.get("liftL");
        liftRight = hardwareMap.dcMotor.get("liftR");
        manip = hardwareMap.dcMotor.get("manip");
        manipWall = hardwareMap.servo.get("manipWall");
        jewelColor = hardwareMap.get(ColorSensor.class, "jewelColor");
        jewelArm = hardwareMap.servo.get("jewelArm");
        jewelHand = hardwareMap.servo.get("jewelHand");
        relicDrive = hardwareMap.dcMotor.get("relicDrive");
        relicFlip = hardwareMap.servo.get("relicFlip");
        relicHand = hardwareMap.servo.get("relicHand");

        rangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeR");
        rangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeL");
        rangeF = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeC");

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
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

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void hMapT(HardwareMap type) { //Initialization of the Robot's hardware map in TeleOp.
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");

        liftLeft = hardwareMap.dcMotor.get("liftL");
        liftRight = hardwareMap.dcMotor.get("liftR");
        manip = hardwareMap.dcMotor.get("manip");
        manipWall = hardwareMap.servo.get("manipWall");
        jewelColor = hardwareMap.get(ColorSensor.class, "jewelColor");
        jewelArm = hardwareMap.servo.get("jewelArm");
        jewelHand = hardwareMap.servo.get("jewelHand");
        relicDrive = hardwareMap.dcMotor.get("relicDrive");
        relicFlip = hardwareMap.servo.get("relicFlip");
        relicHand = hardwareMap.servo.get("relicHand");

        rangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeR");
        rangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeL");
        rangeF = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeC");

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void delay(long milliseconds) throws InterruptedException { //Delays the code via a sleep time.
        if (milliseconds < 0)
            milliseconds = 0;
        Thread.sleep(milliseconds);
    }

    public void setMotorsAll(double linear, double strafe, double turn) {
        motorFL.setPower(linear - strafe + turn); //Figure out the combinations for the setting of power.
        motorBL.setPower(linear - strafe + turn);
        motorFR.setPower(linear - strafe + turn);
        motorBR.setPower(linear - strafe + turn);
    }

    public void setMotors(double left, double right) { //Moves forward when both values are positive.
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

    public void stopMotors() { //Stops the motors.
        if (!opModeIsActive())
            return;

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }

    public void manipAuto(double pow) { //Runs the manipulator for a certain time in auto at a selected power.
        if (!opModeIsActive()) //Doesn't run if the OpMode isn't active.
            return;
        if (pow > 0 || pow < 0) { //Runs the manipulator at the given power if it is not 0.
            manip.setPower(pow);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            manip.setPower(0);
        }
        else { //If the power is 0, don't run the manipulator..
            manip.setPower(0);
            }
    }

    public void vfValue() { //Finds the column we need to place the glyph in using Vuforia.
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
        column = 'U';

        ElapsedTime time = new ElapsedTime();
        time.reset();
        resetStartTime();

        while ((time.milliseconds() < 2000) && opModeIsActive() && (column == 'U')) { //Finding the Cryptokey
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            switch (vuMark) { //Assigns the Cryptokey Image to a value we can use later.
                case UNKNOWN:
                    column = 'U';
                    break;
                case CENTER:
                    column= 'C';
                    break;
                case LEFT:
                    column= 'L';
                    break;
                case RIGHT:
                    column = 'R';
                    break;
            }
        }
        telemetry.addData("Column", column);
        telemetry.update();
    }

    public void vfMovePar(char rb, ModernRoboticsI2cRangeSensor sensorVar, double bSwitch) { //For Parralel Cryptobox movement;
        // strafes to the correct column using range sensors via our vfValue.
        double centerDis = 26.75; //Distance to the center of the cryptobox.
        double kC = 0;
        switch (rb) { //Determines whether our calculations need to calculate for the red side or blue side.
            case 'r':
                kC = 6;
                break;
            case 'b':
                kC = -6;
                break;
        }
        switch (column) { //Moves accordingly to the variable column.
            case 'L':
                rangeMoveStrafe((centerDis + kC), sensorVar, bSwitch);
                break;
            case 'R':
                rangeMoveStrafe((centerDis - kC), sensorVar, bSwitch);
                break;
            case 'C':case 'U':
                break;
        }
    }

    public void vfMovePerp(char rb, ModernRoboticsI2cRangeSensor sensorVar, double bSwitch) { //For Perpendicular Cryptobox movement;
        // strafes to the correct column using range sensors via our vfValue.
        double centerDis = 26.75; //needs to be tested
        double kC = 0;
        switch (rb) { //Determines whether our calculations need to calculate for the red side or blue side.
            case 'r':
                kC = 6; //need to be adjusted
                break;
            case 'b':
                kC = -6; //need to be adjusted.
                break;
        }
        switch (column) { //Moves accordingly to the variable column.
            case 'L':
                rangeMoveStrafe((centerDis + kC), sensorVar, bSwitch);
                break;
            case 'R':
                rangeMoveStrafe((centerDis - kC), sensorVar, bSwitch);
                break;
            case 'C':case 'U':
                break;
        }
    }

    public void vfMoveAlt() { //For Cryptobox movement; strafes to the correct column using wait times via our vfValue.
        switch (column) { //Moves accordingly to the variable column.
            case 'L':
                setMotorStrafe(-.25);
                sleep(750);
                stopMotors();
                break;
            case 'R':
                setMotorStrafe(.25);
                sleep(675);
                stopMotors();
                break;
            case 'C':case 'U':
                break;
        }
    }

    public void rangeMovePID(double inAway, ModernRoboticsI2cRangeSensor sensorVar) { //Moving forward/backwards using a Range Sensor.
        double sensor = sensorVar.getDistance(DistanceUnit.INCH);
        double pow;

        double localRange;
        while (((sensor < inAway - .35) || (sensor > inAway + .35)) && opModeIsActive() && align) { //While sensor isn't in the desired position, run.
            localRange = sensorVar.getDistance(DistanceUnit.INCH);
            if (gamepad1.x)
                align = false;
            while ((Double.isNaN(localRange) || (localRange > 1000)) && opModeIsActive()) {
                //If a faulty value is detected, don't update our used variable till a good one is found.
                localRange = sensorVar.getDistance(DistanceUnit.INCH);
            }
            sensor = localRange; //Sets all working and usable values into a variable we can utilize.


            pow = .15;

            if (sensor > inAway) { //If the sensor value is greater than the target, move forward.
                setMotors(pow, pow);
            }
            if (sensor < inAway) { //If the sensor value is lower than than the target, move backwards.
                setMotors(-pow, -pow);
            }

            telemetry.addData("SensorValue", sensor); //Optional Telemetry
            telemetry.update();
        }
        stopMotors();
    }

    public void rangeMoveStrafe(double inAway, ModernRoboticsI2cRangeSensor sensorVar, double bSet) { //Moving left/right using a Range Sensor.
        //bSet = Basing Switch | 1 = Left Range Sensor | 0 = Right Range Sensor
        double sensor = sensorVar.getDistance(DistanceUnit.INCH);

//      double range;
        double pow = .15;
//      double newPow;

        double localRange;
        ElapsedTime stopTime = new ElapsedTime(); //Time used to stop the robot from moving too far based on outdated values.

        while (((sensor < inAway - .5) || (sensor > inAway + .5)) && opModeIsActive()) { //While sensor isn't in the desired position, run.
            localRange = sensorVar.getDistance(DistanceUnit.INCH);
            while ((Double.isNaN(localRange) || (localRange > 1000)) && opModeIsActive()) {
                //If a faulty value is detected, don't update our used variable till a good one is found.
                stopTime.reset();
                if (stopTime.milliseconds() > 300 && stopTime.milliseconds() < 600) { //Stops the robot from running off of outdated values for too long.
                    stopMotors();
                }
                localRange = sensorVar.getDistance(DistanceUnit.INCH);
            }
            sensor = localRange; //Sets all working and usable values into a variable we can utilize.
            stopTime.reset();

//          range = Math.abs(inAway - sensor);
            pow = .15;

            //RED SIDE AUTOS - Basing Switch
            if (bSet == 0) {
                if (sensor > inAway) {
                    setMotorStrafe(pow);
                }
                if (sensor < inAway) {
                    setMotorStrafe(-pow);
                }
            }

            //BLUE SIDE AUTOS - Basing Switch
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

    public void turnCorr(double pow, double deg, int timer) throws InterruptedException { //Turns to a desired angle using the IMU.
        if (!opModeIsActive()) //if the OpMode is not active, don't run.
            return;

        double newPow;
        double error;
        double errorMove;

        ElapsedTime time = new ElapsedTime();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currPos = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)); // currPos is the current position

        delay(100);
        time.reset();

        while ((currPos > deg + 1 || currPos < deg - 1) && (time.milliseconds() < timer) && opModeIsActive()) { //While sensor isn't in the desired angle position, run.
            error = deg - currPos; //Finding how far away we are from the target position.
            errorMove = Math.abs(deg - currPos);
            if (error > 180) {
                error = error - 360;
            } else if (error < -180) {
                error = error + 360;
            }

            newPow = pow * (Math.abs(error) / 70); //Using the error to calculate our power.
            if (newPow < .15)
                newPow = .1;

            //The following code allows us to turn in the direction we need, and if we cross the axis
            //at which 180 degrees becomes -180, our robot can turn back in the direction which is closest
            //to the position we wish to be at (We won't make a full rotation to get to -175, if we hit 180).
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

    public void jewelKnockerRed() { //Jewel Knocker code for the autonomous on the red side.
        //Hits the blue jewel.
        jewelArm.setPosition(.6);
        jewelHand.setPosition(.4);
        sleep(500);
        jewelArm.setPosition(.15);
        sleep(850);
        if (jewelColor.red() > jewelColor.blue()) { //Compares the blue and red RGB Color sensor value and hits accordingly.
            jewelHand.setPosition((.3));
        } else if (jewelColor.red() < jewelColor.blue()) {
            jewelHand.setPosition((.6));
        }
        sleep(750);

        jewelArm.setPosition(.6);
        jewelHand.setPosition(.45);
        sleep(300);
        jewelHand.setPosition(.3);
        sleep(300);
    }

    public void jewelKnockerBlue() { //Jewel Knocker code for the autonomous on the blue side.
        //Hits the red jewel.
        jewelArm.setPosition(.55);
        jewelHand.setPosition(.4);
        sleep(500);
        jewelArm.setPosition(.1);
        sleep(850);
        if (jewelColor.red() < jewelColor.blue()) { //Compares the blue and red RGB Color sensor value and hits accordingly.
            jewelHand.setPosition((.3));
        } else if (jewelColor.red() > jewelColor.blue()) {
            jewelHand.setPosition((.6));
        }
        sleep(750);

        jewelArm.setPosition(.55);
        jewelHand.setPosition(.45);
        sleep(300);
        jewelHand.setPosition(.3);
        sleep(300);
    }

    public void slowDown(double reduction) { //Method which reduces the power in our code when called.
        //First Reduction which makes power .5
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
//    public double gyroVal() {
//
//    }

//    public void turnPID(double angle) throws InterruptedException {
//        double kP = .20 / 90;
//        double min = -.2;
//        double max = .2;
//        double changeCon = .04;
//        double PIDchange;
//        double angleDiff = (angle);
//        double oldDiff = angleDiff;
//        int counter = 0;
//        double startAngle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
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

}
