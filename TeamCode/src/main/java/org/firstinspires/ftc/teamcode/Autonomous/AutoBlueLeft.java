package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Library.MyOpMode;

@Autonomous(name = "AutoBlueLeft", group = "Linear Opmode")
public class AutoBlueLeft extends MyOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hMap(hardwareMap);
        align = true;
        // Set up our telemetry dashboard
//        composeTelemetry();

        // Wait until we're told to go
        waitForStart();
        runtime.reset();
/**---------------------------------------------------------------------------------------------------------------*/
        vfValue();
        jewelKnockerBlue();

        setMotors(-.25, -.25);
        sleep(1550);
        stopMotors();

        rangeMoveStrafe(26.25, rangeR,0);
        sleep(350);

        try {
            turnCorr(.25, -178, 5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        sleep(1000);
        telemetry.addData(formatAngle(angles.angleUnit, angles.firstAngle),"Angle");
        vfMovePar('b',rangeL, 1);

        rangeMovePID(6, rangeF);

        manipAuto(-.75);
        sleep(500);

        manipAuto(-.75);

//back up, push forward, back up
        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();

        manip.setPower(-1);
        setMotors(.4, .4);
        sleep(250);
        stopMotors();

        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();
        manip.setPower(0);
    }
}