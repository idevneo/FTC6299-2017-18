package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Library.MyOpMode;

@Autonomous(name = "AltBlueLeft", group = "Linear Opmode")
@Disabled
public class AltBlueLeft extends MyOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hMap(hardwareMap);
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
//        vfMovePar('b',rangeL, 1);
        setMotorStrafe(-5);
        sleep(1000);
        stopMotors();
    }
}