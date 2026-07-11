package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystems.*;
import java.util.List;

public class RobotClass {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public List<LynxModule> hubs;

    //Smaller subsystems with specific methods
    public DriveSubsystem drive;
    public IntakeSubsystem intake;
    public ShooterSubsystem shooter;
    public TurretSubsystem turret;
    public VisionSubsystem vision;
    public MiscSubsystems miscSubsystems;

    public RobotClass(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        miscSubsystems = new MiscSubsystems(this);
        vision = new VisionSubsystem(this);
        drive = new DriveSubsystem(this);
        intake = new IntakeSubsystem(this);
        shooter = new ShooterSubsystem(this);
        turret = new TurretSubsystem(this);
    }

    public void init() {
        miscSubsystems.init();
        vision.init();
        drive.init();
        intake.init();
        shooter.init();
        turret.init();
    }

    public void initLoop() {
        drive.initLoop();
    }

    public void start() {
        intake.start();
        turret.start();
    }

    public void update() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        vision.update();
        drive.update();
        intake.update();
        shooter.update();
        turret.update();
        miscSubsystems.update();
    }

    public void telemetry() {
        miscSubsystems.telemetry();
        drive.telemetry();
        intake.telemetry();
        shooter.telemetry();
        turret.telemetry();
        telemetry.update();
    }

    public void stop() {
        miscSubsystems.stop();
        intake.stop();
        shooter.stop();
    }
}
