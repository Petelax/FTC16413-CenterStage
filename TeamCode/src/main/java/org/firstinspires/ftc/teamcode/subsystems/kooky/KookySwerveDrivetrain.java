package org.firstinspires.ftc.teamcode.subsystems.kooky;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants;
import org.firstinspires.ftc.teamcode.subsystems.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.wpilib.MathUtil;

public class KookySwerveDrivetrain {
    private static final boolean USE_WHEEL_FEEDFORWARD = false;
    public KookySwerveModule frontLeftModule, backLeftModule, backRightModule, frontRightModule;
    public KookySwerveModule[] modules;

    public static double TRACK_WIDTH = 9, WHEEL_BASE = 9;
    private final double R;
    public static double frontLeftOffset = 2, frontRightOffset = 0, backLeftOffset = 0, backRightOffset = -0.055;

    public static boolean maintainHeading = false;

    double[] ws = new double[4];
    double[] wa = new double[4];
    double max = 0.0;

    public final double minPow = 0.1;
    public static double imuOffset = 0.0;

    private boolean locked = false;

    public KookySwerveDrivetrain(HardwareMap hardwareMap) {


        RevIMU gyro = new RevIMU(hardwareMap);
        gyro.init();
        gyro.reset();

        CRServo servoLF = hardwareMap.get(CRServo.class, "servoLF");
        CRServo servoRF = hardwareMap.get(CRServo.class, "servoRF");
        CRServo servoLR = hardwareMap.get(CRServo.class, "servoLR");
        CRServo servoRR = hardwareMap.get(CRServo.class, "servoRR");

        DcMotorEx motorLF = hardwareMap.get(DcMotorEx.class, "motorLF");
        DcMotorEx motorRF = hardwareMap.get(DcMotorEx.class, "motorRF");
        DcMotorEx motorLR = hardwareMap.get(DcMotorEx.class, "motorLR");
        DcMotorEx motorRR = hardwareMap.get(DcMotorEx.class, "motorRR");

        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        AbsoluteAnalogEncoder encoderLF = new AbsoluteAnalogEncoder(hardwareMap, DrivebaseConstants.DeviceIDs.LF_ENCODER, DrivebaseConstants.Measurements.LF_OFFSET);
        AbsoluteAnalogEncoder encoderRF = new AbsoluteAnalogEncoder(hardwareMap, DrivebaseConstants.DeviceIDs.RF_ENCODER, DrivebaseConstants.Measurements.RF_OFFSET);
        AbsoluteAnalogEncoder encoderLR = new AbsoluteAnalogEncoder(hardwareMap, DrivebaseConstants.DeviceIDs.LR_ENCODER, DrivebaseConstants.Measurements.LR_OFFSET);
        AbsoluteAnalogEncoder encoderRR = new AbsoluteAnalogEncoder(hardwareMap, DrivebaseConstants.DeviceIDs.RR_ENCODER, DrivebaseConstants.Measurements.RR_OFFSET);

        frontLeftModule = new KookySwerveModule(motorLF, servoLF, encoderLF);
        backLeftModule = new KookySwerveModule(motorRF, servoRF, encoderRF);
        backRightModule = new KookySwerveModule(motorLR, servoLR, encoderLR);
        frontRightModule = new KookySwerveModule(motorRR, servoRR, encoderRR);

        modules = new KookySwerveModule[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (KookySwerveModule m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R = hypot(TRACK_WIDTH, WHEEL_BASE);
    }

    public void read() {
        for (KookySwerveModule module : modules) module.read();
    }

    public void set(Pose2d pose) {
        double x = pose.getX(), y = pose.getY(), head = pose.getHeading();

        double a = x - head * (WHEEL_BASE / R),
                b = x + head * (WHEEL_BASE / R),
                c = y - head * (TRACK_WIDTH / R),
                d = y + head * (TRACK_WIDTH / R);

        if (locked) {
            ws = new double[]{0, 0, 0, 0};
            wa = new double[]{Math.PI / 4, -Math.PI / 4, Math.PI / 4, -Math.PI / 4};
        } else {
            ws = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
            if (!maintainHeading) wa = new double[]{atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};
        }

        max = MathUtil.max(ws);
    }

    public void write() {
        for (int i = 0; i < 4; i++) {
            KookySwerveModule m = modules[i];
            if (Math.abs(max) > 1) ws[i] /= max;
            m.setMotorPower(Math.abs(ws[i]) + ((USE_WHEEL_FEEDFORWARD) ? minPow * Math.signum(ws[i]) : 0));
            m.setTargetRotation(MathUtil.norm(wa[i]));
        }
    }

    public void updateModules() {
        for (KookySwerveModule m : modules) m.update();
    }

    public void setLocked(boolean locked){
        this.locked = locked;
    }

    public boolean isLocked(){
        return locked;
    }

    public String getTelemetry() {
        return frontLeftModule.getTelemetry("leftFrontModule") + "\n" +
                backLeftModule.getTelemetry("leftRearModule") + "\n" +
                frontRightModule.getTelemetry("rightFrontModule") + "\n" +
                backRightModule.getTelemetry("rightRearModule") + "\n";
    }
}
