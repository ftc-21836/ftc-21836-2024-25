package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.AngularRampLogger;
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.ForwardPushTest;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.LateralPushTest;
import com.acmerobotics.roadrunner.ftc.LateralRampLogger;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class TuningOpModes {
    // TODO: change this to TankDrive.class if you're using tank
    public static final Class<?> DRIVE_CLASS = MecanumDrive.class;

    public static final String GROUP = "quickstart";
    public static final boolean DISABLED = false;

    private TuningOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        DriveViewFactory dvf;
        if (DRIVE_CLASS.equals(MecanumDrive.class)) {
            dvf = hardwareMap -> {
                MecanumDrive md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
//                if (md.localizer instanceof MecanumDrive.DriveLocalizer) {
//                    MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) md.localizer;
//                    leftEncs.add(dl.leftFront);
//                    leftEncs.add(dl.leftBack);
//                    rightEncs.add(dl.rightFront);
//                    rightEncs.add(dl.rightBack);
//                } else if (md.localizer instanceof ThreeDeadWheelLocalizer) {
//                    ThreeDeadWheelLocalizer dl = (ThreeDeadWheelLocalizer) md.localizer;
//                    parEncs.add(dl.par0);
//                    parEncs.add(dl.par1);
//                    perpEncs.add(dl.perp);
//                } else if (md.localizer instanceof TwoDeadWheelLocalizer) {
//                    TwoDeadWheelLocalizer dl = (TwoDeadWheelLocalizer) md.localizer;
//                    parEncs.add(dl.par);
//                    perpEncs.add(dl.perp);
//                } else {
//                    throw new RuntimeException("unknown localizer: " + md.localizer.getClass().getName());
//                }
                RawEncoder forward = new RawEncoder(new DcMotorEx() {
                    @Override public void setMotorEnable() {
                }
                @Override public void setMotorDisable() {
                }
                @Override public boolean isMotorEnabled() {return false;}
                    @Override public void setVelocity(double angularRate) {
                }
                @Override public void setVelocity(double angularRate, AngleUnit unit) {
                }
                @Override public double getVelocity() {
                    return md.localizer.velX() * PinpointLocalizer.TICKS_PER_MM;
                }
                @Override public double getVelocity(AngleUnit unit) {return 0;}
                @Override public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
                }
                @Override public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
                }
                @Override public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
                }
                @Override public void setPositionPIDFCoefficients(double p) {
                }
                @Override public PIDCoefficients getPIDCoefficients(RunMode mode) {return null;}
                    @Override public PIDFCoefficients getPIDFCoefficients(RunMode mode) {return null;}
                    @Override public void setTargetPositionTolerance(int tolerance) {
                }
                @Override public int getTargetPositionTolerance() {return 0;}
                    @Override public double getCurrent(CurrentUnit unit) {return 0;}
                    @Override public double getCurrentAlert(CurrentUnit unit) {return 0;}
                    @Override public void setCurrentAlert(double current, CurrentUnit unit) {
                }
                @Override public boolean isOverCurrent() {return false;}
                    @Override public MotorConfigurationType getMotorType() {return null;}
                    @Override public void setMotorType(MotorConfigurationType motorType) {
                }
                @Override public DcMotorController getController() {return md.rightBack.motor.getController();}
                    @Override public int getPortNumber() {return 0;}
                    @Override public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
                }
                @Override public ZeroPowerBehavior getZeroPowerBehavior() {return null;}
                    @Override public void setPowerFloat() {
                }
                @Override public boolean getPowerFloat() {return false;}
                    @Override public void setTargetPosition(int position) {
                }
                @Override public int getTargetPosition() {return 0;}
                    @Override public boolean isBusy() {return false;}

                    @Override
                    public int getCurrentPosition() {
                        md.localizer.update();
                        return md.localizer.rawEncoderX();
                    }
                    @Override public void setMode(RunMode mode) {}@Override public RunMode getMode() {return null;}@Override public void setDirection(Direction direction) {}@Override public Direction getDirection() {return null;}@Override public void setPower(double power) {}@Override public double getPower() {return 0;}@Override public Manufacturer getManufacturer() {return null;}@Override public String getDeviceName() {return "";}@Override public String getConnectionInfo() {return "";}@Override public int getVersion() {return 0;}@Override public void resetDeviceConfigurationForOpMode() {}@Override public void close() {}
                });
                RawEncoder perp = new RawEncoder(new DcMotorEx() {
                    @Override public void setMotorEnable() {
                    }
                    @Override public void setMotorDisable() {
                    }
                    @Override public boolean isMotorEnabled() {return false;}
                    @Override public void setVelocity(double angularRate) {
                    }
                    @Override public void setVelocity(double angularRate, AngleUnit unit) {
                    }
                    @Override public double getVelocity() {
                        return md.localizer.velY() * PinpointLocalizer.TICKS_PER_MM;
                    }
                    @Override public double getVelocity(AngleUnit unit) {return 0;}
                    @Override public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
                    }
                    @Override public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
                    }
                    @Override public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
                    }
                    @Override public void setPositionPIDFCoefficients(double p) {
                    }
                    @Override public PIDCoefficients getPIDCoefficients(RunMode mode) {return null;}
                    @Override public PIDFCoefficients getPIDFCoefficients(RunMode mode) {return null;}
                    @Override public void setTargetPositionTolerance(int tolerance) {
                    }
                    @Override public int getTargetPositionTolerance() {return 0;}
                    @Override public double getCurrent(CurrentUnit unit) {return 0;}
                    @Override public double getCurrentAlert(CurrentUnit unit) {return 0;}
                    @Override public void setCurrentAlert(double current, CurrentUnit unit) {
                    }
                    @Override public boolean isOverCurrent() {return false;}
                    @Override public MotorConfigurationType getMotorType() {return null;}
                    @Override public void setMotorType(MotorConfigurationType motorType) {
                    }
                    @Override public DcMotorController getController() {return md.rightFront.motor.getController();}
                    @Override public int getPortNumber() {return 0;}
                    @Override public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
                    }
                    @Override public ZeroPowerBehavior getZeroPowerBehavior() {return null;}
                    @Override public void setPowerFloat() {
                    }
                    @Override public boolean getPowerFloat() {return false;}
                    @Override public void setTargetPosition(int position) {
                    }
                    @Override public int getTargetPosition() {return 0;}
                    @Override public boolean isBusy() {return false;}

                    @Override
                    public int getCurrentPosition() {
                        md.localizer.update();
                        return md.localizer.rawEncoderY();
                    }
                    @Override public void setMode(RunMode mode) {}@Override public RunMode getMode() {return null;}@Override public void setDirection(Direction direction) {}@Override public Direction getDirection() {return null;}@Override public void setPower(double power) {}@Override public double getPower() {return 0;}@Override public Manufacturer getManufacturer() {return null;}@Override public String getDeviceName() {return "";}@Override public String getConnectionInfo() {return "";}@Override public int getVersion() {return 0;}@Override public void resetDeviceConfigurationForOpMode() {}@Override public void close() {}
                });

                parEncs.add(forward);
                perpEncs.add(perp);

                return new DriveView(
                    DriveType.MECANUM,
                        MecanumDrive.PARAMS.inPerTick,
                        MecanumDrive.PARAMS.maxWheelVel,
                        MecanumDrive.PARAMS.minProfileAccel,
                        MecanumDrive.PARAMS.maxProfileAccel,
                        hardwareMap.getAll(LynxModule.class),
                        Arrays.asList(
                                md.leftFront.motor,
                                md.leftBack.motor
                        ),
                        Arrays.asList(
                                md.rightFront.motor,
                                md.rightBack.motor
                        ),
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        md.lazyImu,
                        md.voltageSensor,
                        () -> new MotorFeedforward(MecanumDrive.PARAMS.kS,
                                MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                                MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick)
                );
            };
        } else if (DRIVE_CLASS.equals(TankDrive.class)) {
            dvf = hardwareMap -> {
                TankDrive td = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

                List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
                if (td.localizer instanceof TankDrive.DriveLocalizer) {
                    TankDrive.DriveLocalizer dl = (TankDrive.DriveLocalizer) td.localizer;
                    leftEncs.addAll(dl.leftEncs);
                    rightEncs.addAll(dl.rightEncs);
                } else if (td.localizer instanceof ThreeDeadWheelLocalizer) {
                    ThreeDeadWheelLocalizer dl = (ThreeDeadWheelLocalizer) td.localizer;
                    parEncs.add(dl.par0);
                    parEncs.add(dl.par1);
                    perpEncs.add(dl.perp);
                } else if (td.localizer instanceof TwoDeadWheelLocalizer) {
                    TwoDeadWheelLocalizer dl = (TwoDeadWheelLocalizer) td.localizer;
                    parEncs.add(dl.par);
                    perpEncs.add(dl.perp);
                } else {
                    throw new RuntimeException("unknown localizer: " + td.localizer.getClass().getName());
                }

                return new DriveView(
                    DriveType.TANK,
                        TankDrive.PARAMS.inPerTick,
                        TankDrive.PARAMS.maxWheelVel,
                        TankDrive.PARAMS.minProfileAccel,
                        TankDrive.PARAMS.maxProfileAccel,
                        hardwareMap.getAll(LynxModule.class),
                        Arrays.asList(td.leftMotors.get(0).motor),
                        Arrays.asList(td.rightMotors.get(0).motor),
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        td.lazyImu,
                        td.voltageSensor,
                        () -> new MotorFeedforward(TankDrive.PARAMS.kS,
                                TankDrive.PARAMS.kV / TankDrive.PARAMS.inPerTick,
                                TankDrive.PARAMS.kA / TankDrive.PARAMS.inPerTick)
                );
            };
        } else {
            throw new RuntimeException();
        }

        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));
        manager.register(metaForClass(DeadWheelDirectionDebugger.class), new DeadWheelDirectionDebugger(dvf));

//        manager.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
//        manager.register(metaForClass(SplineTest.class), SplineTest.class);
//        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
