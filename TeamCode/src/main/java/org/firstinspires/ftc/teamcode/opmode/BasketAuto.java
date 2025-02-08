package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.opmode.Auto.WAIT_DROP_TO_EXTEND;
import static org.firstinspires.ftc.teamcode.opmode.Auto.WAIT_EXTEND_POST_SWEEPER;
import static org.firstinspires.ftc.teamcode.opmode.Auto.WAIT_INTAKE_RETRACT_POST_SUB;
import static org.firstinspires.ftc.teamcode.opmode.Auto.WAIT_POST_INTAKING_SUB;
import static org.firstinspires.ftc.teamcode.opmode.Auto.basket2;
import static org.firstinspires.ftc.teamcode.opmode.Auto.intakingSub;
import static org.firstinspires.ftc.teamcode.opmode.Auto.parkLeft;
import static org.firstinspires.ftc.teamcode.opmode.Auto.scoreSample;
import static org.firstinspires.ftc.teamcode.opmode.Auto.sweptSub;
import static org.firstinspires.ftc.teamcode.opmode.BasketAuto.State.DRIVING_TO_SUB;
import static org.firstinspires.ftc.teamcode.opmode.BasketAuto.State.INTAKING_2;
import static org.firstinspires.ftc.teamcode.opmode.BasketAuto.State.INTAKING_3;
import static org.firstinspires.ftc.teamcode.opmode.BasketAuto.State.PARKING;
import static org.firstinspires.ftc.teamcode.opmode.BasketAuto.State.PRELOAD_AND_1;
import static org.firstinspires.ftc.teamcode.opmode.BasketAuto.State.SCORING;
import static org.firstinspires.ftc.teamcode.opmode.BasketAuto.State.SCORING_1;
import static org.firstinspires.ftc.teamcode.opmode.BasketAuto.State.SCORING_2;
import static org.firstinspires.ftc.teamcode.opmode.BasketAuto.State.INTAKING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.EXTEND_SUB_MAX;
import static org.firstinspires.ftc.teamcode.opmode.Auto.EXTEND_SUB_MIN;
import static org.firstinspires.ftc.teamcode.opmode.Auto.TIME_CYCLE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.TIME_EXTEND_CYCLE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.TIME_SCORE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.SPEED_INTAKING;

import static java.lang.Math.PI;
import static java.lang.Math.cos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

class BasketAuto implements Action {

    enum State {
        PRELOAD_AND_1,
        SCORING_1,
        INTAKING_2,
        SCORING_2,
        INTAKING_3,
        SCORING,
        DRIVING_TO_SUB,
        INTAKING,
        PARKING
    }

    private State state = PRELOAD_AND_1;

    private ElapsedTime matchTimer = null;
    private final ElapsedTime extendoTimer = new ElapsedTime();

    private Action activeTraj;

    private EditablePose sub = intakingSub;
    private double subExtend = EXTEND_SUB_MAX;

    private final Robot robot;
    private final Action
            score1,
            intake2,
            score2,
            intake3,
            score3,
            park,
            i1To2,
            i2To3,
            i3ToSub,
            toSub;

    private final MinVelConstraint sweepConstraint;

    BasketAuto(
            Robot robot,
            Action preloadAnd1,
            Action score1,
            Action intake2,
            Action score2,
            Action intake3,
            Action score3,
            Action i1To2,
            Action i2To3,
            Action i3ToSub,
            Action toSub,
            Action park,
            MinVelConstraint sweepConstraint
    ) {
        this.robot = robot;
        activeTraj = preloadAnd1;
        this.score1 = score1;
        this.intake2 = intake2;
        this.score2 = score2;
        this.intake3 = intake3;
        this.score3 = score3;
        this.i1To2 = i1To2;
        this.i2To3 = i2To3;
        this.i3ToSub = i3ToSub;
        this.toSub = toSub;
        this.park = park;
        this.sweepConstraint = sweepConstraint;
    }

    public boolean run(@NonNull TelemetryPacket p) {
        if (matchTimer == null) matchTimer = new ElapsedTime();

        double remaining = 30 - matchTimer.seconds();

        boolean trajDone = !activeTraj.run(p);

        switch (state) {
            case PRELOAD_AND_1:

                // Sample intaked
                if (robot.intake.hasSample()) {
                    activeTraj = score1;
                    state = SCORING_1;
                } else if (trajDone) { // skip to 2 if didn't get 1
                    activeTraj = i1To2;
                    state = INTAKING_2;
                }

                break;

            case SCORING_1:
                if (trajDone) {
                    activeTraj = intake2;
                    state = INTAKING_2;
                }
                break;
            case INTAKING_2:

                // Sample intaked
                if (robot.intake.hasSample()) {
                    activeTraj = score2;
                    state = SCORING_2;
                } else if (trajDone) { // skip to 3 if didn't get 2
                    activeTraj = i2To3;
                    state = INTAKING_3;
                }

                break;

            case SCORING_2:
                if (trajDone) {
                    activeTraj = intake3;
                    state = INTAKING_3;
                }
                break;
            case INTAKING_3:

                // Sample intaked
                if (robot.intake.hasSample()) {
                    activeTraj = score3;
                    state = SCORING;
                } else if (trajDone) { // skip to sub if didn't get 3
                    activeTraj = i3ToSub;
                    state = DRIVING_TO_SUB;
                }

                break;

            case SCORING:
                if (trajDone) {
                    if (remaining < TIME_CYCLE) {
                        activeTraj = park;
                        state = PARKING;
                    } else {
                        activeTraj = sub == intakingSub ? toSub :
                                robot.drivetrain.actionBuilder(basket2.toPose2d())
                                    .setTangent(basket2.heading)
                                    .splineTo(sub.toVector2d(), sub.heading)
                                    .afterTime(0, () -> robot.intake.extendo.setTarget(subExtend))
                                    .waitSeconds(WAIT_EXTEND_POST_SWEEPER)
                                    .afterTime(0, () -> robot.intake.runRoller(SPEED_INTAKING))
                                    .waitSeconds(WAIT_DROP_TO_EXTEND)
                                    .build();
                        state = DRIVING_TO_SUB;
                    }
                }
                break;

            case DRIVING_TO_SUB:
                if (trajDone) {
                    activeTraj = robot.drivetrain.actionBuilder(sub.toPose2d())
                            .lineToY(sub.y > 0 ? intakingSub.y : sweptSub.y, sweepConstraint)
                            .build();
                    state = INTAKING;
                    extendoTimer.reset();
                }
                break;
            case INTAKING:

                if (remaining < TIME_SCORE) {
                    activeTraj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                            .afterTime(0, () -> {
                                robot.intake.ejectSample();
                                robot.intake.runRoller(0);
                                Deposit.level1Ascent = true;
                                robot.deposit.lift.setTarget(0);
                            })
                            .afterTime(1, () -> robot.intake.extendo.setExtended(false))
                            .strafeToSplineHeading(parkLeft.toVector2d(), parkLeft.heading)
                            .build();
                    state = PARKING;
                    break;
                }

                // Sample intaked
                if (robot.intake.hasSample()) {

                    sub = new EditablePose(robot.drivetrain.pose);
                    subExtend = robot.intake.extendo.getPosition();

                    activeTraj = robot.drivetrain.actionBuilder(sub.toPose2d())
                            .afterTime(0, () -> robot.intake.runRoller(1))
                            .waitSeconds(WAIT_POST_INTAKING_SUB)
                            .afterTime(0, () -> robot.intake.runRoller(0))
                            .setTangent(PI + sub.heading)
                            .waitSeconds(WAIT_INTAKE_RETRACT_POST_SUB)
                            .splineTo(basket2.toVector2d(), PI + basket2.heading)
                            .stopAndAdd(scoreSample(robot))
                            .build();

                    state = SCORING;
                } else {

                    /// <a href="https://www.desmos.com/calculator/2jddu08h7f">Graph</a>
                    robot.intake.extendo.setTarget(
                            EXTEND_SUB_MIN + (EXTEND_SUB_MAX - EXTEND_SUB_MIN) * (1 + cos(2 * PI * extendoTimer.seconds() / TIME_EXTEND_CYCLE)) / 2
                    );

                    // sweep the other way
                    if (trajDone) activeTraj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                            .lineToY(robot.drivetrain.pose.position.y > 0 ? intakingSub.y : sweptSub.y, sweepConstraint)
                            .build();

                }

                break;

            case PARKING:
                return !trajDone;
        }

        return true;
    }
}
