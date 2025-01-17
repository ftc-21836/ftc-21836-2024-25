package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.opmode.BasketAutonAction.State.DRIVING_TO_SUB;
import static org.firstinspires.ftc.teamcode.opmode.BasketAutonAction.State.INTAKING_2;
import static org.firstinspires.ftc.teamcode.opmode.BasketAutonAction.State.INTAKING_3;
import static org.firstinspires.ftc.teamcode.opmode.BasketAutonAction.State.PARKING;
import static org.firstinspires.ftc.teamcode.opmode.BasketAutonAction.State.PRELOAD_AND_1;
import static org.firstinspires.ftc.teamcode.opmode.BasketAutonAction.State.SCORING;
import static org.firstinspires.ftc.teamcode.opmode.BasketAutonAction.State.SCORING_1;
import static org.firstinspires.ftc.teamcode.opmode.BasketAutonAction.State.SCORING_2;
import static org.firstinspires.ftc.teamcode.opmode.BasketAutonAction.State.SWEEPING;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.EXTEND_SUB_MAX;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.EXTEND_SUB_MIN;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.TIME_EXTEND_CYCLE;
import static org.firstinspires.ftc.teamcode.opmode.MainAuton.WAIT_POST_INTAKING;

import static java.lang.Math.PI;
import static java.lang.Math.cos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

import java.util.ArrayList;

class BasketAutonAction implements Action {

    enum State {
        PRELOAD_AND_1,
        SCORING_1,
        INTAKING_2,
        SCORING_2,
        INTAKING_3,
        SCORING,
        DRIVING_TO_SUB,
        SWEEPING,
        PARKING
    }

    private State state = PRELOAD_AND_1;

    private ElapsedTime matchTimer = null;
    private final ElapsedTime timer = new ElapsedTime();

    private MecanumDrive.FollowTrajectoryAction activeTraj;

    private final Robot robot;
    private final MecanumDrive.FollowTrajectoryAction
            score1,
            intake2,
            score2,
            intake3,
            score3,
            park,
            i1To2,
            i2To3,
            i3ToSub,
            subPark;

    private final ArrayList<MecanumDrive.FollowTrajectoryAction>
            toSubs = new ArrayList<>(),
            sweepLefts = new ArrayList<>(),
            sweepRights = new ArrayList<>(),
            scores = new ArrayList<>();

    private final double scoreTime, cycleTime;

    private boolean sweepingLeft = true;

    BasketAutonAction(
            Robot robot,
            MecanumDrive.FollowTrajectoryAction preloadAnd1,
            MecanumDrive.FollowTrajectoryAction score1,
            MecanumDrive.FollowTrajectoryAction intake2,
            MecanumDrive.FollowTrajectoryAction score2,
            MecanumDrive.FollowTrajectoryAction intake3,
            MecanumDrive.FollowTrajectoryAction score3,
            MecanumDrive.FollowTrajectoryAction park,
            MecanumDrive.FollowTrajectoryAction i1To2,
            MecanumDrive.FollowTrajectoryAction i2To3,
            MecanumDrive.FollowTrajectoryAction i3ToSub,
            MecanumDrive.FollowTrajectoryAction subPark,
            TrajectoryActionBuilder toSub,
            TrajectoryActionBuilder sweepLeft,
            TrajectoryActionBuilder sweepRight,
            TrajectoryActionBuilder score
    ) {
        this.robot = robot;
        activeTraj = preloadAnd1;
        this.score1 = score1;
        this.intake2 = intake2;
        this.score2 = score2;
        this.intake3 = intake3;
        this.score3 = score3;
        this.park = park;
        this.i1To2 = i1To2;
        this.i2To3 = i2To3;
        this.i3ToSub = i3ToSub;
        this.subPark = subPark;

        for (int i = 0; i < 4; i++) {
            this.toSubs.add((MecanumDrive.FollowTrajectoryAction) toSub.build());
            this.sweepLefts.add((MecanumDrive.FollowTrajectoryAction) sweepLeft.build());
            this.sweepRights.add((MecanumDrive.FollowTrajectoryAction) sweepRight.build());
            this.scores.add((MecanumDrive.FollowTrajectoryAction) score.build());
        }

        scoreTime = scores.get(0).timeTrajectory.duration;
        cycleTime = toSubs.get(0).timeTrajectory.duration + sweepLefts.get(0).timeTrajectory.duration + scoreTime;

    }

    /// <a href="https://www.desmos.com/calculator/iazwdw6hky">Graph</a>
    private void extend(double t) {
        robot.intake.extendo.setTarget(EXTEND_SUB_MIN + (EXTEND_SUB_MAX - EXTEND_SUB_MIN) * (1 - cos(2 * PI * t / TIME_EXTEND_CYCLE)) / 2);
    }

    public boolean run(@NonNull TelemetryPacket p) {
        if (matchTimer == null) matchTimer = new ElapsedTime();

        double elapsed = matchTimer.seconds();
        double remaining = 30 - elapsed;

        boolean trajDone = !activeTraj.run(p);

        boolean hasSample = robot.intake.hasSample();

        switch (state) {
            case PRELOAD_AND_1:

                // Sample intaked
                if (!hasSample) {

                    timer.reset();

                    // skip to 2 if didn't get 1
                    if (trajDone) {
                        activeTraj = i1To2;
                        state = INTAKING_2;
                    }

                } else {
                    robot.intake.runRoller(1);
                    if (timer.seconds() >= WAIT_POST_INTAKING) {
                        robot.intake.runRoller(0);
                        activeTraj = score1;
                        state = SCORING_1;
                    }
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
                if (!hasSample) {

                    timer.reset();

                    // skip to 3 if didn't get 2
                    if (trajDone) {
                        activeTraj = i2To3;
                        state = INTAKING_3;
                    }

                } else {
                    robot.intake.runRoller(1);
                    if (timer.seconds() >= WAIT_POST_INTAKING) {
                        robot.intake.runRoller(0);
                        activeTraj = score2;
                        state = SCORING_2;
                    }
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
                if (!hasSample) {

                    timer.reset();

                    // skip to sub if didn't get 3
                    if (trajDone) {
                        activeTraj = i3ToSub;
                        state = DRIVING_TO_SUB;
                    }

                } else {
                    robot.intake.runRoller(1);
                    if (timer.seconds() >= WAIT_POST_INTAKING) {
                        robot.intake.runRoller(0);
                        activeTraj = score3;
                        state = SCORING;
                    }
                }

                break;

            case SCORING:
                if (trajDone) {
                    if (remaining < cycleTime) {
                        activeTraj = park;
                        state = PARKING;
                    } else {
                        activeTraj = toSubs.remove(0);
                        state = DRIVING_TO_SUB;
                    }
                }
                break;

            case DRIVING_TO_SUB:
                if (trajDone) {
                    sweepingLeft = true;
                    activeTraj = sweepLefts.remove(0);
                    state = SWEEPING;
                }
                break;
            case SWEEPING:

                if (remaining < scoreTime) {
                    activeTraj = subPark;
                    state = PARKING;
                    break;
                }

                // Sample intaked
                if (!hasSample) {

                    extend(elapsed);

                    timer.reset();

                    // sweep the other way
                    if (trajDone) {
                        sweepingLeft = !sweepingLeft;
                        activeTraj = (sweepingLeft ? sweepRights : sweepLefts).remove(0);
                    }

                } else {
                    robot.intake.runRoller(1);
                    if (timer.seconds() >= WAIT_POST_INTAKING) {
                        robot.intake.runRoller(0);
                        activeTraj = scores.remove(0);
                        state = SCORING;
                    }
                }

                break;

            case PARKING:
                return !trajDone;
        }

        return true;
    }
}
