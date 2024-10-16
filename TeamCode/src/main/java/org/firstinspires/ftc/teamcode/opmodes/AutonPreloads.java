package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.AutonCycles.score;
import static org.firstinspires.ftc.teamcode.opmodes.AutonCycles.stackPos;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.ANGLE_INNER_SPIKE_AUDIENCE_APPROACH;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.PARTNER_WAIT;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.SPEED_INTAKE_STACK_APPROACH;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.SPEED_INTAKING;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.TIME_INTAKING;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.TIME_PRE_SPIKE_AUDIENCE_PAINTBRUSH;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.TIME_PRE_YELLOW;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.TIME_SPIKE_AUDIENCE;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.TIME_SPIKE_BACKDROP;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.TIME_SPIKE_TO_INTAKE_FLIP;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.X_SHIFT_PRE_STACK_AUDIENCE_INNER_SPIKE;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.Y_SHIFT_POST_INNER_SPIKE_BACKDROP;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.centerSpikeAudience;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.centerSpikeBackdrop;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.innerSpikeAudience;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.innerSpikeBackdrop;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.outerSpikeAudience;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.outerSpikeBackdrop;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.FORWARD;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.LEFT;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.REVERSE;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.RIGHT;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.autonBackdrop;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.toPose2d;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Lift.ROW_FLOOR_SCORING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_SECOND;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FIVE_STACK;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;

final class AutonPreloads {

    static void backdropPreloads(TrajectorySequenceBuilder sequence, ArrayList<Pixel> placements, double a, boolean outer, boolean inner) {
        if (inner) {
            Pose2d spike = innerSpikeBackdrop.byAlliance().toPose2d();
            sequence
                    .splineTo(spike.vec(), spike.getHeading())
                    .strafeRight(Y_SHIFT_POST_INNER_SPIKE_BACKDROP * a)
            ;
        } else {
            sequence.lineToSplineHeading((
                    outer ?
                            outerSpikeBackdrop.byAlliance() :
                            centerSpikeBackdrop.byAlliance()
            ).toPose2d());
        }

        sequence
                .addTemporalMarker(() -> {
                    robot.spike.toggle();
                })
                .waitSeconds(TIME_SPIKE_BACKDROP)
                .setTangent(RIGHT)
                .UNSTABLE_addTemporalMarkerOffset(TIME_SPIKE_TO_INTAKE_FLIP, () -> {
                    robot.deposit.lift.setTargetRow(placements.get(0).y);
                })
        ;
        Pose2d yellow = toPose2d(placements.get(0));
        Vector2d distance = new Vector2d(1.0, 0.0);
        if (!outer) sequence.splineTo(yellow.vec().minus(distance), RIGHT).lineTo(yellow.vec());
        else sequence.lineTo(yellow.vec().minus(distance)).lineTo(yellow.vec());
        sequence
                .waitSeconds(TIME_PRE_YELLOW)
                .addTemporalMarker(() -> {
                    robot.deposit.paintbrush.dropPixel();
                    autonBackdrop.add(placements.get(0));
                })
                .waitSeconds(TIME_DROP_SECOND)
        ;
    }

    static void audiencePreloadsAndWhite(TrajectorySequenceBuilder sequence, ArrayList<Pixel> placements, double a, boolean outer, boolean inner) {

        sequence
                .addTemporalMarker(() -> {
                    robot.intake.setExtended(true);
                })
                .UNSTABLE_addTemporalMarkerOffset(TIME_PRE_SPIKE_AUDIENCE_PAINTBRUSH, () -> {
                    robot.deposit.lift.setTargetRow(ROW_FLOOR_SCORING);
                })
        ;

        Pose2d stack = stackPos(1);

        if (inner) {

            EditablePose preStack = new EditablePose(stack).clone();
            preStack.x += X_SHIFT_PRE_STACK_AUDIENCE_INNER_SPIKE;
            Pose2d spike = innerSpikeAudience.byAlliance().toPose2d();
            sequence
                    .setTangent(a * (REVERSE - ANGLE_INNER_SPIKE_AUDIENCE_APPROACH))
                    .splineToSplineHeading(spike, a * ANGLE_INNER_SPIKE_AUDIENCE_APPROACH)
//                    .strafeLeft(a * 2)
                    .addTemporalMarker(() -> {
                        robot.deposit.paintbrush.dropPixel();
                    })
                    .waitSeconds(TIME_SPIKE_AUDIENCE)
                    .addTemporalMarker(() -> {
                        robot.deposit.lift.setTargetRow(-1);
                    })
                    .lineToSplineHeading(preStack.toPose2d())
            ;

        } else if (outer) {

            Pose2d spike = outerSpikeAudience.byAlliance().toPose2d();
            sequence
                    .splineTo(spike.vec(), spike.getHeading())
                    .addTemporalMarker(() -> {
                        robot.deposit.paintbrush.dropPixel();
                    })
                    .waitSeconds(TIME_SPIKE_AUDIENCE)
                    .addTemporalMarker(() -> {
                        robot.deposit.lift.setTargetRow(-1);
                    })
                    .setTangent(a * FORWARD)
                    .turn(a * FORWARD)
            ;

        } else {

            Pose2d spike = centerSpikeAudience.byAlliance().toPose2d();
            sequence
                    .forward(10)
                    .splineTo(spike.vec(), spike.getHeading())
                    .addTemporalMarker(() -> {
                        robot.deposit.paintbrush.dropPixel();
                    })
                    .waitSeconds(TIME_SPIKE_AUDIENCE)
                    .addTemporalMarker(() -> {
                        robot.deposit.lift.setTargetRow(-1);
                    })
                    .setTangent(a * FORWARD)
                    .turn(a * PI / 4.0)
            ;

        }

        sequence
                .addTemporalMarker(() -> {
                    robot.deposit.paintbrush.toggleFloor();
                    robot.intake.setMotorPower(SPEED_INTAKE_STACK_APPROACH);
                    robot.intake.setHeight(FIVE_STACK);
                    robot.intake.setIntakingAmount(1);
                })

                .lineToSplineHeading(stack)
                .setTangent(LEFT)

                .addTemporalMarker(() -> {
                    robot.intake.setMotorPower(SPEED_INTAKING);
                })
                .waitSeconds(TIME_INTAKING)
                .addTemporalMarker( () -> {
                    robot.intake.setMotorPower(0);
                })
        ;

        if (PARTNER_WAIT > 0) sequence.waitSeconds(PARTNER_WAIT);

        score(sequence, placements, 0);
    }
}
