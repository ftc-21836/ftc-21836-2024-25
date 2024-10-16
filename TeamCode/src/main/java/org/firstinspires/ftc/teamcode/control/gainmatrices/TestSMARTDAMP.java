package org.firstinspires.ftc.teamcode.control.gainmatrices;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Lift.PERCENT_OVERSHOOT;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Lift.feedforwardGains;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Lift.pidGains;

public final class TestSMARTDAMP {

    public static void main(String[] args) {

        pidGains.computeKd(feedforwardGains, PERCENT_OVERSHOOT);

        System.out.println(pidGains.kP);
        System.out.println(pidGains.kD);
    }
}
