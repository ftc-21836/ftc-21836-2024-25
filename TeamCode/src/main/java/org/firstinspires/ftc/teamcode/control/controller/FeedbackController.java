package org.firstinspires.ftc.teamcode.control.controller;

import org.firstinspires.ftc.teamcode.control.motion.State;

public interface FeedbackController extends Controller {
    double calculate(State measurement);
}
