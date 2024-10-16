package org.firstinspires.ftc.teamcode.subsystems.utilities;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public final class BulkReader {

    private final List<LynxModule> revHubs;

    public BulkReader(HardwareMap hardwareMap) {
        revHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : revHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public void bulkRead() {
        for (LynxModule hub : revHubs) hub.clearBulkCache();
    }
}
