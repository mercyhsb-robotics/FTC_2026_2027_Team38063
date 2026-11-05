package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.line;

//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "PedroPathingAuto_Forward", group = "Autonomous")
//@Configurable
public class Test_Forward extends OpMode {
//    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private int pathState;

    private final PoseFactory poses = PoseFactory.degrees();
    private final Pose pose1 = poses.of(0, 0, 90);
    private final Pose pose2 = poses.of(0, 48, 90);
    private Path outbound;
    private Path inbound;

    @Override
    public void init() {
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathState = 0;
        follower = Constants.create(hardwareMap);
        follower.setPose(pose1);
        outbound = line(pose1, pose2).linear(pose1, pose2);
        inbound = line(pose2, pose1).linear(pose2, pose1);
//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.follow(outbound);
                    pathState = 1;
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.follow(inbound);
                    pathState = 2;
                }
                break;
            default:
                break;
        }
//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("X", follower.pose().x());
//        panelsTelemetry.debug("Y", follower.pose().y());
//        panelsTelemetry.debug("Heading", follower.pose().heading());
//        panelsTelemetry.update(telemetry);
    }
}
