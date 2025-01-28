package org.firstinspires.ftc.teamcode.VarsAndBoards;

import com.pedropathing.localization.Pose;

public class MapPos {

    //All in inches
    private final Pose StartingPose = new Pose(8, 72, Math.toRadians(0));
    private final Pose Basket = new Pose(24,120, Math.toRadians(270));
    private final Pose OtherObservation = new Pose(120, 120, Math.toRadians(90));
    private final Pose OtherBasket = new Pose(120, 24, Math.toRadians(135));
    private final Pose Observation = new Pose(8,40, Math.toRadians(0));
    private final Pose HangSpecimen = new Pose(35,72+12, Math.toRadians(0));
    private final Pose OtherHangSpecimen = new Pose(112,72,Math.toRadians(90));
    private final Pose TapeHangRobot = new Pose(72,96, Math.toRadians(90));
    private final Pose OtherTapeHangRobot = new Pose(72,48, Math.toRadians(270));
    private final Pose SpecCollect1 = new Pose(48-18, 48-8, Math.toRadians(180));
    private final Pose controlHangSpec = new Pose(21, 88);
    //TODO Add remaining important Poses
}