package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.CellSpec;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.core.TileGrid;

/**
 * The shortcuts that the freedom of rule G304 offers, written as routes with the START left open ({@link RouteTemplate}), so that
 * {@link StartPoseSearch#bestStart} can say how many seconds a smarter start saves, and so that {@code main} can print the table of them.
 *
 * <p>Run {@code tools/start-shortcuts.sh} (or {@code .bat}) to print the table. It takes about a minute: every route is tried from about 37 thousand starts.
 *
 * <p>This is route DATA, like {@link RouteCatalog}: the poses of the legs are the numbers a route author types, so this file is not scanned for hidden constants.
 * The families it covers, each with the standard starts (tiles B1 and C1, front on the audience wall) to compare with:
 * <ol>
 *   <li><b>Shoot, then leave and park.</b> The first volley goes into the cell that is UP at the start. Shortcut: START IN A SHOOTING POSE (front on the audience
 *       wall, in line with the HIVE, the ball leaves 36 in from the aim point) so the 4 pre-loads are fired without driving first.</li>
 *   <li><b>Shoot, take 3 GARDEN POLLEN, step off the wall.</b> Shortcut: a start that is a shooting pose AND near the GARDEN corner.</li>
 *   <li><b>Shoot, take POLLEN from the red-wall FLOWER, park.</b> Shortcut: start nearer the first pick-up.</li>
 *   <li><b>Partner tips first: wait for the camera, shoot 4 at the rear cell, leave and park.</b> The rear-wall equivalent of 1: front on the rear wall, in line with
 *       the HIVE, clear of the rear FLOWER.</li>
 *   <li><b>Partner tips first, then take 3 from the rear FLOWER and leave</b> (the partner's tip is assumed at 12 s, so there is no time for a PARK as well, nor for
 *       4 POLLEN). Shortcut: a start that is a shooting pose beside that FLOWER.</li>
 *   <li><b>Leave and park only.</b> Shortcut: a start close to the LOADING ZONE (not in it: G304.E).</li>
 * </ol>
 * The FLOWER families are timed for a robot whose flat wheels reach just far enough for the wiggle to work ({@link #wheelsThatReach()}): with the 2.7 in of the
 * goBILDA drawing they do not (see {@code FlowerWiggleTest}), and a route that cannot pick up has no best start.
 */
public final class StartShortcuts {
    private StartShortcuts() {
    }

    /** A start worth naming: "about here on this wall, facing this way". The search says where exactly that may be. */
    public static final class Named {
        public final String name;
        public final Wall wall;
        public final double headingDeg;
        public final double alongIn;

        Named(String name, Wall wall, double headingDeg, double alongIn) {
            this.name = name;
            this.wall = wall;
            this.headingDeg = headingDeg;
            this.alongIn = alongIn;
        }
    }

    /** One family of routes: the template, the standard starts to compare with and the starts worth naming. */
    public static final class Shortcut {
        public final String id;
        public final String title;
        public final String idea;
        public final RouteTemplate template;
        public final List<String> standardNames;
        public final List<Pose2> standardStarts;
        public final TimingModel timing;
        public final List<Named> named;

        Shortcut(String id, String title, String idea, RouteTemplate template, List<String> standardNames, List<Pose2> standardStarts, TimingModel timing,
                 List<Named> named) {
            this.id = id;
            this.title = title;
            this.idea = idea;
            this.template = template;
            this.standardNames = Collections.unmodifiableList(standardNames);
            this.standardStarts = Collections.unmodifiableList(standardStarts);
            this.timing = timing;
            this.named = Collections.unmodifiableList(named);
        }
    }

    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double SIZE = BioBuzzConstants.Field.SIZE_IN;
    private static final double HIVE_AXIS_X = BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN;
    /** The wiggle's wheel reach is made this much longer than the least that works, so a pick-up has a little room. */
    private static final double REACH_SPARE_IN = 0.1;

    // ---- the robot the FLOWER families are timed for ---------------------------------------------------------------------------

    /**
     * The timing of a robot whose flat wheels stick out just far enough for the FLOWER wiggle to work (the least reach for which both wheels can reach the POLLEN,
     * plus 0.1 in). The shipped {@code Robot.INTAKE_REACH_IN} is too short, so a route with a FLOWER pick-up has an ERROR for every start and the search would
     * have nothing to say; this robot answers "where would we start IF the wheels reach?". Everything else is the shipped robot.
     */
    public static TimingModel wheelsThatReach() {
        FlowerWiggle.Spec spec = FlowerWiggle.Spec.fromConstants();
        double missing = Math.max(FlowerWiggle.solve(IntakeWheel.LEFT, spec).missingReachIn, FlowerWiggle.solve(IntakeWheel.RIGHT, spec).missingReachIn);
        return new TimingModel().withFlowerSpec(spec.withIntakeReach(spec.intakeReachIn + Math.max(0.0, missing) + REACH_SPARE_IN));
    }

    // ---- the pieces of a route ---------------------------------------------------------------------------------------------------

    /**
     * True if a robot standing at {@code p} may fire at the cell on {@code side} of the red HIVE from there: the standoff band and the aim of the validator, each
     * widened by the placement margin (an inch of standoff, 3 degrees of aim). The margin is there because the robot is never exactly where it was put: at the
     * runtime the solver stands the robot a little closer or farther if it must, so a start that is within the margin of a shooting pose is one.
     */
    static boolean isShootingPose(Pose2 p, Side side) {
        CellSpec cell = new AllianceField(new FieldModel(), Alliance.RED).cell(side);
        double standoff = ShotGeometry.standoff(p, cell);
        return ShotGeometry.onOpeningSide(p, cell)
                && standoff >= BioBuzzConstants.Auto.SHOOT_STANDOFF_MIN_IN - BioBuzzConstants.Auto.START_PLACEMENT_MARGIN_IN
                && standoff <= BioBuzzConstants.Auto.SHOOT_STANDOFF_MAX_IN + BioBuzzConstants.Auto.START_PLACEMENT_MARGIN_IN
                && Math.toDegrees(ShotGeometry.aimError(p, cell)) <= BioBuzzConstants.Auto.SHOOT_AIM_TOLERANCE_DEG + BioBuzzConstants.Auto.START_PLACEMENT_MARGIN_DEG;
    }

    private static AutoRoute.Builder opening(String id, String title, Pose2 s, boolean partner) {
        AutoRoute.Builder b = AutoRoute.builder(id).title(title).startTile(TileGrid.nameAt(s.x, s.y));
        if (partner) {
            b.partnerTipsFirst();
        }
        return b.start(s.x, s.y, s.headingDegrees());
    }

    /**
     * The volley of 4 at the cell on {@code side}: fired from the start itself if the start is a shooting pose (the robot already stands where it would shoot, so it
     * does not move first), else from the pose the solver picks, which means a drive to the nominal standoff pose first.
     */
    private static AutoRoute.Builder shoot4(AutoRoute.Builder b, Pose2 s, Side side) {
        return isShootingPose(s, side) ? b.shootUp(s.x, s.y, s.headingDegrees(), 4) : b.shootUp(4);
    }

    // ---- the six families --------------------------------------------------------------------------------------------------------

    private static RouteTemplate shootThenPark() {
        return new RouteTemplate() {
            @Override
            public AutoRoute build(Pose2 s) {
                return shoot4(opening("shoot_park", "Shoot 4, left corridor, park", s, false), s, Side.AUDIENCE)
                        .driveConstant(30.0, 14.8, -90.0).label("along the wall")
                        .driveReverseTangent(30.0, 100.0).label("corridor")
                        .driveLinear(12.0, 108.0, 0.0).label("to zone")
                        .park(12.0, 108.0, 0.0).build();
            }
        };
    }

    private static RouteTemplate shootThenGarden() {
        return new RouteTemplate() {
            @Override
            public AutoRoute build(Pose2 s) {
                return shoot4(opening("shoot_garden", "Shoot 4, take 3 GARDEN POLLEN, step off the wall", s, false), s, Side.AUDIENCE)
                        .driveConstant(9.0, 18.0, -90).label("to GARDEN")
                        .driveConstant(9.0, FRONT + 0.8, -90).label("into GARDEN").wallContact()
                        .intakeGarden(5.7, 1.4, 9.0, FRONT + 0.8, -90, 3)
                        .driveConstant(20.0, 24.0, -90).label("off the wall").build();
            }
        };
    }

    private static RouteTemplate shootThenFlowerThenPark() {
        final double[] flower = BioBuzzConstants.Field.FLOWER_CENTERS_IN[0];            // the red-wall FLOWER
        return new RouteTemplate() {
            @Override
            public AutoRoute build(Pose2 s) {
                return shoot4(opening("shoot_flower", "Shoot 4, take 3 from the red-wall FLOWER, park", s, false), s, Side.AUDIENCE)
                        .driveConstant(30.0, 14.8, -90.0).label("along the wall")
                        .intakeFlower(flower[0], flower[1], 3)
                        .driveConstant(15.0, 80.0, 180.0).label("up the wall")
                        .driveLinear(15.0, 108.0, 0.0).label("to zone")
                        .park(15.0, 108.0, 0.0).build();
            }
        };
    }

    private static RouteTemplate partnerShootThenPark() {
        return new RouteTemplate() {
            @Override
            public AutoRoute build(Pose2 s) {
                return shoot4(opening("partner_park", "Partner tips: wait, shoot 4 at the rear cell, park", s, true).waitForTip(16.0), s, Side.REAR)
                        .driveConstant(s.x, 112.0, s.headingDegrees()).label("off the wall")
                        .driveLinear(12.0, 108.0, 0.0).label("to zone")
                        .park(12.0, 108.0, 0.0).build();
            }
        };
    }

    private static RouteTemplate partnerShootThenFlower() {
        final double[] flower = BioBuzzConstants.Field.FLOWER_CENTERS_IN[1];            // the rear-wall FLOWER
        return new RouteTemplate() {
            @Override
            public AutoRoute build(Pose2 s) {
                return shoot4(opening("partner_flower", "Partner tips: wait, shoot 4, take 3 from the rear FLOWER, leave", s, true).waitForTip(16.0), s, Side.REAR)
                        .driveConstant(s.x, 121.0, s.headingDegrees()).label("off the wall")
                        .driveConstant(flower[0], 121.0, 90.0).label("in front of the FLOWER")
                        .intakeFlower(flower[0], flower[1], 3)
                        .driveConstant(35.0, 112.0, 90.0).label("away from the wall").build();
            }
        };
    }

    private static RouteTemplate leaveAndPark() {
        return new RouteTemplate() {
            @Override
            public AutoRoute build(Pose2 s) {
                return opening("leave_park", "Leave and park only", s, false)
                        .driveLinear(15.0, 108.0, 0.0).label("to zone")
                        .park(15.0, 108.0, 0.0).build();
            }
        };
    }

    // ---- the table ---------------------------------------------------------------------------------------------------------------

    /** Tiles B1 and C1 of the audience wall, front first: the starts of the team's old OpModes, to compare a better start with. */
    private static List<Pose2> audienceStandard() {
        return Arrays.asList(new Pose2(TileGrid.centerX(1), FRONT, -Math.PI / 2.0), new Pose2(TileGrid.centerX(2), FRONT, -Math.PI / 2.0));
    }

    /** Every family, in the order of the class comment. */
    public static List<Shortcut> all() {
        List<String> audienceNames = Arrays.asList("tile B1, front on the audience wall", "tile C1, front on the audience wall");
        List<Named> audienceNamed = Collections.singletonList(
                new Named("front on the audience wall, in line with the HIVE", Wall.AUDIENCE, -90.0, HIVE_AXIS_X));
        List<String> partnerNames = Collections.singletonList("the catalog's partner start, front on the rear wall");
        List<Pose2> partnerStandard = Collections.singletonList(Pose2.ofDegrees(61.5, SIZE - FRONT, 90.0));
        List<Named> partnerNamed = Collections.singletonList(
                new Named("front on the rear wall, in line with the rear cell, clear of the rear FLOWER", Wall.REAR, 90.0, HIVE_AXIS_X));
        TimingModel shipped = new TimingModel();
        double flowerY = BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][1];
        List<Named> leftNamed = Arrays.asList(audienceNamed.get(0),
                new Named("on the red wall beside the red-wall FLOWER, front toward the wall", Wall.RED, 180.0,
                        flowerY + BioBuzzConstants.Field.FLOWER_BRACKET_ALONG_WALL_IN / 2.0 + BioBuzzConstants.Robot.LEFT_EXTENT_IN));
        List<Named> parkNamed = Collections.singletonList(
                new Named("on the red wall just below the LOADING ZONE, facing it", Wall.RED, 0.0, BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MIN_IN
                        - BioBuzzConstants.Robot.LEFT_EXTENT_IN));
        List<Shortcut> out = new ArrayList<Shortcut>();
        out.add(new Shortcut("shoot_park", "Shoot 4 at the UP cell, left corridor, park",
                "start in a shooting pose: no drive before the first volley", shootThenPark(), audienceNames, audienceStandard(), shipped, audienceNamed));
        out.add(new Shortcut("shoot_garden", "Shoot 4, take 3 GARDEN POLLEN, step off the wall",
                "a shooting pose that is also near the GARDEN corner", shootThenGarden(), audienceNames, audienceStandard(), shipped, audienceNamed));
        out.add(new Shortcut("shoot_flower", "Shoot 4, take 3 from the red-wall FLOWER, park (wheels that reach)",
                "start nearer the first pick-up", shootThenFlowerThenPark(), audienceNames, audienceStandard(), wheelsThatReach(), leftNamed));
        out.add(new Shortcut("partner_park", "Partner tips first: wait, shoot 4 at the rear cell, park",
                "the rear-wall equivalent of a shooting-pose start, clear of the rear FLOWER", partnerShootThenPark(), partnerNames, partnerStandard, shipped,
                partnerNamed));
        out.add(new Shortcut("partner_flower", "Partner tips first: wait, shoot 4, take 3 from the rear FLOWER, leave (wheels that reach)",
                "a shooting pose beside the rear FLOWER", partnerShootThenFlower(), partnerNames, partnerStandard, wheelsThatReach(), partnerNamed));
        out.add(new Shortcut("leave_park", "Leave and park only",
                "a start close to the LOADING ZONE", leaveAndPark(), audienceNames, audienceStandard(), shipped, parkNamed));
        return Collections.unmodifiableList(out);
    }

    /** What one start gives for a template: the time, whether the validator accepts it, and whether it keeps the placement margin. */
    static String describeStart(RouteTemplate template, Pose2 start, StartPoseSearch.Settings settings) {
        StartRobustness.Report r = StartRobustness.check(template, start, Alliance.RED, settings.timing, settings.marginIn, settings.marginDeg);
        String own = r.nominalErrors.isEmpty() ? "valid" : "INVALID (" + r.nominalErrors.get(0).message + ")";
        String margin = "";
        if (r.nominalErrors.isEmpty()) {
            margin = r.placementSafe() ? ", keeps the margin" : ", FAILS the margin (" + (r.newErrors().isEmpty() ? "an ERROR that the start has too" : r.newErrors().get(0).rule.toString()) + ")";
        }
        return String.format(Locale.ROOT, "%s  %.1f s  %s%s", pose(start), r.nominalSeconds, own, margin);
    }

    static String pose(Pose2 p) {
        return String.format(Locale.ROOT, "(x %.2f, y %.2f, heading %.0f deg)", p.x, p.y, p.headingDegrees());
    }

    /**
     * The table of shortcuts as text: for every family the standard starts, the best start with the placement margin, the best start with no margin at all (what the
     * margin costs), and the named starts, each with its time and what it saves.
     */
    public static String table(StartPoseSearch.Settings base) {
        return table(base, null);
    }

    /** As {@link #table(StartPoseSearch.Settings)} for the one family with this id only ({@code null}: all of them). */
    public static String table(StartPoseSearch.Settings base, String onlyId) {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format(Locale.ROOT, "WHERE TO SET THE ROBOT UP (rule G304), red frame. Grid: a start every %.2f in along the walls, every %.0f degrees. Placement margin: %.1f in and %.0f degrees.%n",
                base.stepIn, base.headingStepDeg, base.marginIn, base.marginDeg));
        sb.append("Times are the plan's estimates (placeholder drive and shooter numbers), never a promise.\n");
        for (Shortcut s : all()) {
            if (onlyId != null && !onlyId.equals(s.id)) {
                continue;
            }
            StartPoseSearch.Settings st = base.withTiming(s.timing);
            sb.append('\n').append(s.title).append("\n  idea: ").append(s.idea).append('\n');
            StartPoseSearch.Best best = StartPoseSearch.bestStart(s.template, s.standardStarts, st);
            for (int i = 0; i < s.standardStarts.size(); i++) {
                sb.append("  standard, ").append(s.standardNames.get(i)).append(": ").append(describeStart(s.template, s.standardStarts.get(i), st)).append('\n');
            }
            appendBest(sb, "best start", s.template, best, st);
            StartPoseSearch.Settings none = st.withMargin(0.0, 0.0);
            appendBest(sb, "best start with NO margin", s.template, StartPoseSearch.bestStart(s.template, s.standardStarts, none), none);
            for (Named n : s.named) {
                appendNamed(sb, s, n, st, best);
            }
        }
        return sb.toString();
    }

    /**
     * A start worth naming: the place nearest to the wanted spot that keeps the placement margin and gives a valid route, with its time. If there is none, it says
     * so, and how much of the margin does work (the margin is scaled down in tenths until a start passes).
     */
    private static void appendNamed(StringBuilder sb, Shortcut s, Named n, StartPoseSearch.Settings st, StartPoseSearch.Best best) {
        Pose2 at = StartPoseSearch.nearestSafeStart(Alliance.RED, n.wall, n.headingDeg, n.alongIn, st.marginIn, st.marginDeg);
        if (isThere(n, at) && StartRobustness.passes(s.template, at, Alliance.RED, st.timing, st.marginIn, st.marginDeg)) {
            sb.append("  named, ").append(n.name).append(": ").append(describeStart(s.template, at, st));
            if (!Double.isNaN(best.standardSeconds)) {
                double sec = RouteAnalysis.of(s.template.build(at), new FieldModel(), st.timing).totalSeconds();
                double difference = sec - best.standardSeconds;
                sb.append(String.format(Locale.ROOT, ", %+.1f s against the standard", Math.abs(difference) < 0.05 ? 0.0 : difference));       // "+0.0", never "-0.0"
            }
            sb.append("\n");
            return;
        }
        sb.append("  named, ").append(n.name).append(": NO start there keeps the margin of ").append(st.marginIn).append(" in and ").append(st.marginDeg).append(" degrees");
        for (double scale = 0.9; scale > -0.05; scale -= 0.1) {
            double in = st.marginIn * scale;
            double deg = st.marginDeg * scale;
            Pose2 loose = StartPoseSearch.nearestSafeStart(Alliance.RED, n.wall, n.headingDeg, n.alongIn, in, deg);
            if (isThere(n, loose) && StartRobustness.passes(s.template, loose, Alliance.RED, st.timing, in, deg)) {
                sb.append(String.format(Locale.ROOT, "; it works with %.0f%% of it (%.1f in, %.1f degrees): ", scale * 100.0, in, deg));
                sb.append(describeStart(s.template, loose, st.withMargin(in, deg)));
                break;
            }
        }
        sb.append("\n");
    }

    /** How far along the wall a start may be from the spot a named start asks for and still count as "there", inches: a start 24 in away is another start. */
    private static final double THERE_IN = 3.0;

    private static boolean isThere(Named n, Pose2 start) {
        return start != null && Math.abs(n.wall.along(start) - n.alongIn) <= THERE_IN;
    }

    private static void appendBest(StringBuilder sb, String label, RouteTemplate template, StartPoseSearch.Best best, StartPoseSearch.Settings settings) {
        if (!best.found) {
            sb.append("  ").append(label).append(": none. ").append(best.reason).append("\n");
            return;
        }
        sb.append("  ").append(label).append(": ").append(describeStart(template, best.start, settings));
        if (!Double.isNaN(best.standardSeconds)) {
            sb.append(String.format(Locale.ROOT, ", saves %.1f s against the standard start", best.secondsSaved()));
        }
        sb.append("\n      place it: ").append(StartPlacement.placementText(best.start, Alliance.RED)).append("\n");
    }

    /** Prints the table (of one family if its id is given). Run it with {@code tools/start-shortcuts.sh [id]}. */
    public static void main(String[] args) {
        System.out.println(table(StartPoseSearch.Settings.standard(), args.length > 0 ? args[0] : null));
    }
}
