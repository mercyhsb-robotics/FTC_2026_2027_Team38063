package org.firstinspires.ftc.teamcode.autoshoot.render;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.io.ByteArrayInputStream;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilderFactory;

import org.junit.Test;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AllianceField;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.routes.FlowerPress;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteCatalog;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteValidator;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Step;
import org.firstinspires.ftc.teamcode.autoshoot.routes.StepKind;

/**
 * The robot in the route diagrams: drawn in two tones (the body and the two flat intake wheels) at the start pose and at
 * every INTAKE, SHOOT and PARK pose, with the press-in of a FLOWER shown as a short arrow, and a robot that breaks a rule
 * (an ERROR at its step) outlined in amber.
 */
public class RouteSvgFootprintTest {
    private static final double S = BioBuzzConstants.Auto.RENDER_FIELD_SCALE_PX_PER_IN;
    private static final double CANVAS = BioBuzzConstants.Auto.RENDER_CANVAS_PX;
    private static final double FIELD_PX = BioBuzzConstants.Field.SIZE_IN * S;
    private static final double X0 = (CANVAS - FIELD_PX) / 2.0;
    private static final double Y_BOTTOM = BioBuzzConstants.Auto.RENDER_FIELD_TOP_PX + FIELD_PX;
    private static final String AMBER = "#f9a825";

    // ---- DOM helpers (same style as RouteSvgRendererTest) --------------------------------------------

    private static Document parse(String svg) throws Exception {
        DocumentBuilderFactory f = DocumentBuilderFactory.newInstance();
        f.setNamespaceAware(true);
        return f.newDocumentBuilder().parse(new ByteArrayInputStream(svg.getBytes(Charset.forName("UTF-8"))));
    }

    private static List<Element> all(Node root, String tag) {
        List<Element> out = new ArrayList<Element>();
        NodeList nl = root instanceof Document ? ((Document) root).getElementsByTagName(tag) : ((Element) root).getElementsByTagName(tag);
        for (int i = 0; i < nl.getLength(); i++) {
            out.add((Element) nl.item(i));
        }
        return out;
    }

    private static boolean hasClass(Element e, String cls) {
        for (String c : e.getAttribute("class").split("\\s+")) {
            if (c.equals(cls)) {
                return true;
            }
        }
        return false;
    }

    private static List<Element> withClass(Node root, String tag, String cls) {
        List<Element> out = new ArrayList<Element>();
        for (Element e : all(root, tag)) {
            if (hasClass(e, cls)) {
                out.add(e);
            }
        }
        return out;
    }

    private static Element byId(Document d, String id) {
        for (Element e : all(d, "*")) {
            if (id.equals(e.getAttribute("id"))) {
                return e;
            }
        }
        return null;
    }

    private static double[] polygonBounds(Element polygon) {
        double minX = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        for (String p : polygon.getAttribute("points").trim().split("\\s+")) {
            String[] xy = p.split(",");
            minX = Math.min(minX, Double.parseDouble(xy[0]));
            maxX = Math.max(maxX, Double.parseDouble(xy[0]));
            minY = Math.min(minY, Double.parseDouble(xy[1]));
            maxY = Math.max(maxY, Double.parseDouble(xy[1]));
        }
        return new double[] {minX, maxX, minY, maxY};
    }

    private static double num(Element e, String attr) {
        return Double.parseDouble(e.getAttribute(attr));
    }

    private static double px(double fieldX) {
        return X0 + fieldX * S;
    }

    private static double py(double fieldY) {
        return Y_BOTTOM - fieldY * S;
    }

    /** A route with a FLOWER press-in (the INTAKE is step 3) that the validator accepts. */
    private static AutoRoute flowerRoute(IntakeWheel wheel) {
        AllianceField red = new AllianceField(new FieldModel(), Alliance.RED);
        Pose2 lineUp = FlowerPress.idealLineUp(red, 0, wheel);
        return AutoRoute.builder("flower").title("FLOWER press-in").startTile("B1").start(35, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90).shootUp(4)
                .driveConstant(lineUp.x, lineUp.y, lineUp.headingDegrees()).label("to the FLOWER")
                .intakeFlower(2.4, 48, wheel, 4).build();
    }

    // ---- the two tones -----------------------------------------------------------------------------

    @Test
    public void everyFootprintHasABodyAndTwoWheelDiscs() throws Exception {
        for (AutoRoute r : RouteCatalog.all()) {
            for (Alliance a : Alliance.values()) {
                Document d = parse(RouteSvgRenderer.render(r, a));
                Element route = byId(d, "route");
                int footprints = 0;
                for (int i = 0; i < r.size(); i++) {
                    StepKind k = r.step(i).kind();
                    footprints += k == StepKind.START || k == StepKind.SHOOT || k == StepKind.INTAKE || k == StepKind.PARK ? 1 : 0;
                }
                if (r.step(r.size() - 1).kind() == StepKind.DRIVE) {
                    footprints++;       // the end outline
                }
                assertEquals(r.id(), footprints, withClass(route, "polygon", "chassis").size());
                assertEquals(r.id(), 2 * footprints, withClass(route, "circle", "wheel").size());
            }
        }
    }

    @Test
    public void theBodyAndTheWheelsAreToScaleAndInTwoDifferentTones() throws Exception {
        AutoRoute r = RouteCatalog.byId("red_c1");                       // starts at (58, 9.2) facing -90: the front is toward the bottom
        Document d = parse(RouteSvgRenderer.render(r, Alliance.RED));
        Element start = byId(d, "start-footprint");
        Element chassis = withClass(start, "polygon", "chassis").get(0);
        double[] b = polygonBounds(chassis);
        double length = BioBuzzConstants.Robot.LENGTH_IN - BioBuzzConstants.Robot.INTAKE_REACH_IN;
        assertEquals("the body is as wide as the robot", BioBuzzConstants.Robot.WIDTH_IN * S, b[1] - b[0], 0.05);
        assertEquals("and shorter than it by the wheel reach", length * S, b[3] - b[2], 0.05);
        // facing -y the front is the bottom of the picture: the body's lower edge is the REACH above the perimeter's lower edge
        double[] outline = polygonBounds(withClass(start, "polygon", "outline").get(0));
        assertEquals(BioBuzzConstants.Robot.INTAKE_REACH_IN * S, outline[3] - b[3], 0.05);
        assertEquals("the back edge is shared", outline[2], b[2], 0.05);

        List<Element> wheels = withClass(start, "circle", "wheel");
        assertEquals(2, wheels.size());
        Pose2 pose = r.startPose();
        for (Element wheel : wheels) {
            IntakeWheel which = IntakeWheel.valueOf(wheel.getAttribute("data-wheel"));
            Point2 c = Footprint.wheelCenter(pose, which);
            assertEquals(px(c.x), num(wheel, "cx"), 0.05);
            assertEquals(py(c.y), num(wheel, "cy"), 0.05);
            assertEquals("the wheel is drawn as a disc of its diameter", BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN / 2.0 * S, num(wheel, "r"), 0.05);
            assertFalse("no wheel presses at the start", hasClass(wheel, "pressing"));
        }
        // the two wheels are 12.2 in apart, side to side
        assertEquals(2.0 * BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN * S, Math.abs(num(wheels.get(0), "cx") - num(wheels.get(1), "cx")), 0.05);
        // two tones: the body is in the alliance ink, the wheels are not
        assertEquals(BioBuzzConstants.Auto.RENDER_INK_RED, chassis.getAttribute("fill"));
        assertFalse(BioBuzzConstants.Auto.RENDER_INK_RED.equals(wheels.get(0).getAttribute("fill")));
        assertEquals(wheels.get(0).getAttribute("fill"), wheels.get(1).getAttribute("fill"));
    }

    @Test
    public void theBlueDrawingIsTheSameRobotTurnedHalfAWay() throws Exception {
        AutoRoute r = RouteCatalog.byId("red_b1");
        Document d = parse(RouteSvgRenderer.render(r, Alliance.BLUE));
        Pose2 blue = Alliance.BLUE.fromRed(r.startPose(), BioBuzzConstants.Field.SIZE_IN);
        for (Element wheel : withClass(byId(d, "start-footprint"), "circle", "wheel")) {
            Point2 c = Footprint.wheelCenter(blue, IntakeWheel.valueOf(wheel.getAttribute("data-wheel")));
            assertEquals(px(c.x), num(wheel, "cx"), 0.05);
            assertEquals(py(c.y), num(wheel, "cy"), 0.05);
        }
    }

    @Test
    public void theHeadingArrowStillEndsAtTheFrontOfTheRobot() throws Exception {
        Document d = parse(RouteSvgRenderer.render(RouteCatalog.byId("red_b1"), Alliance.RED));     // faces -90: down the picture
        Element arrow = withClass(byId(d, "start-footprint"), "line", "heading-arrow").get(0);
        double[] outline = polygonBounds(withClass(byId(d, "start-footprint"), "polygon", "outline").get(0));
        double tip = num(arrow, "y2");
        assertTrue("the arrow ends inside the outline, short of the front edge", tip < outline[3] && tip > (outline[2] + outline[3]) / 2.0);
    }

    // ---- the press-in ----------------------------------------------------------------------------------

    @Test
    public void aFlowerIntakeIsDrawnPressedInWithTheWheelRingedAndAnArrowIntoThePollen() throws Exception {
        for (IntakeWheel wheel : IntakeWheel.values()) {
            AutoRoute r = flowerRoute(wheel);
            Document d = parse(RouteSvgRenderer.render(r, Alliance.RED));
            Element intake = withClass(byId(d, "route"), "g", "intake").get(0);
            AllianceField red = new AllianceField(new FieldModel(), Alliance.RED);
            FlowerPress press = FlowerPress.of((Step.Intake) r.step(3), red);
            // the robot is drawn at the press pose (not at the line-up pose)
            // (the outline is NOT centered on the pose: facing the left wall its FRONT edge is 9.77 in to the left of the pose and its
            // BACK edge 8.03 in to the right, and the center-of-rotation dot sits at the pose itself)
            double[] outline = polygonBounds(withClass(intake, "polygon", "outline").get(0));
            assertEquals(px(press.press.x - BioBuzzConstants.Robot.FRONT_EXTENT_IN), outline[0], 0.05);
            assertEquals(px(press.press.x + BioBuzzConstants.Robot.REAR_EXTENT_IN), outline[1], 0.05);
            assertEquals(py(press.press.y), (outline[2] + outline[3]) / 2.0, 0.05);
            Element dot = all(withClass(intake, "g", "pose-point").get(0), "circle").get(0);
            assertEquals(px(press.press.x), num(dot, "cx"), 0.05);
            assertEquals(py(press.press.y), num(dot, "cy"), 0.05);
            // exactly one wheel is ringed: the one the step names, and it sits on the FLOWER's center line
            List<Element> pressing = withClass(intake, "circle", "pressing");
            assertEquals(1, pressing.size());
            assertEquals(wheel.name(), pressing.get(0).getAttribute("data-wheel"));
            assertEquals("the pressing wheel is on the FLOWER's center line (the wall is at x = 0, so y = 48)", py(48.0), num(pressing.get(0), "cy"), 0.05);
            // a short arrow along the wheel's path: it starts at the line-up pose and ends inside the POLLEN
            Element group = withClass(intake, "g", "press-in").get(0);
            assertEquals(wheel.name(), group.getAttribute("data-wheel"));
            Element line = withClass(group, "line", "press-arrow").get(0);
            double tipX = px(Footprint.wheelCenter(press.press, wheel).x - BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN / 2.0);
            Element head = withClass(group, "polygon", "arrowhead").get(0);
            assertEquals("the arrowhead tip is the front tip of the wheel", tipX, Double.parseDouble(head.getAttribute("points").split("\\s+")[0].split(",")[0]), 0.05);
            double pollenEdge = px(BioBuzzConstants.Field.FLOWER_POLLEN_CENTER_FROM_WALL_IN + BioBuzzConstants.Auto.POLLEN_DIAMETER_IN / 2.0);
            assertTrue("the arrow points into the wall and ends inside the POLLEN (1.25 in past its edge)", tipX < pollenEdge - 1.0);
            assertTrue("it moves toward the wall (left in the picture)", num(line, "x2") < num(line, "x1"));
            assertEquals("a straight run in the direction the robot faces", num(line, "y1"), num(line, "y2"), 0.05);
            // it starts at the middle of the wheel at the line-up pose, so it is the approach plus half a wheel long
            double run = (num(line, "x1") - tipX) / S;
            assertEquals("the final approach plus the wheel's radius", BioBuzzConstants.Auto.FLOWER_APPROACH_LENGTH_IN
                    + BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN / 2.0, run, 0.1);
            // and a dotted retreat arrow pointing back out
            Element retreat = withClass(group, "line", "retreat-arrow").get(0);
            assertTrue("the retreat moves away from the wall", num(retreat, "x2") > num(retreat, "x1"));
            assertNotNull(retreat.getAttribute("stroke-dasharray"));
        }
    }

    @Test
    public void onlyAFlowerIntakeHasAPressInArrow() throws Exception {
        for (AutoRoute r : RouteCatalog.all()) {
            Document d = parse(RouteSvgRenderer.render(r, Alliance.RED));
            assertEquals(r.id(), 0, withClass(byId(d, "route"), "g", "press-in").size());
            assertEquals(r.id(), 0, withClass(byId(d, "route"), "circle", "pressing").size());
        }
        Document flower = parse(RouteSvgRenderer.render(flowerRoute(IntakeWheel.LEFT), Alliance.RED));
        assertEquals(1, withClass(byId(flower, "route"), "g", "press-in").size());
    }

    @Test
    public void theBluePressInIsTheHalfTurnOfTheRedOneWithTheSameWheel() throws Exception {
        AutoRoute red = flowerRoute(IntakeWheel.RIGHT);
        Document d = parse(RouteSvgRenderer.render(red, Alliance.BLUE));
        AutoRoute blue = red.forAlliance(Alliance.BLUE);
        FlowerPress press = FlowerPress.of((Step.Intake) blue.step(3), new AllianceField(new FieldModel(), Alliance.BLUE));
        Element pressing = withClass(byId(d, "route"), "circle", "pressing").get(0);
        assertEquals("RIGHT", pressing.getAttribute("data-wheel"));
        Point2 c = Footprint.wheelCenter(press.press, IntakeWheel.RIGHT);
        assertEquals(px(c.x), num(pressing, "cx"), 0.05);
        assertEquals(py(c.y), num(pressing, "cy"), 0.05);
        // on the blue half the FLOWER is on the right wall: the arrow runs to the right
        Element line = withClass(byId(d, "route"), "line", "press-arrow").get(0);
        assertTrue(num(line, "x2") > num(line, "x1"));
    }

    // ---- a robot that breaks a rule is drawn in the warning colour --------------------------------------------

    @Test
    public void aFootprintAtAStepWithAnErrorIsOutlinedInAmberAndOthersAreNot() throws Exception {
        // the second volley fires 4 POLLEN that the robot does not hold (an ERROR at step 3); the first volley is fine
        AutoRoute r = AutoRoute.builder("bad").startTile("B1").start(35, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90).shootUp(4).shootUp(4).build();
        List<Issue> issues = RouteValidator.check(r);
        boolean errorAtStep3 = false;
        for (Issue i : issues) {
            errorAtStep3 |= i.isError() && i.stepIndex == 2;
        }
        assertTrue("the test route must have an ERROR at step 3: " + issues, errorAtStep3);
        Document d = parse(RouteSvgRenderer.render(r, Alliance.RED));
        List<Element> shoots = withClass(byId(d, "route"), "g", "shoot-pose");
        assertEquals(2, shoots.size());
        Element first = withClass(shoots.get(0), "polygon", "outline").get(0);
        Element second = withClass(shoots.get(1), "polygon", "outline").get(0);
        assertFalse(hasClass(first, "error"));
        assertEquals(BioBuzzConstants.Auto.RENDER_INK_RED, first.getAttribute("stroke"));
        assertTrue(hasClass(second, "error"));
        assertEquals(AMBER, second.getAttribute("stroke"));
        assertEquals(AMBER, second.getAttribute("fill"));
    }

    @Test
    public void aWarningAloneDoesNotChangeTheOutline() throws Exception {
        // a FLOWER press-in with a retreat of only 2 in is a WARN at step 4 and nothing else is wrong with that step
        AllianceField red = new AllianceField(new FieldModel(), Alliance.RED);
        Pose2 lineUp = FlowerPress.idealLineUp(red, 0, IntakeWheel.LEFT);
        AutoRoute r = AutoRoute.builder("warn").startTile("B1").start(35, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90).shootUp(4)
                .driveConstant(lineUp.x, lineUp.y, lineUp.headingDegrees())
                .intakeFlower(2.4, 48, IntakeWheel.LEFT, 4).retreat(2.0).build();
        boolean warnAtIntake = false;
        for (Issue i : RouteValidator.check(r)) {
            if (i.stepIndex == 3) {
                assertFalse("only a WARN at the INTAKE: " + i, i.isError());
                warnAtIntake = true;
            }
        }
        assertTrue(warnAtIntake);
        Document d = parse(RouteSvgRenderer.render(r, Alliance.RED));
        Element intake = withClass(byId(d, "route"), "g", "intake").get(0);
        assertFalse(hasClass(withClass(intake, "polygon", "outline").get(0), "error"));
    }

    @Test
    public void everyAmberOutlineInTheCatalogMatchesAnErrorAtThatStep() throws Exception {
        for (AutoRoute r : RouteCatalog.all()) {
            for (Alliance a : Alliance.values()) {
                AutoRoute own = r.forAlliance(a);
                List<Issue> issues = RouteValidator.check(own);
                int expected = 0;
                for (int i = 0; i < own.size(); i++) {
                    StepKind k = own.step(i).kind();
                    boolean hasRobot = k == StepKind.START || k == StepKind.SHOOT || k == StepKind.INTAKE || k == StepKind.PARK
                            || (k == StepKind.DRIVE && i == own.size() - 1);
                    boolean error = false;
                    for (Issue is : issues) {
                        error |= is.isError() && is.stepIndex == i;
                    }
                    expected += hasRobot && error ? 1 : 0;
                }
                Document d = parse(RouteSvgRenderer.render(r, a));
                assertEquals(r.id() + " " + a, expected, withClass(byId(d, "route"), "polygon", "error").size());
            }
        }
    }

    @Test
    public void theTwoDemonstrationRoutesAreDrawnWithoutAnyAmberRobot() throws Exception {
        for (String id : new String[] {"wall_shoot_park", "garden_cycle"}) {
            for (Alliance a : Alliance.values()) {
                Document d = parse(RouteSvgRenderer.render(RouteCatalog.byId(id), a));
                assertEquals(id + " " + a, 0, withClass(byId(d, "route"), "polygon", "error").size());
            }
        }
    }
}
