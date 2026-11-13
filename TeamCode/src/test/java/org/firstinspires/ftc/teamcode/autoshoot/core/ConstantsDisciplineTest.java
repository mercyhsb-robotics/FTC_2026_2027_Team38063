package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Keeps the 'one constants file' rule honest: the constants must be valid, the list of placeholders must match the ADD
 * markers in the source, and the logic classes must not hide tunable numbers of their own.
 *
 * <p>Why it matters: every number that changes how the robot behaves lives in BioBuzzConstants.java, so the team can
 * retune the robot without touching the logic. Only a test can make sure that stays true as the code grows.
 *
 * <p>How it works: unlike most tests, two of these read the robot's SOURCE FILES as plain text and search them with
 * regular expressions ('regex': a mini-language for describing text patterns). The files are found by trying two
 * folders: Gradle starts unit tests inside the TeamCode folder, and the second path covers a start from the repository
 * root. One test runs the same ConstantsCheck the TeleOp runs at INIT, and one uses reflection on the compiled
 * constants.
 *
 * <p>Run it with {@code ./gradlew :TeamCode:testDebugUnitTest} (add {@code --tests "*ConstantsDisciplineTest"} to run
 * only this class).
 *
 * <p>Words used in this file:
 * <ul>
 *   <li><b>Marker:</b> every constant's comment starts with FIXED (from the manual or CAD), ADD (a placeholder
 *       waiting for a measurement), VERIFY (check once on the robot) or TUNE (adjust from testing).</li>
 *   <li><b>Placeholder:</b> a constant marked ADD. BioBuzzConstants.PLACEHOLDERS lists them and the TeleOp shows
 *       that list at INIT as 'still to measure'.</li>
 *   <li><b>Magic number:</b> a bare number written inside logic code, with no name, that secretly changes how the
 *       robot behaves.</li>
 *   <li><b>Reflection:</b> Java looking up a class or field by its name at run time.</li>
 * </ul>
 */
public class ConstantsDisciplineTest {
    /** The robot code's folder, below src/main/java. */
    private static final String PKG = "org/firstinspires/ftc/teamcode/";

    /**
     * Finds a main-source file whichever folder the tests were started from. Gradle normally starts in the TeamCode
     * folder (first path); the second path covers a start from the repository root. Throws if neither exists.
     */
    private static File source(String relative) {
        File a = new File("src/main/java/" + PKG + relative);
        if (a.exists()) {
            return a;
        }
        File b = new File("TeamCode/src/main/java/" + PKG + relative);
        if (b.exists()) {
            return b;
        }
        throw new AssertionError("Cannot find " + relative + " from " + new File(".").getAbsolutePath());
    }

    /** Reads a whole source file as UTF-8 text. */
    private static String read(File f) throws IOException {
        return new String(Files.readAllBytes(f.toPath()), StandardCharsets.UTF_8);
    }

    // ------------------------------------------------------------------------------------------

    /**
     * Situation: the real values in BioBuzzConstants are run through ConstantsCheck, the same checker the TeleOp runs
     *     at INIT.
     * Expected: no errors AND no warnings.
     * Why: this catches typos (a negative range, a volley above 4 shots, two functions on one button) before the robot
     *     is switched on. Warnings fail the test too, so a suspicious value has to be fixed or knowingly reconsidered.
     */
    @Test
    public void theShippedConstantsAreValid() {
        ConstantsCheck.Report r = ConstantsCheck.run();
        assertTrue("constants errors: " + r.errors, r.ok());
        assertTrue("constants warnings: " + r.warnings, r.warnings.isEmpty());
    }

    /**
     * Situation: BioBuzzConstants.java is read as text. Every constant whose comment starts with 'ADD' is collected and
     *     compared with the PLACEHOLDERS list.
     * Expected: the two sets are identical.
     * Why: the TeleOp shows PLACEHOLDERS at INIT. If a value is measured but its marker or the list is not updated, the
     *     driver would be told the wrong thing.
     */
    @Test
    public void theTrackedPlaceholdersMatchTheAddMarkersInTheSource() throws IOException {
        String src = read(source("BioBuzzConstants.java"));
        Set<String> marked = new TreeSet<String>();
        // Matches a /** ... */ comment followed by 'public static final TYPE NAME'. Group 1 is the comment text and
        // group 2 is the constant's name.
        Pattern doc = Pattern.compile("/\\*\\*(.*?)\\*/\\s*public static final [\\w\\[\\]<>.]+ (\\w+)", Pattern.DOTALL);
        Matcher m = doc.matcher(src);
        while (m.find()) {
            // Remove the leading ' * ' of each comment line, leaving just the words.
            String body = m.group(1).replaceAll("(?m)^\\s*\\*", "").trim();
            // A comment that starts with ADD marks a placeholder waiting for a real measurement.
            if (body.startsWith("ADD")) {
                // Spell it 'Group.NAME', the way PLACEHOLDERS does.
                marked.add(enclosingGroup(src, m.start()) + "." + m.group(2));
            }
        }
        Set<String> listed = new TreeSet<String>(Arrays.asList(BioBuzzConstants.PLACEHOLDERS));
        assertEquals("PLACEHOLDERS must list exactly the constants whose comment starts with ADD", marked, listed);
    }

    /**
     * The name of the nested class (Robot, Shooter, Vision, ...) that a position in the source falls in: the last
     * 'public static final class X' declared before that position.
     */
    private static String enclosingGroup(String src, int index) {
        Matcher c = Pattern.compile("public static final class (\\w+)").matcher(src.substring(0, index));
        String last = "";
        while (c.find()) {
            last = c.group(1);
        }
        return last;
    }

    /**
     * Situation: each 'Group.NAME' string in PLACEHOLDERS, for example 'Vision.CAMERA_HEIGHT_IN'.
     * Expected: BioBuzzConstants has a nested class Group with a public field NAME.
     * Why: a typo in the list would otherwise be silently ignored and the real placeholder never reported.
     */
    @Test
    public void everyTrackedPlaceholderNamesARealConstant() throws Exception {
        for (String name : BioBuzzConstants.PLACEHOLDERS) {
            String[] parts = name.split("\\.");
            assertEquals(name, 2, parts.length);
            // Reflection: look up the nested class and its field by name. getField throws if the name does not exist,
            // which fails the test.
            Class<?> group = Class.forName("org.firstinspires.ftc.teamcode.BioBuzzConstants$" + parts[0]);
            Field f = group.getField(parts[1]);
            // (getField never returns null, so this line is only a formality; the real check is that it did not throw.)
            assertTrue(name, f != null);
        }
    }

    // ------------------------------------------------------------------------------------------

    /**
     * Files that make decisions or command hardware: no tunable number may be written into them. These are every .java
     * file in autoshoot/core, autoshoot/hardware and diagnostics (except ConstantsCheck.java, whose job is to check
     * numbers), plus Launcher.java, BioBuzzRobot.java and the FLOWER wiggle files of autoshoot/routes.
     */
    private static List<File> logicFiles() {
        List<File> out = new ArrayList<File>();
        for (String dir : new String[]{"autoshoot/core", "autoshoot/hardware", "diagnostics"}) {
            File[] files = source(dir).listFiles();
            if (files != null) {
                for (File f : files) {
                    if (f.getName().endsWith(".java") && !f.getName().equals("ConstantsCheck.java")) {
                        out.add(f);
                    }
                }
            }
        }
        out.add(source("mechanisms/launcher/Launcher.java"));
        out.add(source("teleop/BioBuzzRobot.java"));
        // The FLOWER wiggle logic in the route framework: it decides where the robot stands and how far it turns, so it must not hide a number either.
        // (The rest of autoshoot/routes is the validator and its explanations; only these files are scanned.)
        for (String name : new String[]{"FlowerWiggle", "FlowerPress", "WheelChoice", "PressRules", "LegSweeps"}) {
            out.add(source("autoshoot/routes/" + name + ".java"));
        }
        return out;
    }

    /**
     * Numbers that are maths, indexing or unit conversion, not tuning: small counts and indexes (0 to 4), halving
     * (0.5), tiny rounding guards (1e-9, 1e-6), nanoseconds per second (1.0e9), milliseconds per second (1000) and
     * fraction-to-percent (100.0).
     */
    private static final Set<String> ALLOWED = new HashSet<String>(Arrays.asList(
            "0", "1", "2", "3", "4", "0.0", "1.0", "2.0", "0.5", "1e-9", "1e-6", "1.0e9",
            "1000", "1000.0", "100.0"));   // ... plus unit conversions (ms to s, fraction to percent)

    /**
     * Situation: the text of every logic file, with comments, string literals, annotations and named 'static final'
     *     constants removed.
     * Expected: no number is left except the ALLOWED ones (0, 1, 2, ... see above).
     * Why: this enforces the team's one rule: every number that changes robot behavior lives in BioBuzzConstants, so
     *     retuning never means editing logic. If it fails, the message lists each file, line and number: move the
     *     number into BioBuzzConstants (or give it a named private constant).
     */
    @Test
    public void noTunableNumbersAreHiddenInTheLogicClasses() throws IOException {
        // Finds decimal or whole numbers (optionally with an exponent like 1e-9 or a suffix like f or L) that are not
        // part of a longer word or name, such as x1.
        Pattern number = Pattern.compile("(?<![\\w.])(\\d+\\.\\d+(?:[eE][+-]?\\d+)?|\\d+(?:[eE][+-]?\\d+)?)[fFdDlL]?(?![\\w.])");
        List<String> violations = new ArrayList<String>();
        for (File f : logicFiles()) {
            String text = read(f);
            // Note: deleting a multi-line block comment also deletes its line breaks, so the line numbers in the
            // failure message count from the comment-stripped text and come out smaller than the real line numbers.
            // Search for the printed line of code instead.
            text = text.replaceAll("(?s)/\\*.*?\\*/", "");            // block comments
            String[] lines = text.split("\\r?\\n");
            for (int i = 0; i < lines.length; i++) {
                // Drop // comments and blank out string literals, so words inside them are not read as numbers.
                String line = lines[i].replaceAll("//.*$", "").replaceAll("\"([^\"\\\\]|\\\\.)*\"", "\"\"");
                if (line.contains("static final") || line.trim().startsWith("@")) {
                    continue;                                          // a NAMED constant is fine
                }
                Matcher m = number.matcher(line);
                while (m.find()) {
                    if (!ALLOWED.contains(m.group(1))) {
                        violations.add(f.getName() + ":" + (i + 1) + "  " + m.group(1) + "   " + lines[i].trim());
                    }
                }
            }
        }
        Collections.sort(violations);
        assertTrue("Tunable numbers belong in BioBuzzConstants (or a named private constant):\n  "
                + join(violations), violations.isEmpty());
    }

    /** Joins the violation lines into one readable block for the failure message. */
    private static String join(List<String> lines) {
        StringBuilder sb = new StringBuilder();
        for (String l : lines) {
            sb.append(l).append("\n  ");
        }
        return sb.toString();
    }
}
