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
 * Keeps the "one constants file" rule honest: the constants must be valid, the list of placeholders must match
 * the ADD markers in the source, and the logic classes must not hide tunable numbers of their own.
 */
public class ConstantsDisciplineTest {
    private static final String PKG = "org/firstinspires/ftc/teamcode/";

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

    private static String read(File f) throws IOException {
        return new String(Files.readAllBytes(f.toPath()), StandardCharsets.UTF_8);
    }

    // ------------------------------------------------------------------------------------------

    @Test
    public void theShippedConstantsAreValid() {
        ConstantsCheck.Report r = ConstantsCheck.run();
        assertTrue("constants errors: " + r.errors, r.ok());
        assertTrue("constants warnings: " + r.warnings, r.warnings.isEmpty());
    }

    @Test
    public void theTrackedPlaceholdersMatchTheAddMarkersInTheSource() throws IOException {
        String src = read(source("BioBuzzConstants.java"));
        Set<String> marked = new TreeSet<String>();
        Pattern doc = Pattern.compile("/\\*\\*(.*?)\\*/\\s*public static final [\\w\\[\\]<>.]+ (\\w+)", Pattern.DOTALL);
        Matcher m = doc.matcher(src);
        while (m.find()) {
            String body = m.group(1).replaceAll("(?m)^\\s*\\*", "").trim();
            if (body.startsWith("ADD")) {
                marked.add(enclosingGroup(src, m.start()) + "." + m.group(2));
            }
        }
        Set<String> listed = new TreeSet<String>(Arrays.asList(BioBuzzConstants.PLACEHOLDERS));
        assertEquals("PLACEHOLDERS must list exactly the constants whose comment starts with ADD", marked, listed);
    }

    private static String enclosingGroup(String src, int index) {
        Matcher c = Pattern.compile("public static final class (\\w+)").matcher(src.substring(0, index));
        String last = "";
        while (c.find()) {
            last = c.group(1);
        }
        return last;
    }

    @Test
    public void everyTrackedPlaceholderNamesARealConstant() throws Exception {
        for (String name : BioBuzzConstants.PLACEHOLDERS) {
            String[] parts = name.split("\\.");
            assertEquals(name, 2, parts.length);
            Class<?> group = Class.forName("org.firstinspires.ftc.teamcode.BioBuzzConstants$" + parts[0]);
            Field f = group.getField(parts[1]);
            assertTrue(name, f != null);
        }
    }

    // ------------------------------------------------------------------------------------------

    /** Files that make decisions or command hardware: no tunable number may be written into them. */
    private static List<File> logicFiles() {
        List<File> out = new ArrayList<File>();
        for (String dir : new String[]{"autoshoot/core", "autoshoot/hardware", "autoshoot/opmodes"}) {
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
        out.add(source("robots/biobuzz/BioBuzzRobot.java"));
        return out;
    }

    /** Numbers that are maths, indexing or unit conversion, not tuning. */
    private static final Set<String> ALLOWED = new HashSet<String>(Arrays.asList(
            "0", "1", "2", "3", "4", "0.0", "1.0", "2.0", "0.5", "1e-9", "1e-6", "1.0e9",
            "1000", "1000.0", "100.0"));   // ... plus unit conversions (ms to s, fraction to percent)

    @Test
    public void noTunableNumbersAreHiddenInTheLogicClasses() throws IOException {
        Pattern number = Pattern.compile("(?<![\\w.])(\\d+\\.\\d+(?:[eE][+-]?\\d+)?|\\d+(?:[eE][+-]?\\d+)?)[fFdDlL]?(?![\\w.])");
        List<String> violations = new ArrayList<String>();
        for (File f : logicFiles()) {
            String text = read(f);
            text = text.replaceAll("(?s)/\\*.*?\\*/", "");            // block comments
            String[] lines = text.split("\\r?\\n");
            for (int i = 0; i < lines.length; i++) {
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

    private static String join(List<String> lines) {
        StringBuilder sb = new StringBuilder();
        for (String l : lines) {
            sb.append(l).append("\n  ");
        }
        return sb.toString();
    }
}
