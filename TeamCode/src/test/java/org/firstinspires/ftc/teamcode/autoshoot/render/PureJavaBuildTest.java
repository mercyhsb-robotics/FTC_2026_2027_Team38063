package org.firstinspires.ftc.teamcode.autoshoot.render;

import static org.junit.Assert.assertTrue;

import java.io.File;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import javax.tools.Diagnostic;
import javax.tools.DiagnosticCollector;
import javax.tools.JavaCompiler;
import javax.tools.JavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.ToolProvider;

import org.junit.Assume;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

/**
 * Keeps the route framework runnable on a PC: BioBuzzConstants and the autoshoot core, routes and render
 * packages must compile with plain javac, with nothing on the classpath (so no FTC SDK, Android or Pedro) and
 * against the Java 8 library (so no {@code List.of}, {@code Optional.isEmpty} and the like). That is exactly what
 * {@code tools/render-autos.sh} does, so a failure here means the script would fail too.
 */
public class PureJavaBuildTest {
    private static final String PKG = "org/firstinspires/ftc/teamcode/";

    @Rule
    public TemporaryFolder temp = new TemporaryFolder();

    private static File sourceRoot() {
        for (String candidate : new String[] {"src/main/java", "TeamCode/src/main/java"}) {
            if (new File(candidate, PKG + "BioBuzzConstants.java").isFile()) {
                return new File(candidate);
            }
        }
        throw new AssertionError("Cannot find the sources from " + new File(".").getAbsolutePath());
    }

    private static void collect(File dir, List<File> out) {
        File[] children = dir.listFiles();
        Arrays.sort(children);
        for (File f : children) {
            if (f.isDirectory()) {
                collect(f, out);
            } else if (f.getName().endsWith(".java")) {
                out.add(f);
            }
        }
    }

    @Test
    public void theConstantsAndThePureJavaPackagesCompileWithPlainJavacAtJava8() throws Exception {
        JavaCompiler javac = ToolProvider.getSystemJavaCompiler();
        Assume.assumeNotNull(javac);        // a JRE without the compiler: nothing to check

        File root = sourceRoot();
        List<File> sources = new ArrayList<File>();
        sources.add(new File(root, PKG + "BioBuzzConstants.java"));
        for (String pkg : new String[] {"core", "routes", "render"}) {
            collect(new File(root, PKG + "autoshoot/" + pkg), sources);
        }

        File out = temp.newFolder("classes");
        List<String> options = new ArrayList<String>(Arrays.asList(
                "-d", out.getPath(),
                "-classpath", out.getPath(),        // nothing else: no FTC SDK, no Android, no Pedro
                "-sourcepath", out.getPath(),
                "-encoding", "UTF-8",
                "-proc:none",
                "-nowarn"));
        if (!System.getProperty("java.specification.version").startsWith("1.")) {
            options.add("--release");
            options.add("8");                       // the Java 8 library, as on the robot
        }

        DiagnosticCollector<JavaFileObject> diagnostics = new DiagnosticCollector<JavaFileObject>();
        StandardJavaFileManager files = javac.getStandardFileManager(diagnostics, Locale.ROOT, Charset.forName("UTF-8"));
        try {
            boolean ok = javac.getTask(null, files, diagnostics, options, null, files.getJavaFileObjectsFromFiles(sources)).call();
            StringBuilder errors = new StringBuilder();
            for (Diagnostic<? extends JavaFileObject> d : diagnostics.getDiagnostics()) {
                if (d.getKind() == Diagnostic.Kind.ERROR) {
                    errors.append(d.getSource() == null ? "" : d.getSource().getName()).append(':').append(d.getLineNumber())
                            .append(": ").append(d.getMessage(Locale.ROOT)).append('\n');
                }
            }
            assertTrue("these sources must compile with plain javac (no FTC SDK imports, Java 8 only):\n" + errors, ok);
        } finally {
            files.close();
        }
        assertTrue("the entry point was built", new File(out, PKG + "autoshoot/render/RenderRoutesMain.class").isFile());
    }
}
