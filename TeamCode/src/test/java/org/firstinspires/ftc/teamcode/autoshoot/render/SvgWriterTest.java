package org.firstinspires.ftc.teamcode.autoshoot.render;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Locale;

import org.junit.Test;

public class SvgWriterTest {

    @Test
    public void numbersUseADotAndAtMostTwoDecimalsWithoutTrailingZeros() {
        assertEquals("12", SvgWriter.n(12.0));
        assertEquals("12.5", SvgWriter.n(12.5));
        assertEquals("0.07", SvgWriter.n(0.07));
        assertEquals("3.14", SvgWriter.n(3.14159));
        assertEquals("-2.4", SvgWriter.n(-2.4));
        assertEquals("0", SvgWriter.n(-0.001));
        assertEquals("100", SvgWriter.n(99.999));
    }

    @Test
    public void numbersIgnoreTheDefaultLocale() {
        Locale old = Locale.getDefault();
        try {
            Locale.setDefault(Locale.GERMANY);
            assertEquals("12.5", SvgWriter.n(12.5));
            assertEquals("2.25", SvgWriter.n(2.25));
        } finally {
            Locale.setDefault(old);
        }
    }

    @Test
    public void notANumberCannotEndUpInTheImage() {
        try {
            SvgWriter.n(Double.NaN);
            fail("NaN must be rejected");
        } catch (IllegalArgumentException expected) {
            // good
        }
        try {
            SvgWriter.n(Double.POSITIVE_INFINITY);
            fail("infinity must be rejected");
        } catch (IllegalArgumentException expected) {
            // good
        }
    }

    @Test
    public void escapingCoversTheXmlSpecialsAndNonAscii() {
        assertEquals("a &amp; b &lt;c&gt; &quot;d&quot; &apos;e&apos;", SvgWriter.esc("a & b <c> \"d\" 'e'"));
        assertEquals("caf&#233;", SvgWriter.esc("caf" + (char) 233));    // e acute, built from its code so the file stays ASCII
        assertEquals("tab\there", SvgWriter.esc("tab\there"));
        assertEquals("no ctrl", SvgWriter.esc("no\u0001ctrl"));
    }

    @Test
    public void attributesAreEscapedNullOnesAreSkippedAndPairsAreRequired() {
        SvgWriter w = new SvgWriter();
        w.empty("rect", "a", "x\"y", "skip", null, "b", "1");
        assertEquals("<rect a=\"x&quot;y\" b=\"1\"/>\n", w.toString());
        try {
            new SvgWriter().empty("rect", "odd");
            fail("an odd number of attribute strings");
        } catch (IllegalArgumentException expected) {
            // good
        }
    }

    @Test
    public void elementsNestAndEscapeTheirText() {
        SvgWriter w = new SvgWriter();
        w.open("g", "id", "x");
        w.element("text", "1 < 2", "y", "5");
        w.close("g");
        String s = w.toString();
        assertTrue(s, s.startsWith("<g id=\"x\">"));
        assertTrue(s, s.contains("<text y=\"5\">1 &lt; 2</text>"));
        assertTrue(s, s.trim().endsWith("</g>"));
        assertFalse(s.contains("1 < 2"));
    }
}
