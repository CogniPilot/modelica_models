"""Regression coverage for annotation ownership and portable site output."""

from pathlib import Path
import tempfile
import unittest

from tools.build_docs import build, parse, rewrite_links


class DocumentationTests(unittest.TestCase):
    def test_nested_ownership_comments_strings_and_short_class(self):
        source = r"""within Example;
package Outer
  // model Fake end Fake;
  /* Documentation(info="wrong") */
  type Mode = enumeration(On, Off);
  model Inner "a quoted \"description\""
    parameter Real gain = 2 "gain";
    annotation(Documentation(info="<html><p>Inner \"quoted\" α</p></html>"));
  end Inner;
  annotation(Documentation(info="<html>Outer " + "only</html>", revisions="other"));
end Outer;"""
        classes = parse(source, Path("Example/Outer.mo"))
        self.assertEqual(
            [c.name for c in classes],
            ["Example.Outer", "Example.Outer.Mode", "Example.Outer.Inner"],
        )
        self.assertEqual(classes[0].info, "<html>Outer only</html>")
        self.assertEqual(classes[2].info, '<html><p>Inner "quoted" α</p></html>')
        self.assertEqual(classes[1].info, "")
        self.assertTrue(
            source[classes[2].start : classes[2].stop].endswith("end Inner;")
        )

    def test_redeclaration_binding_and_class_extends(self):
        source = chr(10).join(
            [
                "within; package A",
                "  model B extends Base(redeclare package Rotation = Other); end B;",
                "  redeclare record extends Orientation Real q[4]; end Orientation;",
                '  annotation(Documentation(info="<html>A help</html>"));',
                "end A;",
            ]
        )
        classes = parse(source, Path("A/package.mo"))
        self.assertEqual([c.name for c in classes], ["A", "A.B", "A.Orientation"])
        self.assertEqual(classes[0].info, "<html>A help</html>")
        self.assertIn("Real q[4]", source[classes[2].start : classes[2].stop])

    def test_links(self):
        info = '<html><a href="modelica://A.B#part">local</a><a href="modelica://Modelica.SIunits">external</a></html>'
        result = rewrite_links(info, {"A.B"})
        self.assertIn('href="A.B.html#part"', result)
        self.assertIn("modelica://Modelica.SIunits", result)
        self.assertNotIn("<html>", result)

    def test_site_excludes_nonlibrary_fixtures(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "Public").mkdir()
            (root / "Public/package.mo").write_text(
                'within; package Public "Public package summary" annotation(Documentation(info="<html><h2>Help</h2></html>")); end Public;'
            )
            (root / "Public/Foo.mo").write_text(
                'within Public; model Foo "Useful model description" parameter Real gain=2; end Foo;'
            )
            for folder in ("dev", "tools", "artifacts", "tools/upstream"):
                fixture = root / folder
                fixture.mkdir(parents=True, exist_ok=True)
                if "/" not in folder:
                    (fixture / "package.mo").write_text(
                        f"within; package {folder} end {folder};"
                    )
                (fixture / "fixture.mo").write_text("model Hidden end Hidden;")
            output = root / "site"
            self.assertEqual(build(root, output), 2)
            page = (output / "Public.Foo.html").read_text()
            self.assertIn("parameter Real gain=2;", page)
            self.assertIn("Useful model description", page)
            landing = (output / "index.html").read_text()
            self.assertIn("Public package summary", landing)
            self.assertIn('<details id="index">', landing)
            self.assertNotIn('<details id="index" open', landing)
            self.assertIn("Public/Foo.mo#L1", page)
            self.assertIn('href="Public.html"', page)
            self.assertIn("<h2>Help</h2>", (output / "Public.html").read_text())
            self.assertFalse((output / "Hidden.html").exists())


if __name__ == "__main__":
    unittest.main()
