"""Regression coverage for annotation ownership and portable site output."""

from pathlib import Path
from html.parser import HTMLParser
import tempfile
import unittest

from tools.build_docs import (
    api,
    build,
    highlighted_source,
    parse,
    reference_links,
    rewrite_links,
    source_page_name,
)


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

    def test_public_api_ownership_arrays_defaults_and_inheritance(self):
        source = """within; package Public
  record Data
    Real field[2](each unit="m") = {1, 2} "position";
  protected
    Real hidden;
  end Data;
  model Example
    extends Public.Base(gain=2);
    parameter Real x[2, 2](each unit="m") =
      {{1, 2}, {3, 4}} "matrix";
    parameter String label = "value" "label description";
    input Public.Data data;
    output Real y, z[2] = {1,2} "second";
    model Nested parameter Real leak = 1; end Nested;
  protected
    parameter Real secret = 2;
  public
    parameter Real visible = 3;
  equation
    y = data.field[1];
    assert(y > 0, "not a declaration");
  end Example;
end Public;"""
        classes = parse(source, Path("Public/package.mo"))
        example = next(item for item in classes if item.name == "Public.Example")
        declarations, bases = api(example, source)
        self.assertEqual(bases, ["Public.Base"])
        self.assertEqual(
            [d.name for d in declarations], ["x", "label", "data", "y", "z", "visible"]
        )
        self.assertIn("{{1, 2}, {3, 4}}", declarations[0].details)
        self.assertIn('unit="m"', declarations[0].details)
        self.assertEqual(declarations[0].description, "matrix")
        self.assertEqual(declarations[1].details, '= "value"')
        self.assertEqual(declarations[1].description, "label description")
        fields, _ = api(next(c for c in classes if c.name == "Public.Data"), source)
        self.assertEqual([d.name for d in fields], ["field"])

    def test_definition_links_are_conservative(self):
        source = """within; package Public
  function f input Real x; output Real y; algorithm y := x; end f;
  package Other function duplicate end duplicate; end Other;
  package Another function duplicate end duplicate; end Another;
  model Example
    import Alias = Public.f;
    Real value;
  equation
    value = Public.f(1) + f(2) + Alias(3) + duplicate(4) + Unknown.call(5);
  end Example;
  model Shadow
  protected
    Real f;
  equation
    f = 1;
  end Shadow;
  model Dynamic
    replaceable package Dispatch = Public.Other;
  equation
    value = Dispatch.duplicate();
  end Dynamic;
end Public;"""
        path = Path("Public/package.mo")
        classes = parse(source, path)
        links = reference_links(source, path, classes, {path: source})
        linked_text = {source[start:stop] for start, (stop, _) in links.items()}
        self.assertTrue({"Public.f", "f", "Alias"}.issubset(linked_text))
        self.assertNotIn(source.index("duplicate(4)"), links)
        self.assertNotIn("Unknown.call", linked_text)
        self.assertNotIn(source.index("Dispatch.duplicate()"), links)
        self.assertNotIn(source.index("    f = 1") + 4, links)
        call = source.index("Public.f(1)")
        self.assertEqual(links[call][1], "source-Public.package.mo.html#L2")

    def test_highlighted_text_and_original_download(self):
        class Text(HTMLParser):
            def __init__(self):
                super().__init__()
                self.parts = []

            def handle_data(self, data):
                self.parts.append(data)

        source = 'within; package Public "α < > &"\n/* <comment> */\nend Public;\n'
        highlighted = highlighted_source(source)
        parser = Text()
        parser.feed(highlighted)
        self.assertEqual("".join(parser.parts), source)
        self.assertIn('id="L3"', highlighted)
        self.assertIn("&lt;comment&gt;", highlighted)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "Public").mkdir()
            original = source.replace("\n", "\r\n").encode("utf-8")
            (root / "Public/package.mo").write_bytes(original)
            output = root / "site"
            build(root, output)
            self.assertEqual((output / "raw/Public/package.mo").read_bytes(), original)
            page = (output / source_page_name(Path("Public/package.mo"))).read_text()
            self.assertIn("Public.html", page)
            self.assertIn('id="L3"', page)
            self.assertIn("Go to definition", page)

    def test_member_order_and_base_link(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "Public").mkdir()
            (root / "Public/package.mo").write_text(
                "within; package Public end Public;"
            )
            (root / "Public/package.order").write_text("Zebra\nAlpha\n")
            (root / "Public/Zebra.mo").write_text(
                'within Public; model Zebra "Zebra summary" end Zebra;'
            )
            (root / "Public/Alpha.mo").write_text(
                'within Public; model Alpha "Alpha summary" extends Zebra; end Alpha;'
            )
            output = root / "site"
            build(root, output)
            page = (output / "Public.html").read_text()
            table = page.split("<h2>Members</h2>", 1)[1]
            self.assertLess(table.index("Zebra.html"), table.index("Alpha.html"))
            self.assertIn("Zebra summary", table)
            child = (output / "Public.Alpha.html").read_text()
            self.assertIn("<h2>Base classes</h2>", child)
            self.assertIn('href="Public.Zebra.html">Zebra</a>', child)

    def test_site_excludes_nonlibrary_fixtures(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "LICENSE").write_bytes(b"License terms\n")
            (root / "NOTICE").write_bytes(b"Upstream attribution\r\n")
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
            self.assertEqual((output / "LICENSE").read_bytes(), b"License terms\n")
            self.assertEqual(
                (output / "NOTICE").read_bytes(), b"Upstream attribution\r\n"
            )
            self.assertIn('href="NOTICE">Credits</a>', landing)
            self.assertIn("Public package summary", landing)
            self.assertIn('<details id="index">', landing)
            self.assertNotIn('<details id="index" open', landing)
            self.assertIn("Public/Foo.mo#L1", page)
            self.assertIn('href="Public.html"', page)
            self.assertIn("<h2>Help</h2>", (output / "Public.html").read_text())
            self.assertFalse((output / "Hidden.html").exists())


if __name__ == "__main__":
    unittest.main()
