# Publish the Modelica reference

The [online reference](https://cognipilot.github.io/modelica_models/) is generated
from `Documentation(info=...)` HTML in the public Modelica package trees. Edit
those annotations to change library help. Class pages also expose the original
parameter, input, output and implementation declarations, nested members and
GitHub source links. Local `modelica://` class links resolve to generated pages;
links to external libraries retain their Modelica URI for Modelica tools.

Build with Python 3.11 or later; no compiler or third-party Python package is
required:

```sh
python -m unittest tools.tests.test_build_docs
python tools/build_docs.py --output "$HOME/scratch/modelica_models/site"
python -m http.server 8000 --directory "$HOME/scratch/modelica_models/site"
```

Open `http://localhost:8000`. Use a dedicated output directory. Root directories
containing `package.mo` define public libraries; development notes, reproduction
fixtures and tooling directories are excluded. This generator extracts named
classes and their own documentation annotations; it does not flatten inherited
interfaces or evaluate Modelica expressions.

The `Modelica documentation` workflow validates extraction and builds the site
on pull requests. Pushes to `main` also upload and deploy the site using the
official GitHub Pages actions. In repository Settings → Pages, select **GitHub
Actions** as the build source. Generated HTML is a workflow artifact, never a
second editable API reference in Git.

Each class has **Documentation**, **View source** and **Download .mo** links.
Source pages show complete files with stable `#L<number>` anchors, syntax
highlighting and links back to each class's help. Raw downloads preserve the
original file bytes, including comments, Unicode and line endings. The source
browser uses the original text; browser HTML rendering normalizes line endings.

Public API tables list explicit parameters, constants, inputs, outputs and record
or connector fields, with types, dimensions, modifications, defaults and source
descriptions. Protected declarations and equation/algorithm bodies are excluded
from these tables. Members follow `package.order` when present. Base-class links
lead to inherited contracts rather than presenting a flattened interface.

Go-to-definition links use conservative lexical lookup for fully qualified names,
package ancestry and straightforward import aliases. Local component names and
replaceable package bindings suppress links through those names. Ambiguous or
unresolved references remain plain text. This is a source browser, not a compiler
semantic resolver: wildcard imports, inheritance lookup, dynamic dispatch and
evaluated dimensions/defaults require a Modelica tool.

The visual theme and original CogniPilot logo live under `tools/docs_assets/`;
brand attribution and provenance are recorded there. The generated site has no
external font, script or Python dependency.
The site includes the repository's `LICENSE` and `NOTICE`, linked from each
page's footer.
