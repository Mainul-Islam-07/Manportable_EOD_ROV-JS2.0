"""
Patch the installed `speexdsp` package for Python 3.12+.

The PyPI `speexdsp` wheel ships a SWIG wrapper that does `import imp`, a module
removed in Python 3.12, so `from speexdsp import EchoCanceller` raises
`ModuleNotFoundError: No module named 'imp'`.

This script rewrites the broken swig_import_helper block to a modern
`from . import _speexdsp`. It is idempotent -- safe to run repeatedly, and a
no-op if the package is already patched.

NOTE: this locates the target file via `pip show` (filesystem path), not via
importlib.util.find_spec(). find_spec() on "speexdsp.speexdsp" forces Python
to first import the speexdsp package (__init__.py does
`from .speexdsp import *`), which re-triggers the very `import imp` crash
we're trying to fix -- so the old approach could never succeed.

Run it inside the same venv where speexdsp is installed:

    source venv/bin/activate
    python fix_speexdsp.py
"""

import os
import subprocess
import sys

OLD = """from sys import version_info
if version_info >= (2,6,0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_speexdsp', [dirname(__file__)])
        except ImportError:
            import _speexdsp
            return _speexdsp
        if fp is not None:
            try:
                _mod = imp.load_module('_speexdsp', fp, pathname, description)
            finally:
                fp.close()
            return _mod
    _speexdsp = swig_import_helper()
    del swig_import_helper
else:
    import _speexdsp
del version_info"""

NEW = "from . import _speexdsp"


def find_speexdsp_file():
    """Locate speexdsp/speexdsp.py on disk without importing the package."""
    result = subprocess.run(
        [sys.executable, "-m", "pip", "show", "-f", "speexdsp"],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        sys.exit("speexdsp is not installed in this environment. "
                  "Activate the venv and `pip install speexdsp` first.")

    location = None
    files = []
    section = None
    for line in result.stdout.splitlines():
        if line.startswith("Location:"):
            location = line.split(":", 1)[1].strip()
        elif line.startswith("Files:"):
            section = "files"
        elif section == "files" and line.startswith(" "):
            files.append(line.strip())

    if not location:
        sys.exit("Could not determine install location of speexdsp via pip show.")

    # Prefer the exact file from the RECORD listing, fall back to a direct join.
    candidates = [f for f in files if f.replace("\\", "/").endswith("speexdsp/speexdsp.py")]
    if candidates:
        path = os.path.join(location, candidates[0])
    else:
        path = os.path.join(location, "speexdsp", "speexdsp.py")

    if not os.path.isfile(path):
        sys.exit(f"Expected file not found at {path}. "
                  "The package layout may differ; patch it by hand.")
    return path


def main():
    path = find_speexdsp_file()

    with open(path, "r") as f:
        src = f.read()

    if "import imp" not in src:
        print(f"Already patched (or no fix needed): {path}")
        return

    if OLD not in src:
        sys.exit(f"Could not find the expected `import imp` block in {path}. "
                  "The package version may differ; patch it by hand.")

    with open(path, "w") as f:
        f.write(src.replace(OLD, NEW))

    print(f"Patched {path}")
    print("Verifying...")
    os.system(f'{sys.executable} -c '
              f'"from speexdsp import EchoCanceller; '
              f'EchoCanceller.create(256, 2048, 16000); print(\'speexdsp OK\')"')


if __name__ == "__main__":
    main()
