"""Auto-génère data/poi.js depuis src/strategy/poi.h.

Invoqué automatiquement par PlatformIO via extra_scripts.
Extrait toutes les déclarations `const Vec2 NAME = Vec2(X, Y);` du
namespace POI{} et produit un fichier JavaScript exporting POI_COORDS.

Le fichier généré n'est PAS destiné à être édité à la main — toute
modification doit se faire dans src/strategy/poi.h.
"""
import re
import sys
from pathlib import Path

# Sous PlatformIO, le script est exécuté via SConscript et __file__ n'existe pas.
# On récupère alors la racine projet via l'env PIO ; en standalone, via __file__.
try:
    Import("env")  # type: ignore[name-defined]  # noqa: F821 — fourni par PIO
    ROOT = Path(env["PROJECT_DIR"])  # type: ignore[name-defined]  # noqa: F821
except (NameError, KeyError):
    ROOT = Path(__file__).resolve().parent.parent

POI_H  = ROOT / "src" / "strategy" / "poi.h"
POI_JS = ROOT / "data" / "poi.js"


def sync_poi():
    if not POI_H.exists():
        print(f"[sync_poi] WARN: {POI_H} introuvable, skip", file=sys.stderr)
        return

    src = POI_H.read_text(encoding="utf-8")

    # Strip comments — // ligne et /* bloc */
    src = re.sub(r"//[^\n]*", "", src)
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.DOTALL)

    # Extrait le corps du namespace POI
    m = re.search(r"namespace\s+POI\s*\{(.*)\}", src, flags=re.DOTALL)
    if not m:
        print(f"[sync_poi] ERREUR: namespace POI introuvable dans {POI_H}", file=sys.stderr)
        sys.exit(1)
    body = m.group(1)

    pois = re.findall(
        r"const\s+Vec2\s+(\w+)\s*=\s*Vec2\s*\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)\s*;",
        body,
    )

    POI_JS.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "// AUTO-GÉNÉRÉ depuis src/strategy/poi.h — NE PAS ÉDITER À LA MAIN",
        "// Régénéré à chaque `pio run` via scripts/sync_poi.py",
        "const POI_COORDS = {",
    ]
    for name, x, y in pois:
        xi = int(float(x)) if float(x).is_integer() else float(x)
        yi = int(float(y)) if float(y).is_integer() else float(y)
        lines.append(f"  {name}: [{xi}, {yi}],")
    lines.append("};")
    POI_JS.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"[sync_poi] {len(pois)} POIs -> {POI_JS.relative_to(ROOT)}")


# PlatformIO importe ce module au chargement → exécution immédiate.
sync_poi()
