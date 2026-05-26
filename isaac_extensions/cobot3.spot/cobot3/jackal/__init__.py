"""Optional Clearpath Jackal secondary robot for the cobot3.spot extension.

This subpackage is intentionally self-contained. Coupling to the rest of the
extension is limited to:

  * ``spot/scene.py``: a lazy ``from .jackal.scene import add_jackal_to_world``
    inside one branch of ``setup_scene``.
  * ``spot/extension.py``: ``from .jackal.panel import build_jackal_*`` and
    three UI builder calls.

To drop Jackal entirely later, delete this folder and remove those references.
"""
