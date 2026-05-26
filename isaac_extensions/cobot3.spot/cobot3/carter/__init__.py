"""Optional Nova Carter secondary robot for the cobot3.spot extension.

Symmetric with the spot/jackal/ subpackage. Coupling to the rest of the
extension is limited to:

  * ``spot/extension.py``: ``from .carter.panel import build_carter_*`` and
    three UI builder calls.
  * ``spot/scene.py``: no Carter-specific code. The Carter panel sets
    ``sample._secondary_spawn`` before ``load_world_async()``; setup_scene
    invokes whatever callback is registered.

Drop this folder + remove the panel imports/calls in extension.py to fully
remove Carter support.
"""
