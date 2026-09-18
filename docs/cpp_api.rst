==================
C++ API Reference
==================

There is currently no C++ source in this repository — every package
under ``src/`` is a pure ``ament_python`` package, and the ``include/``
directory that ``Doxyfile`` points ``INPUT`` at does not exist. Doxygen
therefore has nothing to scan, and ``breathe`` has nothing to render.

This page is kept as a placeholder so the toctree in ``index.rst`` stays
valid. Once C++ sources are added under ``include/`` and/or a package's
``src/``, add the relevant ``breathe`` directives here, for example::

   .. doxygenindex::
      :project: feldfreund_devkit_ros

See ``CONTRIBUTING`` (or the docs workflow) for whether to keep running
the Doxygen/Breathe steps in CI while this stays empty — right now they
run every build for no output.
