# Generated maps

Mapping mode writes `a2_map.pcd` into this directory when the launch process
is stopped normally with Ctrl+C. Generated PCD files are runtime artifacts and
should be backed up separately when they are important.

Terrain navigation projects the PCD into `a2_nav2_map.pgm` and
`a2_nav2_map.yaml`. The combined navigation launch regenerates these files when
the PCD is newer. The projection is used only by the global planner; live local
obstacles come from GSeg3D and the Ground Consistency costmap layer.
