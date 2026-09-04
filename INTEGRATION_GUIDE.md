# Integration guide: adding this radar/lidar to a bigger existing sim

This covers bolting `rmagine_gazebo_plugins`/`radarays_gazebo_plugins`'s
sensors onto an existing Gazebo Harmonic world (e.g. a USV/vehicle
simulation) that was built independently of this migration. Read
`ARCHITECTURE.md` first if you haven't.

## 0. The one hard prerequisite: Gazebo Harmonic (`gz-sim8`)

These plugins are compiled `System` shared libraries against Harmonic's
ABI (`gz::sim::systems::...`). They will not load into a Gazebo Classic or
Fortress/Garden world without a rebuild against that version's headers —
this isn't a config issue, it's a real binary compatibility constraint. If
your existing sim runs on a different Gazebo version, that's the first
thing to resolve, before anything else here applies.

## 1. Pick generic ray-sensor vs. radar-specific

- Just need a LiDAR/depth-like sensor (`/scan`, `/points`)? Use
  `rmagine_gazebo_plugins` alone — `rmagine_embree_sensor_system` (CPU) or
  `rmagine_optix_sensor_system` (GPU).
- Need the actual radar model (Fresnel reflection, per-material returns,
  Perlin clutter, polar `/radar/image`)? Add `radarays_gazebo_plugins`'s
  `radarays_embree_sensor_system`/`radarays_optix_sensor_system` on top —
  it depends on the rmagine one being present (see the working example
  below, both are attached to the same sensor model).

## 2. Add the map system once, at world level

One `rmagine_embree_map_system` (or `_optix_`) plugin per world, added as
a direct child of `<world>`, not of any particular model. It watches every
non-ignored model/link in the world and keeps a live `EmbreeMap`/`OptixMap`
in sync as things move:

```xml
<plugin name="rmagine_embree_map_system" filename="librmagine_embree_map_system.so">
  <ignore_model>my_vehicle</ignore_model>   <!-- your vehicle's own model name -->
  <update>
    <rate_limit>20</rate_limit>
    <delta_trans>0.001</delta_trans>
    <delta_rot>0.001</delta_rot>
    <delta_scale>0.001</delta_scale>
  </update>
</plugin>
```

**`<ignore_model>` matters for a real integration**: without it, your
vehicle's own hull/body becomes part of the ray-traced scene and the
sensor can self-intersect. Repeat the tag for every model that should be
fully excluded (your vehicle, any other sensor-carrying model). For
finer-grained exclusion of just one link (not a whole model), there's also
`<ignore_link>model_name::link_name</ignore_link>`.

`rate_limit`/`delta_*` control how eagerly the map re-syncs to moving
geometry — for a world with a lot of independently-moving things (waves,
other vessels, etc.) these are the knobs to tune if map rebuilding shows up
in a profile.

## 3. Attach the sensor plugin(s) to your vehicle's own SDF model

**Note (2026-08-31): rmagine's own upstream rewrite changed this shape**
after this guide was first written — `rmagine_embree_sensor_system` is no
longer a per-model plugin with flat params. It's now a single bare
world-level factory plugin (like the map system) that auto-discovers any
`<sensor type="custom" gz:type="rmagine_embree">` element anywhere in the
world; each sensor's own params (`map_key`/`frame`/`topic_scan`/
`topic_points`/`update_rate`/the lidar scan geometry) live on that
`<sensor>` element itself. `radarays_embree_sensor_system` (the radar-
image one) is untouched — still a plain per-model `<plugin>` block. This
is the exact, verified structure from
`radarays_gazebo_plugins/worlds/gz_static_radar_cpu.sdf`:

```xml
<world name="my_world">
  ...
  <!-- one factory, direct child of <world>, discovers every rmagine_embree
       <sensor> anywhere below -- same one-per-world rule as the map system -->
  <plugin name="rmagine_embree_sensor_system" filename="librmagine_embree_sensor_system.so"/>

  <model name="my_vehicle">
    ...
    <plugin name="radarays_embree_sensor_system" filename="libradarays_embree_sensor_system.so">
      <map_key>default</map_key>
      <frame>radar_link</frame>
      <topic_image>my_vehicle/radar/image</topic_image>
      <update_rate>4</update_rate>
      <min_angle>-1.0472</min_angle>
      <max_angle>1.0472</max_angle>
      <samples>400</samples>
      <range_max>100.0</range_max>
      <n_cells>1024</n_cells>
    </plugin>
    <link name="radar_link">
      ...
      <sensor name="radar_lidar_sensor" type="custom" gz:type="rmagine_embree">
        <map_key>default</map_key>
        <frame>radar_link</frame>
        <topic_scan>my_vehicle/radar/scan</topic_scan>
        <topic_points>my_vehicle/radar/points</topic_points>
        <update_rate>4</update_rate>
        <lidar>
          <scan>
            <horizontal>
              <min_angle>-1.0472</min_angle>
              <increment>0.0052486216</increment>
              <samples>400</samples>
            </horizontal>
          </scan>
          <range>
            <min>0.2</min>
            <max>100.0</max>
          </range>
        </lidar>
      </sensor>
    </link>
  </model>
</world>
```

Namespace `topic_scan`/`topic_points`/`topic_image` per-vehicle (as above)
if you might ever have more than one sensor-carrying model in the same
world — these are plain global topic names, nothing auto-namespaces them
for you. `map_key` only needs to change from `"default"` if you
deliberately want more than one independent map in the same world (e.g.
different regions); for a single vehicle in one world, leave it.

**`radar/scan` and `radar/points` are gz-transport-native, not ROS —
you need a bridge.** Unlike `radarays_embree_sensor_system`'s
`/radar/image` (published directly via `rclcpp`, nothing extra needed),
`rmagine_embree_sensor_system` publishes over native `gz::transport`
only. To get `/scan`/`/points` into ROS 2, run a `ros_gz_bridge
parameter_bridge` alongside your sim, with an explicit `GZ_TO_ROS`
direction (the plain `topic@ros_type@gz_type` CLI form defaults to
bidirectional and can create feedback-loop artifacts — not what you
want here). Real, working example, from
`rmagine_gazebo_plugins/tests/testdata/embree_harmonic/ros_gz_bridge_fixtures.yaml`:

```yaml
- ros_topic_name: "/my_vehicle/radar/scan"
  gz_topic_name: "my_vehicle/radar/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/my_vehicle/radar/points"
  gz_topic_name: "my_vehicle/radar/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS
```

then `ros2 run ros_gz_bridge parameter_bridge --ros-args -p
config_file:=<path-to-that-yaml>`, or add it as a `Node(package=
"ros_gz_bridge", executable="parameter_bridge", ...)` action in your
launch file — see `radarays_gazebo_plugins/launch/gz_static_radar_cpu.launch.py`
for a real working launch-file example of exactly this.

## 4. Give your environment real materials (this matters for a marine sim specifically)

Any `<visual>` without a `<radarays_material>` tag falls back to one
shared default material — fine for a smoke test, not for a meaningful
demo. Tag water, hulls, docks, buoys, etc. individually:

```xml
<visual name="hull_visual">
  ...
  <radarays_material>
    <velocity>0.1</velocity>
    <ambient>0.5</ambient>
    <diffuse>0.5</diffuse>
    <specular>500.0</specular>
  </radarays_material>
</visual>
```

Worth knowing: `radarays_ros/config/oru3.yaml` (the standalone-node
materials convention) already ships preset velocities for **water** and
**water/sea** among its material list — this codebase already anticipated
a marine use case, even though it's never been run against one. That's a
real, reusable starting point for tuning USV-relevant materials rather
than guessing values from scratch. Once you have a real radar/lidar
reading from your own vehicle to compare against, `radaray_opti` (see
`QUICKSTART.md` #5) can search for better values the same way it does
against the synthetic 2-wall scene — same mechanism, just pointed at your
real materials file and a real recorded comparison image instead.

## 5. GPU variant

Swap `embree` → `optix` in every plugin `filename`/`name` above
(`rmagine_optix_map_system`/`_sensor_system`,
`radarays_optix_sensor_system`) — same SDF shape, same parameters. Needs
an NVIDIA GPU + OptiX SDK on the machine actually running the sim (see
`MIGRATION_HANDOFF.md` #18 for the exact SDK version that's been verified,
7.5.0 — a newer one hit a real ABI break there).

## 6. Recommended rollout order (don't edit your live world file first)

Given this radar/lidar work was deliberately kept separate from your USV
sim throughout this whole migration, treat the actual merge as its own
small project, not a one-shot edit:

1. Copy your world file (or a small representative subset of it — one
   hull model + a patch of water + one buoy) to a scratch file.
2. Add the map system + sensor plugins there first, confirm `/radar/image`
   (or `/scan`+`/points`) looks sane in isolation — same checks as
   `QUICKSTART.md` #2-3 (rqt_image_view, `ros2 param set` live tuning).
3. Only once that's confirmed working, fold the plugin blocks into your
   real world file/vehicle SDF.

This mirrors the same "verify each piece for real before trusting it"
approach this whole migration was built on — a plugin that loads but
produces a degenerate all-zero image is a very easy failure mode to miss
if you only check "did Gazebo start without crashing."

## 7. Performance knobs, if it's ever too slow

- `samples` (rays per scan) and `n_cells` (radar image range resolution)
  are the two biggest per-frame cost drivers — both are plain SDF params
  above, no rebuild needed to change them.
- `update_rate` — no reason to run the sensor faster than whatever
  actually consumes its output needs.
- Map rebuild cost scales with total scene triangle count. A big, detailed
  hull/dock mesh is a bigger cost than the sensor resolution itself in
  most cases — worth profiling before assuming the sensor plugin is the
  bottleneck.
