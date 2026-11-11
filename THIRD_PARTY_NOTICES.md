# Asset provenance and licensing status

First-party benchmark code, synthetic scenes and project documentation: a project license has not yet been selected. Public repository visibility is recorded separately from that decision.

Third-party model files retain their upstream notices and licenses:

| Model | Upstream revision | Bundled notice |
|---|---|---|
| UR5e description and meshes | UniversalRobots/Universal_Robots_ROS2_Description, `78c5afd8bfee78a459c0265304340a85f1d44792` | [v0.1 model license](assets/ur5e/LICENSE), [rack model license](assets/ur5e_robotiq/LICENSE_UR) |
| Robotiq 2F-85 and UR adapter | robotiq/robotiq-ros, `157c4a62b66166c2bcaf04503e141a40b250fb3c` | [Robotiq license](assets/ur5e_robotiq/LICENSE_ROBOTIQ) |

The selected upstream model files are distributed with their BSD-3-Clause notices. Modifications and per-file SHA256 values are in [UR5e provenance](assets/ur5e/provenance.json) and [rack robot provenance](assets/ur5e_robotiq/provenance.json). Rack visuals were converted from DAE to OBJ; collision STL files were retained. Robotiq linkage joints are frozen at a parallel grasp or fully open for the nominal exit audit. The v0.1 gripper is a separate synthetic fixture.

Python packages are installed from the pinned requirements and retain their own licenses. TAPOM solver code and its unavailable dependencies are not included in this repository. No manufacturer endorsement is asserted.
