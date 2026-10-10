# Model and dependency provenance

`x2_ultra.urdf` and `x2_ultra.xml` are the upstream X2 Ultra model assets already
present in this project's GitLab `v1.0` branch. The accompanying upstream notice
is retained as `x2_urdf_upstream_README.md`; it declares Mulan PSL v2 for the model.
The complete upstream SDK and original mesh trees are not distributed in this source export.
FK/IK retain the existing base arm kinematic parameters. V2.1 additionally ships a
decimated visual representation of the required arm STL meshes, with source
filenames/hashes and conversion metadata alongside the derived asset. These are
visual assets, not STEP solids or collision geometry.

Runtime dependencies include Python and NumPy; online robot operation additionally
requires the operator's existing ROS 2 / AimDK message installation. Desktop builds
use PyQt5/Qt, OpenSSH, PyInstaller, AppImage tooling and AppImage type-2 runtime. Their licenses apply to their
respective components; the delivery builder includes available distribution notices.

The AppImage embeds the statically linked `runtime-x86_64` from
`AppImage/type2-runtime` (commit `8f39b89`). It avoids a host `libfuse.so.2`
dependency. The upstream license (including bundled component links) is retained
in `licenses/AppImage-runtime` and the AppImage's `usr/licenses/AppImage-runtime`.
Runtime source: https://github.com/AppImage/type2-runtime/tree/8f39b89.

This notice does not grant a new license to third-party components or to the
project's own code. Preserve the upstream notices when redistributing model files.

The derived arm visualization meshes come from the 14 arm STL files referenced
by the user-supplied
`omnipicker_omnipicker/x2_ultra_plus_omnipicker_omnipicker.urdf`.
Source filenames, SHA-256 hashes and conversion metadata are preserved in
`desktop/assets/arm_meshes.json`; the asset README records the derivation.
These assets provide arm visualization only; the source model does not replace
the solver's base kinematics or add end-effector support.
No license for the entire user-supplied asset trees is inferred from the existing
base URDF notice. Recipients must retain source notices and the applicable rights
for any third-party assets they redistribute.
