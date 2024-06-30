{
  lib,
  stdenv,
  cmake,
  hpp-gepetto-viewer,
  hpp-manipulation-corba,
  libsForQt5,
  pkg-config,
}:

stdenv.mkDerivation {
  pname = "hpp-tutorial";
  version = "5.0.0";

  src = lib.fileset.toSource {
    root = ./.;
    fileset = lib.fileset.unions [
      ./CMakeLists.txt
      ./doc
      ./include
      ./Media
      ./meshes
      ./package.xml
      ./rviz
      ./script
      ./src
      ./srdf
      ./urdf
    ];
  };

  strictDeps = true;

  nativeBuildInputs = [
    cmake
    libsForQt5.wrapQtAppsHook
    pkg-config
  ];
  buildInputs = [ libsForQt5.qtbase ];
  propagatedBuildInputs = [
    hpp-gepetto-viewer
    hpp-manipulation-corba
  ];

  doCheck = true;

  meta = {
    description = "Tutorial for humanoid path planner platform";
    homepage = "https://github.com/humanoid-path-planner/hpp_tutorial";
    license = lib.licenses.bsd2;
    maintainers = [ lib.maintainers.nim65s ];
  };
}
