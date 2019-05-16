%% Build coords_gen.mex
mex -Is-functions/coords_gen/src/ s-functions/coords_gen/src/s_coords_gen.cpp s-functions/coords_gen/src/coords_gen.cpp s-functions/coords_gen/src/UTM.cpp

%% Build path_alg.mex
includes = [
    "s-functions/path_alg/inc/"
    "s-functions/lib/eigen-eigen-323c052e1731/"
    "s-functions/lib/lp-filter"
];

if ispc
    includes = [
        includes
        "C:\OSGeo4W64\include"
    ];
end
include_str = join(arrayfun(@(path) strcat("-I", path), includes),   " ");

src_path = "s-functions/path_alg/src/";
sources  = [
    "s_path_alg.cpp"
    "path_alg.cpp"
    "dist_to_path.cpp"
    "pid.cpp"
    "../../lib/lp-filter/LowPassFilter.cpp"
];
sources_str = join(arrayfun(@(src) strcat(src_path, src), sources), " ");

if isunix
    lib_str = "-lproj";
elseif ispc
    lib_str = "-lproj_i -lproj_5_2 -LC:\OSGeo4W64\lib";
end
compile_cmd_str = join([ "mex" lib_str include_str sources_str ]);

eval (compile_cmd_str)
