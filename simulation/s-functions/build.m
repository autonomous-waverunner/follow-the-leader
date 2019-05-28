%% Build coords_gen.mex
mex -Icoords_gen/src/ coords_gen/s_coords_gen.cpp coords_gen/src/coords_gen.cpp

%% Build path_alg.mex
includes = [
    "../../code/WR/inc/"
    "../../code/WR/lib/eigen-eigen-323c052e1731/"
    "../../code/WR/lib/lp-filter"
];

if ispc
    includes = [
        includes
        "C:\OSGeo4W64\include"
    ];
end
include_str = join(arrayfun(@(path) strcat("-I", path), includes),   " ");

src_path = "../../code/WR/src/";
sources  = [
    "path_alg.cpp"
    "dist_to_path.cpp"
    "pid.cpp"
    "../lib/lp-filter/LowPassFilter.cpp"
];
sources_str = join(["./s_path_alg.cpp"; arrayfun(@(src) strcat(src_path, src), sources)], " ");

if isunix
    lib_str = "-lproj";
elseif ispc
    lib_str = "-lproj_i -lproj_5_2 -LC:\OSGeo4W64\lib";
end
compile_cmd_str = join([ "mex" lib_str include_str sources_str ]);

eval (compile_cmd_str)
