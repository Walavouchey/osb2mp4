project('osb2mp4', 'cpp',
  default_options: [
    'cpp_std=c++17',
    'warning_level=3',
    'default_library=static',
    'buildtype=release',
  ])

add_global_arguments('-Wno-unsequenced', language: 'cpp')
add_global_arguments('-Wno-c11-extensions', language: 'cpp')
add_global_arguments('-Wno-gnu-case-range', language: 'cpp')

gifdec_dep = subproject('gifdec').get_variable('gifdec_dep')
openmp_dep = dependency('openmp', required: false)
opencv_dep = dependency('opencv', required: true)
libsndfile_dep = subproject('libsndfile').get_variable('libsndfile_dep')
core_proj = subproject('Core')
core_dep = core_proj.get_variable('core_dep')
cli_dep = core_proj.get_variable('cli_dep')
types_dep = subproject('Types').get_variable('types_dep')

osb2mp4_exe = executable('osb2mp4', [
    'src/main.cpp',
    'src/Animation.cpp',
    'src/Helpers.cpp',
    'src/ImageReader.cpp',
    'src/Lexer/util.cpp',
    'src/Loop.cpp',
    'src/OSB/Lexer.cpp',
    'src/OSB/OSBFile.cpp',
    'src/OSB/Parser.cpp',
    'src/OSB/Token.cpp',
    'src/Parser.cpp',
    'src/Sprite.cpp',
    'src/Storyboard.cpp',
    'src/Trigger.cpp',
  ],
  include_directories: 'src',
  dependencies: [
    core_dep,
    cli_dep,
    gifdec_dep,
    libsndfile_dep,
    opencv_dep,
    openmp_dep,
    types_dep,
  ])
