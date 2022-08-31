project('osb2mp4', 'cpp',
  default_options: [
    'cpp_std=c++17',
    'warning_level=3',
    'default_library=static',
    'buildtype=release',
  ])

add_global_arguments('-Wno-unsequenced', language: 'cpp')
add_global_arguments('-Wno-c11-extensions', language: 'cpp')

gifdec_dep = subproject('gifdec').get_variable('gifdec_dep')
openmp_dep = dependency('openmp', required: false)
opencv_dep = dependency('opencv', required: true)

executable('osb2mp4', [
    'src/main.cpp',
    'src/Components.cpp',
    'src/Helpers.cpp',
    'src/ImageReader.cpp',
    'src/Parser.cpp',
    'src/Storyboard.cpp',
  ],
  include_directories: 'src',
  dependencies: [gifdec_dep, openmp_dep, opencv_dep])
