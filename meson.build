project('osb2mp4', 'cpp',
  default_options: [
    'cpp_std=c++17',
    'warning_level=3',
    'default_library=static',
    'buildtype=release',
  ])

add_global_arguments('-Wno-unsequenced', language: 'cpp')
add_global_arguments('-Wno-c11-extensions', language: 'cpp')

gifdec_proj = subproject('gifdec')
gifdec_dep = gifdec_proj.get_variable('gifdec_dep')
openmp_dep = dependency('openmp', required: false)
opencv_dep = dependency('opencv', required: true)

executable('osb2mp4', [
    'osb2mp4.cpp',
    'src/ImageReader.cpp'
  ],
  include_directories: 'lib',
  dependencies: [gifdec_dep, openmp_dep, opencv_dep])
