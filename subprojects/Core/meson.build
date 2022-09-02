project('Core', 'cpp',
  default_options: [
    'cpp_std=c++17',
    'warning_level=3',
    'default_library=static',
    'buildtype=release',
  ])

if not meson.is_subproject()
  add_global_arguments('-Wno-unsequenced', language: 'cpp')
  add_global_arguments('-Wno-c11-extensions', language: 'cpp')
  add_global_arguments('-Wno-gnu-case-range', language: 'cpp')
endif

types_dep = subproject('Types').get_variable('types_dep')

core_lib = library('core', [
    'Core/Error.cpp',
    'Core/MappedFile.cpp',
    'Core/ThreadPool.cpp',
    ], include_directories: '.',
    dependencies: types_dep)

core_dep = declare_dependency(
  link_with: core_lib,
  include_directories: '.')

cli_lib = library('cli', [
    'CLI/ArgumentParser.cpp'
  ], include_directories: '.',
  dependencies: types_dep)

cli_dep = declare_dependency(
  link_with: cli_lib,
  include_directories: '.')
