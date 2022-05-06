project('gifdec', 'c',
  default_options: [
    'c_std=c11',
    'warning_level=3',
  ])

gifdec_lib = library('gifdec', 'src/gifdec.c',
  include_directories: 'lib'
  )
gifdec_dep = declare_dependency(
  link_with: gifdec_lib,
  include_directories: 'lib'
  )
