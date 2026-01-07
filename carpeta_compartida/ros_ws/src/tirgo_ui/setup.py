from distutils.core import setup
setup(
    name='tirgo_ui',
    version='0.0.1',  # pon el mismo número en package.xml
    packages=['tirgo_ui', 'tirgo_ui.routes'],
    package_dir={'': 'src'},
)
