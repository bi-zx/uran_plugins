import os
from setuptools import Extension, setup

flags = ["/std:c++14"] if os.name == "nt" else ["-std=c++14"]
setup(ext_modules=[Extension("uran_navigation_plugin._terrain_rays", ["src/terrain_rays.cpp"],
                             language="c++", extra_compile_args=flags)])
