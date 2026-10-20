from setuptools import setup

package_name = "xmacro"

setup(
    name=package_name,
    version="1.2.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    # common.sdf.xmacro / common.urdf.xmacro are loaded relative to the module file,
    # so they must be installed next to the .py files.
    package_data={package_name: ["*.xmacro"]},
    include_package_data=True,
    install_requires=[],
    zip_safe=True,
    maintainer="PFA",
    maintainer_email="pfa@todo.todo",
    description="Vendored copy of xmacro 1.2.1 (MIT) for expanding *.xmacro files at launch time.",
    license="MIT",
)
