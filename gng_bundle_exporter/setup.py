from setuptools import find_packages, setup

package_name = "gng_bundle_exporter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fuzzrobo",
    maintainer_email="fuzzrobo@example.com",
    description="Standalone exporter that converts ROS bag topics into HTML-ready GNG analysis JSON bundles.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gng-bundle-export = gng_bundle_exporter.cli:main",
        ],
    },
)
