import setuptools

with open("README.md", "r", encoding="utf8", errors="ignore") as fh:
    LONG_DESCRIPTION = fh.read()

# Packages that pyswarming uses explicitly:
INSTALL_REQUIRES = [
    "numpy",
    "numdifftools",
    "matplotlib",
]

setuptools.setup(
    name="pyswarming",
    version="1.1.4",
    author="Emerson Martins de Andrade",
    author_email="mrson@oceanica.ufrj.br",
    description="A research toolkit for Swarm Robotics",
    long_description=LONG_DESCRIPTION,
    long_description_content_type="text/markdown",
    url="https://github.com/mrsonandrade/pyswarming",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "License :: OSI Approved :: BSD License",
        "Intended Audience :: Science/Research",
        "Operating System :: OS Independent",
        "Topic :: Scientific/Engineering :: GIS",
    ],
    python_requires=">=3.7",
    install_requires=INSTALL_REQUIRES,
)
