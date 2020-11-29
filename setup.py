import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="vehicle_control_tutorial",
    version="0.0.1",
    author="Christian Klauer",
    author_email="chr@gmail.com",
    description="A tutotial on vehicle dynamics and control",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/christianausb/vehicleControl",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    install_requires=[
        'numpy', 'control', 'openrtdynamics2'
    ],
    python_requires='>=3.6',
)
